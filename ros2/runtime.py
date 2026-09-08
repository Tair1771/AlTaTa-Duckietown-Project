"""ROS 2 adapter for the shared core; wheel output is only a supervisor request.

This adapter and supervisor must share a host monotonic clock. They are NOT
a distributed wheel protocol. No native Duckietown actuation publisher exists
here until the hardware feasibility gate is approved.
"""
import json
import time
from types import SimpleNamespace as NS
import cv2
import numpy as np
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class WheelRequest:
    def __init__(self):
        self.header = NS(stamp=None)
        self.vel_left = self.vel_right = 0.


class Bridge:
    def compressed_imgmsg_to_cv2(self, msg, desired_encoding="bgr8"):
        image = cv2.imdecode(np.frombuffer(bytes(msg.data), dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("Invalid compressed image")
        return image


class Publisher:
    def __init__(self, publisher, convert=lambda msg: msg):
        self.publisher, self.convert = publisher, convert

    def publish(self, msg):
        converted = self.convert(msg)
        if converted is not None:
            self.publisher.publish(converted)

    def get_num_connections(self):
        return self.publisher.get_subscription_count()


class Runtime:
    CvBridge, CvBridgeError = Bridge, ValueError
    WheelsCmdStamped, CompressedImage, String = WheelRequest, CompressedImage, String
    Duration = staticmethod(float)

    def __init__(self, node, vehicle):
        self.node, self.vehicle = node, vehicle
        self.shutdown_hooks = []
        self.sequence = 0
        self.safety = None
        self.safety_received = 0.
        self.groups = []
        self.Time = NS(now=lambda: NS(to_sec=lambda: node.get_clock().now().nanoseconds / 1e9))
        node.create_subscription(String, f"/{vehicle}/safety/status", self.receive_safety, 1)

    def receive_safety(self, msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                self.safety, self.safety_received = value, time.monotonic()
        except (ValueError, TypeError):
            self.safety = None

    def get_param(self, name, default):
        name = name.lstrip("~")
        if name == "wheels_topic":
            default = f"/{self.vehicle}/safety/request"
        if not self.node.has_parameter(name):
            self.node.declare_parameter(name, default)
        return self.node.get_parameter(name).value

    def Publisher(self, topic, message_type, queue_size=1, latch=False):
        qos = QoSProfile(depth=queue_size, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL if latch else DurabilityPolicy.VOLATILE)
        if message_type is WheelRequest:
            if topic != f"/{self.vehicle}/safety/request":
                raise ValueError("ROS 2 accepts only the diagnostic supervisor request topic")
            return Publisher(self.node.create_publisher(String, f"/{self.vehicle}/safety/request", 1), self.request)
        return Publisher(self.node.create_publisher(message_type, topic, qos))

    def request(self, msg):
        state = self.safety
        if state is None or time.monotonic() - self.safety_received > .25:
            return None
        self.sequence += 1
        return String(data=json.dumps(dict(session=state.get("session"), epoch=state.get("epoch"),
            sequence=self.sequence, issued_monotonic=time.monotonic(),
            left=float(msg.vel_left), right=float(msg.vel_right)), allow_nan=False))

    def Subscriber(self, topic, message_type, callback, queue_size=1, **kwargs):
        group = MutuallyExclusiveCallbackGroup()
        self.groups.append(group)
        qos = QoSProfile(depth=queue_size)
        if message_type is CompressedImage:
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
            def receive(msg):
                # Preserve acquisition stamp, never replace it with receive time.
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                callback(NS(data=msg.data, format=msg.format,
                            header=NS(stamp=NS(to_sec=lambda: stamp))))
        else:
            receive = callback
        return self.node.create_subscription(message_type, topic, receive, qos, callback_group=group)

    def Timer(self, seconds, callback):
        group = MutuallyExclusiveCallbackGroup()
        self.groups.append(group)
        return self.node.create_timer(seconds, lambda: callback(None), callback_group=group)

    def on_shutdown(self, callback):
        self.shutdown_hooks.append(callback)

    def loginfo(self, message, *args):
        self.node.get_logger().info(message % args if args else message)

    def logwarn(self, message, *args):
        self.node.get_logger().warning(message % args if args else message)
