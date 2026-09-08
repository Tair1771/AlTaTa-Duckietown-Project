#!/usr/bin/env python3
"""Independent ROS 2 diagnostic wheel lease supervisor; no hardware publisher."""
import json
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from std_srvs.srv import SetBool
from safety import SafetyGate
import lifecycle


class Supervisor(Node):
    def __init__(self):
        super().__init__("duck2_safety")
        vehicle = os.environ.get("VEHICLE_NAME", "duck2")
        self.gate = SafetyGate(time.monotonic)
        self.last_status, self.received = {}, 0.
        self.declare_parameter("source_timestamp_verified", False)
        self.output_pub = self.create_publisher(String, f"/{vehicle}/safety/diagnostic_wheels", 1)
        self.state_pub = self.create_publisher(String, f"/{vehicle}/safety/status", 1)
        self.create_subscription(String, f"/{vehicle}/safety/request", self.request, 1)
        self.create_subscription(String, f"/{vehicle}/lane_follower/status", self.status, 1)
        self.create_subscription(String, f"/{vehicle}/lane_follower/command", self.command, 10)
        self.create_service(SetBool, f"/{vehicle}/safety/arm", self.arm)
        self.create_timer(.02, self.tick)

    def request(self, msg):
        try:
            self.gate.accept(json.loads(msg.data))
        except (ValueError, TypeError):
            self.gate.stop("malformed_request")

    def command(self, msg):
        # Stop does not depend on perception, timestamps, or a fresh heartbeat.
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict) and value.get("action") == "stop":
                self.gate.stop("operator_stop")
                self.tick()
        except (ValueError, TypeError):
            pass  # Full command validation/ack remains in the shared core.

    def status(self, msg):
        try:
            status = json.loads(msg.data)
            if isinstance(status, dict):
                self.last_status, self.received = status, time.monotonic()
        except (ValueError, TypeError):
            self.gate.stop("invalid_status")

    def ready(self):
        return (self.get_parameter("source_timestamp_verified").value is True
                and time.monotonic() - self.received < .25
                and self.last_status.get("camera_valid") is True
                and self.last_status.get("drive_enabled") is True
                and self.last_status.get("manual_stop") is False)

    def arm(self, request, response):
        if request.data:
            response.success = self.gate.arm(self.ready())
        else:
            self.gate.stop("operator_stop")
            response.success = True
        response.message = self.gate.reason + "; diagnostic output only"
        return response

    def tick(self):
        if self.gate.armed and not self.ready():
            self.gate.stop("source_not_ready")
        left, right = self.gate.output()
        self.output_pub.publish(String(data=json.dumps(dict(left=left, right=right))))
        state = self.gate.status()
        state.update(source_ready=self.ready(), status_age=time.monotonic()-self.received)
        self.state_pub.publish(String(data=json.dumps(state)))


def main():
    lifecycle.init()
    node = Supervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        lifecycle.begin_cleanup()
        if rclpy.ok():
            node.gate.stop("shutdown")
            node.tick()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
