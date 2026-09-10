#!/usr/bin/env python3
"""Independent fail-stop monitor for an active continuous-route session."""
import json
import threading
import time

import rospy
from duckietown_msgs.msg import BoolStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import String

from duckie_lane_follower.continuous_safety import SafetyState


def main():
    rospy.init_node("duck2_continuous_safety_watchdog")
    vehicle = rospy.get_param("~vehicle", "duck2")
    root = "/%s/" % vehicle
    wheel_topic = root + "wheels_driver_node/wheels_cmd"
    state = SafetyState(time.monotonic())
    lock = threading.Lock()
    stop = rospy.Publisher(root + "wheels_driver_node/emergency_stop",
                           BoolStamped, queue_size=1, latch=True)

    def status(message):
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                with lock:
                    state.record_status(time.monotonic(), value)
        except (TypeError, ValueError):
            pass

    def wheels(message, executed=False):
        with lock:
            method = state.record_executed if executed else state.record_request
            method(time.monotonic(), message.vel_left, message.vel_right)

    def encoder(message, side):
        try:
            with lock:
                state.record_encoder(time.monotonic(), side, message.data)
        except (AttributeError, TypeError, ValueError, OverflowError):
            pass

    rospy.Subscriber(root + "lane_follower/status", String, status, queue_size=10)
    rospy.Subscriber(wheel_topic, WheelsCmdStamped, wheels, queue_size=20)
    rospy.Subscriber(root + "wheels_driver_node/wheels_cmd_executed",
                     WheelsCmdStamped, lambda msg: wheels(msg, True), queue_size=20)
    rospy.Subscriber(root + "left_wheel_encoder_node/tick", WheelEncoderStamped,
                     lambda msg: encoder(msg, "left"), queue_size=20)
    rospy.Subscriber(root + "right_wheel_encoder_node/tick", WheelEncoderStamped,
                     lambda msg: encoder(msg, "right"), queue_size=20)

    stopped = [False]
    def emergency(reason):
        if stopped[0]:
            return
        stopped[0] = True
        rospy.logerr("Continuous safety stop: %s", reason)
        for _ in range(5):
            message = BoolStamped()
            message.header.stamp = rospy.Time.now()
            message.data = True
            stop.publish(message)
            time.sleep(0.02)

    rospy.on_shutdown(lambda: emergency("watchdog process ended"))
    rate = rospy.Rate(10)
    ownership_bad_since = None
    while not rospy.is_shutdown():
        try:
            code, message, system = rospy.get_master().getSystemState()
            if code != 1:
                raise RuntimeError(message)
            publishers = sorted(dict(system[0]).get(wheel_topic, []))
            ownership_bad_since = (None if publishers == ["/lane_follower_node"]
                                   else ownership_bad_since or time.monotonic())
            checked_publishers = (publishers if ownership_bad_since is None
                                  or time.monotonic() - ownership_bad_since >= 0.3
                                  else ["/lane_follower_node"])
            with lock:
                reason = state.fault(time.monotonic(), checked_publishers)
        except Exception as error:
            reason = "ROS safety inspection failed: %s" % error
        if reason:
            emergency(reason)
            rospy.signal_shutdown(reason)
            break
        rate.sleep()


if __name__ == "__main__":
    main()
