"""Release the installed driver's stop latch only during stopped preparation."""
import json
import time
import urllib.request


def require_stopped(status):
    if (status.get("state") != "awaiting_route" or status.get("manual_stop") is not True
            or status.get("drive_enabled") is not True
            or status.get("wheel_speeds") != [0, 0]
            or status.get("wheel_publishers") != ["/lane_follower_node"]):
        raise RuntimeError("Refusing driver release outside stopped, exclusive preparation")


def main():
    import rospy
    from duckietown.dtros import DTROS, NodeType
    from duckietown_msgs.msg import BoolStamped, WheelsCmdStamped
    node = DTROS(node_name="duck2_stopped_driver_preparation", node_type=NodeType.GENERIC)
    topic = "/duck2/wheels_driver_node/"
    pub = rospy.Publisher(topic + "emergency_stop", BoolStamped, queue_size=1)
    stop_seen = []
    def observe_stop(message):
        if message.data:
            stop_seen.append(True)
    observer = rospy.Subscriber(topic + "emergency_stop", BoolStamped, observe_stop, queue_size=10)

    def check():
        with urllib.request.urlopen("http://127.0.0.1:8765/status", timeout=2) as response:
            require_stopped(json.load(response))
        code, _, state = rospy.get_master().getSystemState()
        publishers = dict(state[0]).get(topic + "emergency_stop", [])
        # The installed HTTP API permanently advertises this publisher so its
        # emergency button remains available. Do not stop or disable that API.
        allowed = {rospy.get_name(), "/duck2/robot_http_api_node",
                   "/duck2_continuous_safety_watchdog"}
        if code != 1 or not set(publishers).issubset(allowed) or stop_seen:
            raise RuntimeError("Unknown stop owner or an active emergency-stop request")
        for name in ("wheels_cmd", "wheels_cmd_executed"):
            message = rospy.wait_for_message(topic + name, WheelsCmdStamped, timeout=2)
            if message.vel_left != 0 or message.vel_right != 0:
                raise RuntimeError("Wheel request/feedback is not zero")

    deadline = time.monotonic() + 5
    while pub.get_num_connections() == 0 and time.monotonic() < deadline:
        time.sleep(0.05)
    if pub.get_num_connections() == 0:
        raise RuntimeError("Driver stop subscriber unavailable")
    check()
    released = False
    try:
        for _ in range(3):
            check()
            msg = BoolStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = False
            released = True
            pub.publish(msg)
            time.sleep(0.1)
        check()
        print("Driver stop release sent; controller remains awaiting_route and zero feedback verified.")
    except BaseException:
        if released:
            for _ in range(3):
                msg = BoolStamped()
                msg.header.stamp = rospy.Time.now()
                msg.data = True
                pub.publish(msg)
                time.sleep(0.1)
        raise
    rospy.signal_shutdown("Stopped preparation complete")


if __name__ == "__main__":
    main()
