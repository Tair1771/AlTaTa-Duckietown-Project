#!/usr/bin/env python3
"""Robot-side supervisor for one deliberately authorized ground check.

This is a test harness, not an autonomous driving launcher.  It must run inside
duck2's ROS container after the normal kinematics publisher has been stopped.
The supervisor owns the emergency-stop timing and starts a separate watchdog
before releasing the stop.  A lost parent pipe makes the watchdog stop early.
"""

import argparse
import json
import math
import os
import select
import signal
import subprocess
import sys
import time


MAX_DURATION = 8.0
MAX_BASE_SPEED = 0.05
MAX_WHEEL_SPEED = 0.05
MAX_STEERING = 0.02
WATCHDOG_MARGIN = 0.25


def validate_limits(duration, base_speed, max_speed, max_steering):
    values = (duration, base_speed, max_speed, max_steering)
    if any(type(value) is bool or not math.isfinite(value) for value in values):
        raise ValueError("Ground-check limits must be finite numbers")
    if not 0 < duration <= MAX_DURATION:
        raise ValueError("Ground-check duration must be in (0, 8.0] seconds")
    if not 0 < base_speed <= MAX_BASE_SPEED:
        raise ValueError("Ground-check base speed exceeds 0.05")
    if not base_speed <= max_speed <= MAX_WHEEL_SPEED:
        raise ValueError("Ground-check wheel cap must be between base speed and 0.05")
    if not 0 <= max_steering <= MAX_STEERING:
        raise ValueError("Ground-check steering cap exceeds 0.02")


def wait_until(deadline, monotonic=time.monotonic, sleep=time.sleep):
    """Wait only until an absolute monotonic deadline."""
    while True:
        remaining = deadline - monotonic()
        if remaining <= 0:
            return
        sleep(min(remaining, 0.01))


def summarize_samples(samples, released_at, stopped_at):
    active = [(left, right) for stamp, left, right in samples
              if released_at <= stamp < stopped_at
              and (abs(left) > 1e-7 or abs(right) > 1e-7)]
    final = [(left, right) for stamp, left, right in samples if stamp >= stopped_at]
    return {
        "nonzero_executed_samples": len(active),
        "executed_left_range": ([min(v[0] for v in active), max(v[0] for v in active)]
                                if active else None),
        "executed_right_range": ([min(v[1] for v in active), max(v[1] for v in active)]
                                 if active else None),
        "post_stop_samples": len(final),
        "post_stop_all_zero": bool(final) and all(
            abs(left) <= 1e-7 and abs(right) <= 1e-7 for left, right in final
        ),
    }


def latest_samples_zero(samples, count=8):
    """Require recent driver feedback to confirm the completed stop sequence."""
    recent = samples[-count:]
    return len(recent) == count and all(
        abs(left) <= 1e-7 and abs(right) <= 1e-7
        for _, left, right in recent
    )


def ros_types():
    import rosgraph
    import rospy
    from duckietown_msgs.msg import BoolStamped, WheelEncoderStamped, WheelsCmdStamped
    from std_msgs.msg import String
    return rosgraph, rospy, BoolStamped, WheelEncoderStamped, WheelsCmdStamped, String


def publishers(rospy, BoolStamped, WheelsCmdStamped, vehicle):
    stop = rospy.Publisher(
        "/{}/wheels_driver_node/emergency_stop".format(vehicle),
        BoolStamped,
        queue_size=1,
        latch=True,
    )
    wheels = rospy.Publisher(
        "/{}/wheels_driver_node/wheels_cmd".format(vehicle),
        WheelsCmdStamped,
        queue_size=1,
    )
    return stop, wheels


def publish_estop(rospy, BoolStamped, publisher, value, repeats=4):
    for _ in range(repeats):
        message = BoolStamped()
        message.header.stamp = rospy.Time.now()
        message.data = bool(value)
        publisher.publish(message)
        time.sleep(0.02)


def publish_stop(rospy, BoolStamped, WheelsCmdStamped, stop, wheels, repeats=8):
    for _ in range(repeats):
        now = rospy.Time.now()
        stopped = BoolStamped()
        stopped.header.stamp = now
        stopped.data = True
        zero = WheelsCmdStamped()
        zero.header.stamp = now
        zero.vel_left = 0.0
        zero.vel_right = 0.0
        stop.publish(stopped)
        wheels.publish(zero)
        time.sleep(0.02)


def watchdog_main(args):
    _, rospy, BoolStamped, _, WheelsCmdStamped, _ = ros_types()
    rospy.init_node("duck2_ground_watchdog", anonymous=True, disable_signals=True)
    stop, wheels = publishers(rospy, BoolStamped, WheelsCmdStamped, args.vehicle)
    time.sleep(0.3)
    publish_stop(rospy, BoolStamped, WheelsCmdStamped, stop, wheels)
    print("WATCHDOG_READY", flush=True)
    command = sys.stdin.readline()
    if command.strip() != "GO":
        publish_stop(rospy, BoolStamped, WheelsCmdStamped, stop, wheels)
        print("WATCHDOG_STOPPED_PARENT_LOST", flush=True)
        return 2
    wait_until(time.monotonic() + args.watchdog_delay)
    publish_stop(rospy, BoolStamped, WheelsCmdStamped, stop, wheels)
    print("WATCHDOG_STOPPED_AT_DEADLINE", flush=True)
    return 0


def wait_for_watchdog(watchdog, timeout=8.0):
    ready, _, _ = select.select([watchdog.stdout], [], [], timeout)
    if not ready:
        raise RuntimeError("Independent stop watchdog did not become ready")
    line = watchdog.stdout.readline().strip()
    if line != "WATCHDOG_READY":
        raise RuntimeError("Independent stop watchdog failed: {}".format(line))


def supervisor_main(args):
    validate_limits(args.duration, args.base_speed, args.max_speed, args.max_steering)
    if not args.acknowledge_upright_clear:
        raise ValueError("Ground motion requires --acknowledge-upright-clear")
    if not os.path.isfile(args.node_path):
        raise ValueError("Lane-follower source is missing: {}".format(args.node_path))

    rosgraph, rospy, BoolStamped, WheelEncoderStamped, WheelsCmdStamped, String = ros_types()
    rospy.init_node("duck2_bounded_ground_supervisor", anonymous=False, disable_signals=True)
    stop, wheels = publishers(rospy, BoolStamped, WheelsCmdStamped, args.vehicle)
    requested, executed, left_ticks, right_ticks, statuses = [], [], [], [], []
    wheel_topic = "/{}/wheels_driver_node/wheels_cmd".format(args.vehicle)
    rospy.Subscriber(wheel_topic, WheelsCmdStamped,
                     lambda m: requested.append((time.monotonic(), float(m.vel_left), float(m.vel_right))),
                     queue_size=100)
    rospy.Subscriber(wheel_topic + "_executed", WheelsCmdStamped,
                     lambda m: executed.append((time.monotonic(), float(m.vel_left), float(m.vel_right))),
                     queue_size=100)
    rospy.Subscriber("/{}/left_wheel_encoder_node/tick".format(args.vehicle), WheelEncoderStamped,
                     lambda m: left_ticks.append(int(m.data)), queue_size=100)
    rospy.Subscriber("/{}/right_wheel_encoder_node/tick".format(args.vehicle), WheelEncoderStamped,
                     lambda m: right_ticks.append(int(m.data)), queue_size=100)
    def status_callback(message):
        try:
            statuses.append(json.loads(message.data))
        except (TypeError, ValueError):
            pass
    rospy.Subscriber("/{}/lane_follower/status".format(args.vehicle), String,
                     status_callback, queue_size=10)

    lane = None
    watchdog = None
    released_at = None
    stopped_at = None
    try:
        time.sleep(0.3)
        publish_stop(rospy, BoolStamped, WheelsCmdStamped, stop, wheels)
        environment = os.environ.copy()
        environment["VEHICLE_NAME"] = args.vehicle
        lane_log = open(args.lane_log, "w")
        lane_args = [
            sys.executable, args.node_path,
            "_drive_enabled:=true", "_show_debug:=false",
            "_camera_topic:=/{}/camera_node/image/compressed".format(args.vehicle),
            "_wheels_topic:=" + wheel_topic,
            "_base_speed:={}".format(args.base_speed),
            "_max_speed:={}".format(args.max_speed),
            "_max_steering:={}".format(args.max_steering),
            "_lost_speed:=0.0", "_route_enabled:=false",
            "_obstacle_enabled:=false", "_avoidance_enabled:=false",
            "_avoidance_speed:={}".format(args.base_speed),
            "_junction_speed:={}".format(args.base_speed),
            "_junction_bias:={}".format(min(0.025, args.base_speed)),
        ]
        lane = subprocess.Popen(lane_args, env=environment, stdout=lane_log,
                                stderr=subprocess.STDOUT)

        deadline = time.monotonic() + 10.0
        topic_publishers = []
        while time.monotonic() < deadline:
            state = rosgraph.Master(rospy.get_name()).getSystemState()
            topic_publishers = dict(state[0]).get(wheel_topic, [])
            valid_status = statuses and statuses[-1].get("camera_valid")
            lane_ready = any(abs(left) > 1e-7 or abs(right) > 1e-7
                             for _, left, right in requested)
            if "/lane_follower_node" in topic_publishers and valid_status and lane_ready:
                break
            if lane.poll() is not None:
                raise RuntimeError("Lane follower exited during preflight")
            time.sleep(0.05)
        expected = {rospy.get_name(), "/lane_follower_node"}
        if set(topic_publishers) != expected:
            raise RuntimeError("Wheel topic is not exclusive: {}".format(topic_publishers))
        if not statuses or not statuses[-1].get("camera_valid"):
            raise RuntimeError("No valid camera status before release")
        if statuses[-1].get("red_stop") or statuses[-1].get("lane_error") is None:
            raise RuntimeError("Scene is not suitable for a straight ground check")
        if not executed:
            raise RuntimeError("No executed-wheel feedback during stopped preflight")
        if any(abs(left) > 1e-7 or abs(right) > 1e-7
               for _, left, right in executed[-8:]):
            raise RuntimeError("Emergency stop did not hold zero during preflight")

        if args.preflight_only:
            print("PREFLIGHT_COMPLETE", flush=True)
            return 0

        watchdog_args = [
            sys.executable, os.path.abspath(__file__), "--watchdog",
            "--vehicle", args.vehicle,
            "--watchdog-delay", str(args.duration + WATCHDOG_MARGIN),
        ]
        watchdog = subprocess.Popen(
            watchdog_args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, universal_newlines=True, bufsize=1,
            start_new_session=True,
        )
        wait_for_watchdog(watchdog)
        watchdog.stdin.write("GO\n")
        watchdog.stdin.flush()

        released_at = time.monotonic()
        publish_estop(rospy, BoolStamped, stop, False)
        wait_until(released_at + args.duration)
        stopped_at = time.monotonic()
        publish_stop(rospy, BoolStamped, WheelsCmdStamped, stop, wheels)
        print("MOTION_STOPPED", flush=True)

        lane.send_signal(signal.SIGINT)
        lane.wait(timeout=8)
        time.sleep(0.35)
        publish_stop(rospy, BoolStamped, WheelsCmdStamped, stop, wheels)
        summary = summarize_samples(executed, released_at, stopped_at)
        summary.update({
            "requested_duration_s": args.duration,
            "measured_stop_deadline_s": stopped_at - released_at,
            "left_encoder_delta": (left_ticks[-1] - left_ticks[0]
                                   if len(left_ticks) > 1 else None),
            "right_encoder_delta": (right_ticks[-1] - right_ticks[0]
                                    if len(right_ticks) > 1 else None),
            "camera_valid": bool(statuses and statuses[-1].get("camera_valid")),
            "lane_diagnostic": statuses[-1].get("lane_diagnostic") if statuses else None,
            "final_feedback_all_zero": latest_samples_zero(executed),
        })
        print(json.dumps(summary, sort_keys=True), flush=True)
        if not summary["final_feedback_all_zero"]:
            raise RuntimeError("Executed-wheel feedback was not zero after stop")
        print("SUPERVISOR_COMPLETE", flush=True)
        return 0
    finally:
        publish_stop(rospy, BoolStamped, WheelsCmdStamped, stop, wheels)
        if lane is not None and lane.poll() is None:
            lane.send_signal(signal.SIGINT)
            try:
                lane.wait(timeout=5)
            except subprocess.TimeoutExpired:
                lane.kill()
                lane.wait(timeout=3)
        if watchdog is not None:
            if watchdog.stdin:
                try:
                    watchdog.stdin.close()
                except OSError:
                    pass
            try:
                watchdog.wait(timeout=5)
            except subprocess.TimeoutExpired:
                watchdog.kill()
                watchdog.wait(timeout=3)
        if "lane_log" in locals():
            lane_log.close()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", default="duck2")
    parser.add_argument("--node-path")
    parser.add_argument("--lane-log", default="/tmp/duck2-ground-lane.log")
    parser.add_argument("--duration", type=float, default=0.8)
    parser.add_argument("--base-speed", type=float, default=0.04)
    parser.add_argument("--max-speed", type=float, default=0.05)
    parser.add_argument("--max-steering", type=float, default=0.01)
    parser.add_argument("--acknowledge-upright-clear", action="store_true")
    parser.add_argument("--preflight-only", action="store_true",
                        help="Validate a stopped setup without releasing emergency stop")
    parser.add_argument("--watchdog", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--watchdog-delay", type=float, default=1.05,
                        help=argparse.SUPPRESS)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.watchdog:
        if not 0 < args.watchdog_delay <= MAX_DURATION + WATCHDOG_MARGIN:
            raise ValueError("Invalid watchdog delay")
        return watchdog_main(args)
    return supervisor_main(args)


if __name__ == "__main__":
    sys.exit(main())
