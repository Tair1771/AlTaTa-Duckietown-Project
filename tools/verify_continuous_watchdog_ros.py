"""Production launcher fault checks; only run via the isolated bench runner."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


def main():
    if os.environ.get("DUCK2_ISOLATED_BENCH") != "1":
        raise RuntimeError("Use isolated_bench_check.py --production-watchdog")
    if os.environ.get("ROS_MASTER_URI") != "http://localhost:11311":
        raise RuntimeError("An isolated ROS master is required")
    import cv2
    import numpy as np
    import rospy
    from sensor_msgs.msg import CompressedImage
    from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped, BoolStamped
    from chat_core import RobotTransport
    from prepare_companion_driving import launch_text

    log = open("/tmp/production-watchdog.log", "w")
    core = subprocess.Popen(["roscore"], stdout=log, stderr=subprocess.STDOUT)
    controller = None
    finish = threading.Event()
    def stop_process(process, group=False):
        if process.poll() is not None:
            return
        if group:
            os.killpg(process.pid, signal.SIGINT)
        else:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(8)
        except subprocess.TimeoutExpired:
            if group:
                os.killpg(process.pid, signal.SIGKILL)
            else:
                process.kill()
            process.wait(5)
    try:
        time.sleep(3)
        rospy.init_node("isolated_fake_hardware", disable_signals=True)
        root = "/duck2/"
        camera = rospy.Publisher(root+"camera_node/image/compressed", CompressedImage, queue_size=1)
        executed = rospy.Publisher(root+"wheels_driver_node/wheels_cmd_executed", WheelsCmdStamped, queue_size=1)
        encoders = [rospy.Publisher(root+side+"_wheel_encoder_node/tick", WheelEncoderStamped, queue_size=1)
                    for side in ("left", "right")]
        state = {"request": [0., 0.], "advance": True, "emergency": []}
        rospy.Subscriber(root+"wheels_driver_node/wheels_cmd", WheelsCmdStamped,
                         lambda m: state.update(request=[m.vel_left, m.vel_right]), queue_size=1)
        rospy.Subscriber(root+"wheels_driver_node/emergency_stop", BoolStamped,
                         lambda m: state["emergency"].append(bool(m.data)), queue_size=10)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(frame, (160, 240), (180, 475), (0, 255, 255), -1)
        cv2.rectangle(frame, (470, 240), (490, 475), (255, 255, 255), -1)
        _, jpeg = cv2.imencode(".jpg", frame)
        def hardware():
            ticks = [0, 0]
            while not finish.is_set():
                image = CompressedImage()
                image.header.stamp = rospy.Time.now()
                image.format, image.data = "jpeg", jpeg.tobytes()
                camera.publish(image)
                feedback = WheelsCmdStamped()
                feedback.header.stamp = rospy.Time.now()
                feedback.vel_left, feedback.vel_right = state["request"]
                executed.publish(feedback)
                for i, publisher in enumerate(encoders):
                    if state["advance"] and abs(state["request"][i]) >= .07:
                        ticks[i] += 1
                    message = WheelEncoderStamped()
                    message.header.stamp = rospy.Time.now()
                    message.data, message.resolution, message.type = ticks[i], 135, 1
                    publisher.publish(message)
                finish.wait(.05)
        threading.Thread(target=hardware, daemon=True).start()
        script = launch_text().replace("/app", "/project/packages/duckie_lane_follower/src")
        script = script.replace("ROS_HOSTNAME=duck2.local", "ROS_HOSTNAME=localhost")
        launch = Path("/tmp/production-launch.sh")
        launch.write_text(script)
        transport = RobotTransport()

        def wait(check, label, seconds=12):
            deadline = time.monotonic()+seconds
            while time.monotonic() < deadline:
                try:
                    if check():
                        return
                except (OSError, RuntimeError, ValueError):
                    pass
                time.sleep(.1)
            raise AssertionError(label)

        for fault in ("encoder_stall", "competing_publisher", "controller_crash"):
            state.update(request=[0., 0.], advance=True, emergency=[])
            controller = subprocess.Popen(["bash", str(launch)], stdout=log, stderr=subprocess.STDOUT,
                                          start_new_session=True)
            wait(lambda: transport.poll_status().get("camera_valid") is True, "fresh startup", 30)
            initial = transport.poll_status()
            assert initial["state"] == "awaiting_route" and initial["manual_stop"]
            assert initial["wheel_speeds"] == [0., 0.]
            assert initial["wheel_publishers"] == ["/lane_follower_node"]
            epoch = initial["control_epoch"]
            for action, options in (("set_route", {"route": ["A", "B"], "position_confirmed": True}),
                                    ("continue", {})):
                ack = transport.send(action, expected_control_epoch=epoch, **options)
                assert ack["accepted"], ack
            heartbeats_end = threading.Event()
            def heartbeat():
                while not heartbeats_end.wait(.15):
                    try:
                        transport.heartbeat()
                    except Exception:
                        pass
            worker = threading.Thread(target=heartbeat, daemon=True)
            worker.start()
            competitor = None
            try:
                wait(lambda: max(state["request"]) > .07, "simulated movement")
                time.sleep(2)
                assert controller.poll() is None and not any(state["emergency"]), "healthy run stopped"
                assert max(state["request"]) > .07
                if fault == "encoder_stall":
                    state["advance"] = False
                elif fault == "competing_publisher":
                    # Advertising is sufficient; no second wheel command is needed.
                    competitor = subprocess.Popen(["python3", "-c",
                        "import rospy; from duckietown_msgs.msg import WheelsCmdStamped; "
                        "rospy.init_node('isolated_competitor'); "
                        "p=rospy.Publisher('/duck2/wheels_driver_node/wheels_cmd',WheelsCmdStamped,queue_size=1); rospy.spin()"],
                        stdout=log, stderr=subprocess.STDOUT)
                else:
                    import xmlrpc.client
                    _, _, uri = rospy.get_master().lookupNode("/lane_follower_node")
                    _, _, pid = xmlrpc.client.ServerProxy(uri).getPid("/isolated_fake_hardware")
                    os.kill(pid, signal.SIGKILL)
                wait(lambda: any(state["emergency"]), fault+" emergency-stop delivery", 8)
                wait(lambda: controller.poll() is not None, fault+" launcher cleanup", 8)
                print("PASS production launcher: stopped startup, healthy simulated motion, "
                      + fault+" emergency stop and process exit", flush=True)
            finally:
                heartbeats_end.set()
                worker.join(2)
                if competitor is not None:
                    competitor.terminate()
                    competitor.wait(5)
                if controller.poll() is None:
                    stop_process(controller, group=True)
                controller = None
            time.sleep(2)
        print("PRODUCTION_WATCHDOG_PASS", flush=True)
    finally:
        finish.set()
        if controller is not None and controller.poll() is None:
            stop_process(controller, group=True)
        stop_process(core)
        log.close()
        print(Path("/tmp/production-watchdog.log").read_text(), flush=True)


if __name__ == "__main__":
    main()
