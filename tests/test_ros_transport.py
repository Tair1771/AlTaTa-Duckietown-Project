"""Run inside the project image with --network none; no real robot connection."""
import os
from pathlib import Path
import tempfile
import json
import uuid
import signal
import subprocess
import sys
import time
import xmlrpc.client
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped

os.environ.update(ROS_MASTER_URI="http://localhost:11311", ROS_HOSTNAME="localhost",
                  VEHICLE_NAME="duck2")
master = subprocess.Popen(["roscore"], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
child = None
gateway_process = None
capture_process = None
log = open("/tmp/lane-integration.log", "w")
try:
    deadline = time.monotonic() + 15
    while True:
        try:
            xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"]).getUri("/integration")
            break
        except OSError:
            if time.monotonic() > deadline:
                raise RuntimeError("ROS master startup timed out")
            time.sleep(.1)
    rospy.init_node("lane_integration", disable_signals=True)
    received = []
    sub = rospy.Subscriber("/duck2/wheels_driver_node/wheels_cmd", WheelsCmdStamped,
                           lambda m: received.append((m.vel_left, m.vel_right)), queue_size=100)
    camera = rospy.Publisher("/duck2/camera_node/image/compressed", CompressedImage, queue_size=1)
    latest_status = {}
    status_sub = rospy.Subscriber("/duck2/lane_follower/status", String,
                                  lambda m: latest_status.update(json.loads(m.data)), queue_size=10)
    commands = rospy.Publisher("/duck2/lane_follower/command", String, queue_size=10)
    def command(action, **kwargs):
        identifier = str(uuid.uuid4())
        payload = dict(id=identifier, action=action, issued_at=time.time())
        payload.update(kwargs)
        commands.publish(String(data=json.dumps(payload)))
        deadline = time.monotonic()+3
        while time.monotonic()<deadline:
            ack = latest_status.get("last_command") or {}
            if ack.get("id") == identifier:
                assert ack["accepted"], ack
                return
            time.sleep(.02)
        raise AssertionError("Command acknowledgment timed out")
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(image, (160,240), (180,455), (0,255,255), -1)
    cv2.rectangle(image, (470,240), (490,455), (255,255,255), -1)
    def stream(img, seconds, stamp=None):
        ok, data = cv2.imencode(".jpg", img)
        assert ok
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            m = CompressedImage()
            m.header.stamp = rospy.Time.now() if stamp is None else stamp()
            m.format, m.data = "jpeg", data.tobytes()
            camera.publish(m)
            time.sleep(.05)

    def start(drive, extra=None):
        global child
        path = "/code/catkin_ws/src/duckiebot-ros/packages/duckie_lane_follower/src/lane_follower_node.py"
        child = subprocess.Popen([sys.executable, path, "_drive_enabled:="+drive,
                                  "_show_debug:=false"] + (extra or []), stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic()+15
        while camera.get_num_connections() == 0:
            if child.poll() is not None:
                raise RuntimeError("Node exited during startup")
            if time.monotonic()>deadline:
                raise RuntimeError("Camera subscription timed out")
            time.sleep(.1)
        time.sleep(.3)
        received.clear()

    def stop():
        child.send_signal(signal.SIGINT)
        child.wait(timeout=10)
        time.sleep(.2)

    start("false")
    stream(image, 1.)
    assert received and all(x == (0.,0.) for x in received), received[-5:]
    stop()
    print("PASS actual ROS: debug publishes only zeros", flush=True)

    start("true")
    stream(image, 1.5)
    assert any(max(x)>0 for x in received), "No driving output in isolated test"
    time.sleep(.8)
    assert received[-1] == (0.,0.), "Camera timeout failed"
    stream(image, 1.)
    assert max(received[-1])>0, "Fresh camera did not recover"
    for label, timestamp in [
            ("zero", lambda: rospy.Time(0)),
            ("stale", lambda: rospy.Time.now()-rospy.Duration(1)),
            ("future", lambda: rospy.Time.now()+rospy.Duration(1))]:
        stream(image, .2, timestamp)
        received.clear()
        stream(image, .3, timestamp)
        assert received and all(x == (0., 0.) for x in received), label
        assert latest_status["camera_valid"] is False, latest_status
        stream(image, .4)
        assert max(received[-1]) > 0, "Recovery failed after " + label
    fixed_stamp = rospy.Time.now()
    stream(image, .2, lambda: fixed_stamp)
    received.clear()
    stream(image, .2, lambda: fixed_stamp)
    assert received and all(x == (0., 0.) for x in received), "Duplicate images drove"
    received.clear()
    stream(image, .2, lambda: fixed_stamp-rospy.Duration(.01))
    assert received and all(x == (0., 0.) for x in received), "Out-of-order images drove"
    stream(image, .4)
    assert max(received[-1]) > 0, "Fresh camera did not recover after timestamp rejection"
    print("PASS actual ROS: zero, stale, future, duplicate and out-of-order timestamps stop output",
          flush=True)
    received.clear()
    stop()
    assert received and received[-1] == (0.,0.), "SIGINT zero message not delivered"
    print("PASS actual ROS: camera timeout, recovery and SIGINT zero delivery", flush=True)

    start("true")
    stream(image, .8)
    red = image.copy()
    cv2.rectangle(red, (260,370), (580,390), (0,0,255), -1)
    stream(red, .4)
    assert received[-1] == (0.,0.), "Red line did not stop"
    received.clear()
    stream(image, .5)
    assert received and all(x == (0.,0.) for x in received), "Stop latch released"
    stop()
    print("PASS actual ROS: JPEG red-line detection and persistent stop", flush=True)

    start("true", ["_route_enabled:=true", "_junctions_calibrated:=true"])
    stream(image, .5)
    assert received and all(x == (0.,0.) for x in received), "Unconfirmed route drove"
    command("set_route", route=["A","D","C","E","A"], position_confirmed=True)
    command("continue")
    stream(image, .6)
    assert max(received[-1]) > 0
    command("slow_down")
    assert abs(latest_status["speed_scale"]-.8)<.001
    command("turn", value="right")
    assert latest_status["route"] == ["A","D","B","C","E","A"]
    stream(red, .3)
    assert received[-1] == (0.,0.)
    stream(red, 2.1)
    assert latest_status["state"] == "crossing", latest_status
    stream(np.zeros_like(image), 3.0)
    assert latest_status["route_index"] == 1, latest_status
    stream(image, .8)
    assert latest_status["state"] == "following", latest_status
    assert latest_status["route_index"] == 2, latest_status
    gateway_path="/code/catkin_ws/src/duckiebot-ros/packages/duckie_lane_follower/src/command_gateway.py"
    gateway_env=dict(os.environ,DUCK2_CONTROL_TOKEN="integration-only")
    gateway_process=subprocess.Popen([sys.executable,gateway_path],env=gateway_env,
                                     stdout=log,stderr=subprocess.STDOUT)
    sys.path.insert(0,"/project/laptop")
    from chat_core import RobotTransport
    transport=RobotTransport(token="integration-only")
    deadline=time.monotonic()+10
    while True:
        try:
            desktop_status=transport.status()
            break
        except RuntimeError:
            if time.monotonic()>deadline:
                raise
            time.sleep(.1)
    assert desktop_status["vehicle"]=="duck2"
    try:
        RobotTransport(token="incorrect").status()
        raise AssertionError("Wrong gateway token was accepted")
    except RuntimeError:
        pass
    stream(image,.4)
    assert max(received[-1])>0
    transport.heartbeat()
    ack=transport.send("slow_down")
    assert ack["accepted"],ack
    ack=transport.send("stop")
    assert ack["accepted"],ack
    print("PASS desktop HTTP transport: authenticated status, speed command and stop acknowledgment",
          flush=True)
    time.sleep(.1)
    assert received[-1] == (0.,0.)

    # Keep images arriving while the desktop heartbeat disappears.
    stream(image,.2)
    transport.heartbeat()
    assert transport.send("continue")["accepted"]
    stream(image,2.5)
    assert received[-1] == (0.,0.)
    assert latest_status["client_connection_lost"], latest_status
    transport.heartbeat()
    stream(image,.3)
    assert received[-1] == (0.,0.), "Heartbeat reconnect resumed without Continue"
    transport.heartbeat()
    assert transport.send("continue")["accepted"]
    stream(image,.3)
    assert max(received[-1])>0
    assert transport.send("stop")["accepted"]
    print("PASS actual ROS: laptop heartbeat loss stops despite fresh camera; explicit resume required",
          flush=True)

    # Test obstacle hold and explicit clear/release without the client lease.
    stream(image,.2)
    command("continue")
    stream(image,.3)
    obstacle=image.copy()
    cv2.rectangle(obstacle,(285,365),(360,435),(0,255,255),-1)
    stream(obstacle,.4)
    assert latest_status["obstacle_stop"],latest_status
    assert received[-1] == (0.,0.)
    stream(image,.8)
    assert received[-1] == (0.,0.), "Clearing obstacle resumed without Continue"
    command("continue")
    stream(image,.3)
    assert max(received[-1])>0
    command("stop")
    print("PASS actual ROS: obstacle candidate holds a stop until clear and explicitly resumed",
          flush=True)

    # The recorder has camera/status subscribers and never publishes wheel commands.
    capture_root=Path(tempfile.mkdtemp(prefix="duck2-capture-"))
    capture_path="/code/catkin_ws/src/duckiebot-ros/packages/duckie_lane_follower/src/camera_capture.py"
    capture_process=subprocess.Popen([sys.executable,capture_path,
        "_output_dir:="+str(capture_root),"_frame_limit:=5","_rate:=10","_duration:=8"],
        stdout=log,stderr=subprocess.STDOUT)
    deadline=time.monotonic()+10
    while capture_process.poll() is None and time.monotonic()<deadline:
        stream(image,.2)
    assert capture_process.wait(timeout=2)==0, "Recorder failed"
    manifests=list(capture_root.glob("*/frames.jsonl"))
    assert len(manifests)==1
    rows=[json.loads(row) for row in manifests[0].read_text().splitlines()]
    assert len(rows)==5
    for row in rows:
        recorded=cv2.imread(str(manifests[0].parent/row["file"]))
        assert recorded is not None and recorded.shape==(480,640,3)
        assert row["vehicle"]=="duck2"
        assert row["status"] is not None and row["status_age"]<1.5
    assert received[-1]==(0.,0.)
    print("PASS camera recorder: five readable PNGs with JSONL timestamps and node status",flush=True)
    stop()
    print("PASS actual ROS: route placement gate, speed command, turn override, "
          "stop dwell, visual reacquisition, route advance and remote stop", flush=True)

    gateway_process.terminate()
    gateway_process.wait(timeout=10)
    gateway_process=None
    child=subprocess.Popen(["bash","/launch/duckiebot-ros/lane-chat.sh"],env=gateway_env,
                           stdout=log,stderr=subprocess.STDOUT)
    deadline=time.monotonic()+15
    while camera.get_num_connections()==0:
        if child.poll() is not None:
            raise RuntimeError("Combined launcher exited during startup")
        if time.monotonic()>deadline:
            raise RuntimeError("Combined launcher did not connect to camera")
        time.sleep(.1)
    stream(image,.5)
    transport.poll_status()
    assert transport.send("set_route",route=["A","D"],position_confirmed=True)["accepted"]
    assert transport.send("continue")["accepted"]
    stream(image,.5)
    assert max(received[-1])>0
    received.clear()
    stop()
    assert received and received[-1]==(0.,0.)
    assert camera.get_num_connections()==0
    print("PASS combined launcher: node and gateway start together; Ctrl+C stops wheel output",
          flush=True)
finally:
    if capture_process is not None and capture_process.poll() is None:
        capture_process.terminate()
        capture_process.wait(timeout=10)
    if gateway_process is not None and gateway_process.poll() is None:
        gateway_process.terminate()
        gateway_process.wait(timeout=10)
    if child is not None and child.poll() is None:
        child.terminate()
        child.wait(timeout=10)
    rospy.signal_shutdown("integration complete")
    master.terminate()
    master.wait(timeout=10)
    log.close()
    print(open("/tmp/lane-integration.log").read()[-4000:])
