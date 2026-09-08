"""Actual ROS 2 DDS/process checks; MUST run in a --network none container.

Synthetic camera and diagnostic output only. Does not import a robot driver.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import urllib.request
import uuid
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from builtin_interfaces.msg import Time
from std_msgs.msg import String
from std_srvs.srv import SetBool

ROOT = Path(__file__).resolve().parents[1]
os.environ.update(VEHICLE_NAME="duck2", ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
children = []
logs = []
rclpy.init()
node = Node("diagnostic_integration")
state, safety, wheels = {}, {}, []
subs = [
    node.create_subscription(String, "/duck2/lane_follower/status", lambda m: state.update(json.loads(m.data)), 10),
    node.create_subscription(String, "/duck2/safety/status", lambda m: safety.update(json.loads(m.data)), 10),
    node.create_subscription(String, "/duck2/safety/diagnostic_wheels",
                             lambda m: wheels.append((time.monotonic(), json.loads(m.data))), 10)]
camera = node.create_publisher(CompressedImage, "/duck2/camera_node/image/compressed", qos_profile_sensor_data)
commands = node.create_publisher(String, "/duck2/lane_follower/command", 10)
arm_client = node.create_client(SetBool, "/duck2/safety/arm")
thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
thread.start()
image = np.zeros((480, 640, 3), np.uint8)
cv2.rectangle(image, (160,240), (180,455), (0,255,255), -1)
cv2.rectangle(image, (470,240), (490,455), (255,255,255), -1)
encoded = cv2.imencode(".jpg", image)[1].tobytes()

def start(script, *args):
    log = open("/tmp/ros2-"+script+".log", "w+")
    logs.append(log)
    process = subprocess.Popen([sys.executable, str(ROOT/"ros2"/(script+".py")), *args],
                               stdout=log, stderr=subprocess.STDOUT)
    children.append(process)
    return process

def frame(stamp=None):
    msg = CompressedImage()
    msg.header.stamp = node.get_clock().now().to_msg() if stamp is None else stamp
    msg.format, msg.data = "jpeg", encoded
    camera.publish(msg)

source_enabled = threading.Event()
source_enabled.set()
source_done = threading.Event()
def camera_loop():
    while not source_done.wait(.04):
        if source_enabled.is_set():
            frame()
producer = threading.Thread(target=camera_loop, daemon=True)
producer.start()

def wait(predicate, timeout=5, stream=True):
    source_enabled.set() if stream else source_enabled.clear()
    deadline = time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate():
            return
        time.sleep(.035)
    raise AssertionError("Timed out: "+str(dict(state=state, safety=safety, wheels=wheels[-3:])))

def stream(seconds=.4):
    source_enabled.set()
    time.sleep(seconds)

def arm(value=True):
    assert arm_client.wait_for_service(timeout_sec=5)
    request = SetBool.Request(data=value)
    future = arm_client.call_async(request)
    wait(future.done)
    return future.result().success

def command(action):
    identifier = str(uuid.uuid4())
    commands.publish(String(data=json.dumps(dict(id=identifier, action=action, issued_at=time.time()))))
    wait(lambda: (state.get("last_command") or {}).get("id") == identifier)
    assert state["last_command"]["accepted"], state["last_command"]

try:
    supervisor = start("supervisor")
    autonomy = start("autonomy", "--ros-args", "--params-file", str(ROOT/"ros2/diagnostic.yaml"),
                     "-p", "require_client_heartbeat:=false")
    gateway = start("gateway")
    wait(lambda: state.get("camera_valid") is True and len(wheels)>2)
    assert not arm(), "Unverified camera source must refuse arming"
    assert all(m["left"] == m["right"] == 0 for _,m in wheels)
    parameter_client = AsyncParameterClient(node, "/duck2_safety")
    assert parameter_client.wait_for_services(timeout_sec=5)
    verified = parameter_client.set_parameters([Parameter("source_timestamp_verified", value=True)])
    wait(verified.done)
    assert all(result.successful for result in verified.result().results)
    wait(lambda: safety.get("source_ready") is True)
    assert arm()
    wait(lambda: any(m["left"]>0 and m["right"]>0 for _,m in wheels[-5:]))
    assert all(0<=m["left"]<=.05 and 0<=m["right"]<=.05 for _,m in wheels)
    command("slow_down")
    assert abs(state["speed_scale"]-.8) < .001
    command("stop")
    wait(lambda: wheels[-1][1]["left"] == wheels[-1][1]["right"] == 0)
    command("continue")
    wait(lambda: safety.get("source_ready") is True)
    assert not safety["armed"]
    assert arm()
    wait(lambda: wheels[-1][1]["left"]>0)
    # Exercise unchanged HTTP command/ack path while the camera continues.
    source_enabled.set()
    try:
        with urllib.request.urlopen("http://127.0.0.1:8765/status", timeout=3) as response:
            assert json.load(response)["camera_valid"]
        req = urllib.request.Request("http://127.0.0.1:8765/command",
              data=json.dumps(dict(action="stop")).encode(), headers={"Content-Type":"application/json"})
        with urllib.request.urlopen(req, timeout=4) as response:
            assert json.load(response)["accepted"]
    finally:
        pass
    command("continue")
    wait(lambda: safety.get("source_ready") is True)
    assert arm()
    wait(lambda: wheels[-1][1]["left"]>0)
    # Frozen perception process must not block the independent supervisor.
    os.kill(autonomy.pid, signal.SIGSTOP)
    started = time.monotonic()
    wait(lambda: not safety.get("armed", True), timeout=1.5, stream=False)
    assert wheels[-1][1]["left"] == wheels[-1][1]["right"] == 0
    expired_in = time.monotonic()-started
    os.kill(autonomy.pid, signal.SIGCONT)
    stream(.5)
    wait(lambda: state.get("camera_valid") and state.get("camera_age", 1) < .1)
    assert not safety["armed"], "Reconnect must not automatically rearm"
    wait(lambda: safety.get("source_ready") is True)
    assert arm()
    wait(lambda: wheels[-1][1]["left"]>0)
    # Camera disappearance stops and latches at the supervisor.
    wait(lambda: not safety.get("armed", True), timeout=2, stream=False)
    assert wheels[-1][1]["left"] == wheels[-1][1]["right"] == 0
    # An invalid acquisition stamp must not be repaired by the adapter.
    wait(lambda: safety.get("source_ready") is True)
    assert arm()
    wait(lambda: wheels[-1][1]["left"]>0)
    source_enabled.clear()
    time.sleep(.06)
    frame(Time())
    wait(lambda: state.get("camera_valid") is False, stream=False)
    assert "invalid" in state["camera_error"].lower(), state
    wait(lambda: not safety.get("armed", True), stream=False)
    # Supervisor restart creates a new session and does not replay motion.
    previous_session = safety["session"]
    supervisor.terminate()
    supervisor.wait(timeout=5)
    supervisor = start("supervisor", "--ros-args", "-p", "source_timestamp_verified:=true")
    wait(lambda: safety.get("session") != previous_session and safety.get("source_ready") is True)
    assert not safety["armed"]
    assert wheels[-1][1]["left"] == wheels[-1][1]["right"] == 0
    assert arm()
    wait(lambda: wheels[-1][1]["left"]>0)
    assert arm(False)
    wait(lambda: wheels[-1][1]["left"] == wheels[-1][1]["right"] == 0)
    assert not any(name == "/duck2/wheels_driver_node/wheels_cmd"
                   for name,_ in node.get_topic_names_and_types())
    print(json.dumps(dict(result="PASS", diagnostic_only=True,
                         frozen_process_stop_observed_seconds=expired_in,
                         checks=["source_gate", "bounded_output", "stop_continue", "http_ack",
                                 "frozen_process", "explicit_rearm", "camera_loss", "invalid_source_stamp",
                                 "supervisor_restart", "independent_stop", "no_hardware_topic"])))
finally:
    source_done.set()
    producer.join(timeout=3)
    for process in reversed(children):
        if process.poll() is None:
            os.kill(process.pid, signal.SIGCONT)
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
    rclpy.shutdown()
    thread.join(timeout=3)
    node.destroy_node()
    for log in logs:
        log.seek(0)
        print(log.read(), file=sys.stderr)
        log.close()
