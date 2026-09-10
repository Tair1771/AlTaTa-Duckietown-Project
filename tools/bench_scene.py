"""Synthetic camera scene server; run ONLY in the verified network-none container."""
import json
import os
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


def main():
    children = []
    source = "/project/packages/duckie_lane_follower/src/"
    log = open("/tmp/bench-services.log", "w")
    def spawn(args):
        children.append(subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT))
    state = {"scene": "straight", "status": {}, "crossing_at": None}
    spawn(["roscore"])
    time.sleep(3)
    rospy.init_node("synthetic_bench_scene", disable_signals=True)
    publisher = rospy.Publisher("/duck2/camera_node/image/compressed", CompressedImage, queue_size=1)
    def receive(message):
        value = json.loads(message.data)
        if value.get("state") in ("crossing", "reacquiring"):
            if state["crossing_at"] is None:
                state["crossing_at"] = time.monotonic()
        elif state["crossing_at"] is not None:
            state["scene"] = "straight"
            state["crossing_at"] = None
        state["status"] = value
    rospy.Subscriber("/duck2/lane_follower/status", String, receive, queue_size=1)
    for program, flags in (
        ("lane_follower_node.py", ["_drive_enabled:=true", "_route_enabled:=true",
         "_junctions_calibrated:=true", "_require_client_heartbeat:=true",
         "_base_speed:=0.09", "_max_speed:=0.20", "_obstacle_enabled:=false"]),
        ("command_gateway.py", []), ("camera_gateway.py", [])):
        spawn(["python3", source + program] + flags)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass
        def do_GET(self):
            body = json.dumps({"bench_simulation": True, "scene": state["scene"]}).encode()
            self.send_response(200)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        def do_POST(self):
            try:
                length = int(self.headers.get("Content-Length", 0))
                if not 0 < length < 512:
                    raise ValueError("size")
                scene = json.loads(self.rfile.read(length))["scene"]
                if scene not in ("straight", "curve", "red", "blank", "camera_loss"):
                    raise ValueError("scene")
                state["scene"] = scene
                self.do_GET()
            except (ValueError, KeyError):
                self.send_error(400)
    server = ThreadingHTTPServer(("127.0.0.1", 8768), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    stopping = threading.Event()
    signal.signal(signal.SIGTERM, lambda *args: stopping.set())
    signal.signal(signal.SIGINT, lambda *args: stopping.set())
    try:
        while not stopping.is_set():
            if any(child.poll() is not None for child in children):
                raise RuntimeError("Bench service exited; see /tmp/bench-services.log")
            scene = state["scene"]
            if scene != "camera_loss":
                crossing = state["crossing_at"]
                if crossing is not None:
                    scene = "blank" if time.monotonic()-crossing < 2 else "straight"
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                if scene != "blank":
                    for y in range(240, 476):
                        shift = int(100 * ((480-y)/240.)**2) if scene == "curve" else 0
                        cv2.line(frame, (160+shift,y), (180+shift,y), (0,255,255), 1)
                        cv2.line(frame, (470+shift,y), (490+shift,y), (255,255,255), 1)
                    if scene == "red":
                        cv2.rectangle(frame, (180,390), (470,435), (0,0,255), -1)
                cv2.putText(frame, "SIMULATION - NO HARDWARE", (20,40),
                            cv2.FONT_HERSHEY_SIMPLEX, .7, (255,255,255), 2)
                ok, data = cv2.imencode(".jpg", frame)
                message = CompressedImage()
                message.header.stamp = rospy.Time.now()
                message.format, message.data = "jpeg", data.tobytes()
                publisher.publish(message)
            stopping.wait(.1)
    finally:
        server.shutdown()
        for child in reversed(children):
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(5)
                except subprocess.TimeoutExpired:
                    child.kill()
        log.close()


if __name__ == "__main__":
    main()
