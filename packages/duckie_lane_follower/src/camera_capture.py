#!/usr/bin/env python3
"""Record camera frames and contemporaneous status; this node has no publisher."""
import datetime
import json
import os
from pathlib import Path
import threading
import time
import uuid

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

class CameraCapture:
    def __init__(self):
        self.vehicle = os.environ["VEHICLE_NAME"]
        self.rate = float(rospy.get_param("~rate", 5.0))
        self.limit = int(rospy.get_param("~frame_limit", 300))
        duration = float(rospy.get_param("~duration", 60.0))
        if not .1 <= self.rate <= 30 or not 1 <= self.limit <= 10000 or not 1 <= duration <= 3600:
            raise ValueError("Invalid capture rate, frame limit or duration")
        base = Path(rospy.get_param("~output_dir", "/data/captures")).expanduser().resolve()
        name = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.output = base / (name+"-"+uuid.uuid4().hex[:8])
        self.output.mkdir(parents=True)
        self.manifest = (self.output/"frames.jsonl").open("w")
        self.lock = threading.RLock()
        self.bridge = CvBridge()
        self.count = 0
        self.failed = False
        self.stopped = False
        self.last_saved = -float("inf")
        self.latest_status = None
        self.status_time = None
        self.timer = threading.Timer(duration, lambda: rospy.signal_shutdown("Capture duration reached"))
        self.timer.daemon = True
        rospy.on_shutdown(self.close)
        self.status_sub = rospy.Subscriber("/%s/lane_follower/status" % self.vehicle,
                                           String, self.status_callback, queue_size=1)
        self.camera_sub = rospy.Subscriber("/%s/camera_node/image/compressed" % self.vehicle,
                                           CompressedImage, self.camera_callback,
                                           queue_size=1, buff_size=2**24)
        self.timer.start()
        rospy.loginfo("Camera-only recording directory: %s", self.output)

    def status_callback(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                return
        except (ValueError, TypeError):
            return
        with self.lock:
            self.latest_status = value
            self.status_time = time.monotonic()

    def camera_callback(self, msg):
        with self.lock:
            now = time.monotonic()
            if self.stopped or now-self.last_saved < 1.0/self.rate:
                return
            try:
                frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
                if frame is None or frame.size == 0:
                    raise ValueError("Empty frame")
                name = "frame-%05d.png" % self.count
                if not cv2.imwrite(str(self.output/name), frame):
                    raise OSError("Could not save camera image")
                row = {
                    "file": name, "vehicle": self.vehicle,
                    "received_at": time.time(), "camera_stamp": msg.header.stamp.to_sec(),
                    "source": "ROS CompressedImage; physical origin not independently verified",
                    "status": self.latest_status,
                    "status_age": None if self.status_time is None else now-self.status_time,
                }
                self.manifest.write(json.dumps(row, allow_nan=False)+"\n")
                self.manifest.flush()
                self.count += 1
                self.last_saved = now
            except (CvBridgeError, cv2.error, ValueError, OSError) as error:
                self.failed = True
                rospy.logerr("Capture failed: %s", error)
                rospy.signal_shutdown("Capture failed")
                return
            if self.count >= self.limit:
                rospy.signal_shutdown("Frame limit reached")

    def close(self):
        with self.lock:
            if self.stopped:
                return
            self.stopped = True
            self.timer.cancel()
            self.manifest.close()
            rospy.loginfo("Saved %s camera frames to %s", self.count, self.output)

def main():
    rospy.init_node("duck2_camera_capture")
    capture = CameraCapture()
    rospy.spin()
    if capture.failed or capture.count == 0:
        raise SystemExit(2)

if __name__ == "__main__":
    main()
