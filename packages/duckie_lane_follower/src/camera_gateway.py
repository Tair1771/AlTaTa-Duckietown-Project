#!/usr/bin/env python3
"""Read-only camera and lane-diagnostic HTTP service. It has no ROS publisher."""

import hmac
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

from lane_follower_node import LaneFollowerNode


def ppm_bytes(image_bgr):
    LaneFollowerNode.validate_image(image_bgr)
    rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    height, width = rgb.shape[:2]
    return ("P6\n%d %d\n255\n" % (width, height)).encode("ascii") + rgb.tobytes()


def color_param(name, lower, upper):
    bounds = [rospy.get_param("~%s_%s" % (name, suffix), default)
              for suffix, default in (("lower", lower), ("upper", upper))]
    for bound in bounds:
        if (not isinstance(bound, (list, tuple)) or len(bound) != 3
                or any(type(value) is not int or not 0 <= value <= limit
                       for value, limit in zip(bound, (180, 255, 255)))):
            raise ValueError("~%s bounds must be three HSV integers" % name)
    return tuple(np.array(bound, dtype=np.uint8) for bound in bounds)


class LanePreview:
    """Use the controller's exact detector method without constructing a controller."""

    validate_image = staticmethod(LaneFollowerNode.validate_image)
    junction_lane_geometry = LaneFollowerNode.junction_lane_geometry
    detect_lane_bgr = LaneFollowerNode.detect_lane_bgr

    def __init__(self):
        self.roi_y0_fraction = float(rospy.get_param("~roi_y0_fraction", 0.50))
        self.roi_y1_fraction = float(rospy.get_param("~roi_y1_fraction", 0.95))
        self.yellow_right_cutoff = float(rospy.get_param("~yellow_right_cutoff", 0.60))
        self.white_left_cutoff = float(rospy.get_param("~white_left_cutoff", 0.35))
        self.fallback_lane_width_fraction = float(
            rospy.get_param("~fallback_lane_width_fraction", 0.27))
        self.lane_target_fraction = float(rospy.get_param("~lane_target_fraction", 0.50))
        self.junction_straight_lane_target_fraction = float(rospy.get_param(
            "~junction_straight_lane_target_fraction", self.lane_target_fraction))
        self.red_stop_y_fraction = float(rospy.get_param("~red_stop_y_fraction", 0.65))
        self.red_stop_trigger_bottom_fraction = float(rospy.get_param(
            "~red_stop_trigger_bottom_fraction", self.red_stop_y_fraction))
        self.temporal_lane_width_fallback = bool(
            rospy.get_param("~temporal_lane_width_fallback", False))
        self.temporal_lane_width_timeout = float(
            rospy.get_param("~temporal_lane_width_timeout", 0.30))
        self.temporal_yellow_only_timeout = float(
            rospy.get_param("~temporal_yellow_only_timeout",
                            self.temporal_lane_width_timeout))
        self.boundary_risk_stop = bool(rospy.get_param("~boundary_risk_stop", False))
        self.white_boundary_risk_fraction = float(
            rospy.get_param("~white_boundary_risk_fraction", 0.43))
        self.yellow_lower, self.yellow_upper = color_param(
            "yellow", [24, 140, 120], [36, 255, 255])
        self.white_lower, self.white_upper = color_param(
            "white", [0, 0, 170], [180, 55, 255])
        self._duck_boxes = []
        self._lane_half_width_px = None
        self._lane_half_width_time = None
        self._lane_both_visible = False
        self._lane_limits = None
        self._junction_lane_geometry = None
        self._lane_diagnostic = "Waiting for camera"


class CameraFeed:
    def __init__(self, vehicle):
        self.bridge = CvBridge()
        self.detector = LanePreview()
        self.lock = threading.Lock()
        self.frames = {}
        self.received_at = None
        self.captured_at = None
        self.diagnostic = "Waiting for camera"
        self.subscriber = rospy.Subscriber(
            "/%s/camera_node/image/compressed" % vehicle,
            CompressedImage, self.receive, queue_size=1, buff_size=2 ** 24)

    def receive(self, message):
        try:
            image = self.bridge.compressed_imgmsg_to_cv2(message, desired_encoding="bgr8")
            lane_error, overlay, mask = self.detector.detect_lane_bgr(image)
            diagnostic = self.detector._lane_diagnostic
            if lane_error is not None:
                diagnostic += "; error %.3f" % lane_error
            stamp = float(message.header.stamp.to_sec())
            if not math.isfinite(stamp) or stamp <= 0:
                raise ValueError("Camera timestamp is missing")
            rendered = {"normal": ppm_bytes(image), "mask": ppm_bytes(mask),
                        "overlay": ppm_bytes(overlay)}
        except (CvBridgeError, ValueError, TypeError, cv2.error) as error:
            with self.lock:
                self.diagnostic = "Frame rejected: %s" % error
            return
        with self.lock:
            self.frames = rendered
            self.received_at = time.monotonic()
            self.captured_at = stamp
            self.diagnostic = diagnostic

    def get(self, view):
        if view not in ("normal", "mask", "overlay"):
            raise ValueError("Unknown camera view")
        with self.lock:
            if self.received_at is None or view not in self.frames:
                raise RuntimeError("No camera frame has been received")
            return (self.frames[view], max(0.0, time.monotonic() - self.received_at),
                    self.captured_at, self.diagnostic)


def handler_for(feed, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def authorized(self):
            if self.headers.get("Origin"):
                self.send_error(403, "Browser requests are not accepted")
                return False
            if token and not hmac.compare_digest(
                    self.headers.get("Authorization", ""), "Bearer " + token):
                self.send_error(401, "View token is missing or incorrect")
                return False
            return True

        def do_GET(self):
            if not self.authorized():
                return
            parsed = urlparse(self.path)
            if parsed.path == "/health":
                try:
                    _, age, captured, diagnostic = feed.get("normal")
                    body = json.dumps({"camera_age": age, "captured_at": captured,
                                       "diagnostic": diagnostic}).encode("utf8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(body)
                except RuntimeError as error:
                    self.send_error(503, str(error))
                return
            if parsed.path != "/camera":
                self.send_error(404, "Not found")
                return
            view = parse_qs(parsed.query).get("view", ["normal"])[0]
            try:
                data, age, captured, diagnostic = feed.get(view)
                self.send_response(200)
                self.send_header("Content-Type", "image/x-portable-pixmap")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Camera-Age", "%.3f" % age)
                self.send_header("X-Captured-At", "%.6f" % captured)
                clean = diagnostic.replace("\r", " ").replace("\n", " ")[:200]
                self.send_header("X-Diagnostic", clean)
                self.end_headers()
                self.wfile.write(data)
            except ValueError as error:
                self.send_error(400, str(error))
            except RuntimeError as error:
                self.send_error(503, str(error))
    return Handler


def main():
    # Duckietown wraps rospy.Subscriber and requires a DTROS node before a
    # subscriber is constructed.  This node has no publishers.
    node = DTROS(node_name="duck2_read_only_camera_gateway",
                 node_type=NodeType.PERCEPTION)
    host = rospy.get_param("~listen_host", "127.0.0.1")
    port = int(rospy.get_param("~port", 8766))
    token = os.environ.get("DUCK2_VIEW_TOKEN", "")
    if host not in ("127.0.0.1", "localhost") and not token:
        raise ValueError("Set DUCK2_VIEW_TOKEN when exposing the viewer beyond loopback")
    feed = CameraFeed(os.environ["VEHICLE_NAME"])
    server = ThreadingHTTPServer((host, port), handler_for(feed, token))
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    rospy.on_shutdown(server.shutdown)
    rospy.loginfo("Read-only camera gateway listening on %s:%s", host, port)
    rospy.spin()
    server.server_close()
    del node


if __name__ == "__main__":
    main()
