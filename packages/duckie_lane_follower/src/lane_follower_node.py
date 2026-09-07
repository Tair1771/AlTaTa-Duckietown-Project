#!/usr/bin/env python3

"""
Lane follower node adapted from the local Gym-Duckietown simulation script.

This version is for the real Duckiebot:
- subscribes to the Duckiebot camera topic
- runs the OpenCV lane detector
- computes left/right wheel speeds
- publishes WheelsCmdStamped commands

It deliberately does not import gym_duckietown, pyglet, Simulator, or manual_update().
"""

import os
import copy
import json
import math
import threading
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


# Junction ports read from the supplied map. Curved roads can join equal ports.
MAP_PORTS = {
    "A": {"N": "D", "E": "B", "S": "E"},
    "B": {"N": "D", "E": "C", "S": "E", "W": "A"},
    "C": {"N": "D", "S": "E", "W": "B"},
    "D": {"N": "A", "S": "C", "W": "B"},
    "E": {"N": "B", "E": "C", "W": "A"},
}
HEADINGS = ("N", "E", "S", "W")


def validate_route(route):
    if not isinstance(route, list) or not 2 <= len(route) <= 50:
        raise ValueError("Route must contain 2 to 50 junction names")
    if any(not isinstance(j, str) or j not in MAP_PORTS for j in route):
        raise ValueError("Unknown map junction")
    for a, b in zip(route, route[1:]):
        if b not in MAP_PORTS[a].values():
            raise ValueError("No road from %s to %s" % (a, b))
    if any(a == c for a, c in zip(route, route[2:])):
        raise ValueError("U-turns are not supported")
    return list(route)


def junction_turn(previous, junction, following):
    entry = next(p for p, j in MAP_PORTS[junction].items() if j == previous)
    exit_port = next(p for p, j in MAP_PORTS[junction].items() if j == following)
    incoming = (HEADINGS.index(entry) + 2) % 4
    delta = (HEADINGS.index(exit_port) - incoming) % 4
    return {0: "straight", 1: "right", 3: "left"}.get(delta, "uturn")


def route_via_turn(route, index, turn):
    """Change the next junction exit and reconnect to the existing next waypoint."""
    if index >= len(route) - 1:
        raise ValueError("The next junction is the route destination")
    previous, junction, target = route[index-1:index+2]
    choices = [j for j in MAP_PORTS[junction].values()
               if j != previous and junction_turn(previous, junction, j) == turn]
    if not choices:
        raise ValueError("No %s exit at junction %s" % (turn, junction))
    exit_junction = choices[0]
    # Breadth-first search includes the incoming road so it never inserts U-turns.
    queue = [[junction, exit_junction]]
    while queue:
        path = queue.pop(0)
        if path[-1] == target:
            candidate = route[:index] + path + route[index+2:]
            try:
                return validate_route(candidate)
            except ValueError:
                continue
        for neighbor in MAP_PORTS[path[-1]].values():
            if neighbor not in path and neighbor != path[-2]:
                queue.append(path + [neighbor])
    raise ValueError("Cannot reconnect this turn to the remaining route")


class LaneFollowerNode(DTROS):
    def __init__(self, node_name):
        super(LaneFollowerNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION,
        )

        self.vehicle_name = os.environ["VEHICLE_NAME"]
        self.camera_topic = f"/{self.vehicle_name}/camera_node/image/compressed"
        self.wheels_topic = f"/{self.vehicle_name}/wheels_driver_node/wheels_cmd"

        self.bridge = CvBridge()

        self.publisher = rospy.Publisher(
            self.wheels_topic,
            WheelsCmdStamped,
            queue_size=1,
        )

        # Driving parameters. Keep these conservative for first tests.
        self.drive_enabled = rospy.get_param("~drive_enabled", False)
        self.show_debug = rospy.get_param("~show_debug", False)
        self.flip_steering = rospy.get_param("~flip_steering", True)

        self.base_speed = self.number_param("~base_speed", 0.08)
        self.lost_speed = self.number_param("~lost_speed", 0.0)
        self.max_speed = self.number_param("~max_speed", 0.18)

        self.alpha = self.number_param("~alpha", 0.10)
        self.deadband = self.number_param("~deadband", 0.04)
        self.k_p = self.number_param("~k_p", 0.20)
        self.max_steering = self.number_param("~max_steering", 0.08)
        self.max_steering_change = self.number_param("~max_steering_change", 0.004)

        # Image-processing parameters copied from the current simulator version.
        self.roi_y0_fraction = self.number_param("~roi_y0_fraction", 0.50)
        self.roi_y1_fraction = self.number_param("~roi_y1_fraction", 0.95)
        self.yellow_right_cutoff = self.number_param("~yellow_right_cutoff", 0.60)
        self.white_left_cutoff = self.number_param("~white_left_cutoff", 0.35)
        self.fallback_lane_width_fraction = self.number_param("~fallback_lane_width_fraction", 0.27)

        self.filtered_error = 0.0
        self.prev_steering = 0.0
        self.frame_count = 0

        # A stop stays latched until the route/command layer explicitly releases it.
        self.red_stop_latched = False
        self.red_stop_y_fraction = self.number_param("~red_stop_y_fraction", 0.65)
        self.red_stop_min_width_fraction = self.number_param("~red_stop_min_width_fraction", 0.18)
        self.acceleration_limit = self.number_param("~acceleration_limit", 0.15)
        self._last_control_time = time.monotonic()
        self._last_publish_time = time.monotonic()
        self._last_wheel_speeds = (0.0, 0.0)

        self.route_enabled = rospy.get_param("~route_enabled", False)
        self.junctions_calibrated = rospy.get_param("~junctions_calibrated", False)
        self.route = validate_route(rospy.get_param("~route", ["A", "D", "C", "E", "A"]))
        self.route_index = 1
        self.auto_continue = rospy.get_param("~auto_continue", True)
        self.navigation_state = "awaiting_route" if self.route_enabled else "following"
        self.manual_stop = False
        self.speed_scale = 1.0
        self.stop_hold_seconds = self.number_param("~stop_hold_seconds", 2.0)
        self.junction_entry_seconds = self.number_param("~junction_entry_seconds", 0.5)
        self.junction_turn_seconds = self.number_param("~junction_turn_seconds", 1.6)
        self.junction_straight_seconds = self.number_param("~junction_straight_seconds", 1.0)
        self.junction_reacquire_timeout = self.number_param("~junction_reacquire_timeout", 3.0)
        self.junction_speed = self.number_param("~junction_speed", 0.05)
        self.junction_bias = self.number_param("~junction_bias", 0.025)
        self._stop_started = None
        self._crossing_started = None
        self._reacquire_started = None
        self._lane_good_since = None
        self._red_clear_since = None
        self._departed_red = False
        self._active_turn = None
        self._last_lane_error = None
        self._lane_both_visible = False
        self._last_command = None
        self._control_epoch = 0
        self.require_client_heartbeat = rospy.get_param("~require_client_heartbeat", False)
        if self.require_client_heartbeat:
            self.manual_stop = True
        self._client_timeout = 2.0
        self._heartbeat_clients = {}
        self._active_client_id = None
        self._client_connection_lost = False
        self.obstacle_enabled = rospy.get_param("~obstacle_enabled", True)
        self.obstacle_stop_latched = False
        self._obstacle_visible = False
        self._obstacle_clear_since = None
        self._obstacle_box = None
        self._lane_limits = None
        self._seen_commands = {}
        self._fault_reason = None
        # Reject settings that could invalidate timing or the wheel-command gate.
        for value in (self.base_speed, self.max_speed, self.acceleration_limit,
                      self.stop_hold_seconds, self.junction_entry_seconds,
                      self.junction_turn_seconds, self.junction_straight_seconds,
                      self.junction_reacquire_timeout, self.junction_speed, self.junction_bias):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Speed and junction settings must be finite and positive")
        if not 0 < self.alpha <= 1:
            raise ValueError("alpha must be in (0, 1]")
        self.validate_settings()
        self.yellow_lower, self.yellow_upper = self.color_range("yellow", [24, 140, 120], [36, 255, 255])
        self.white_lower, self.white_upper = self.color_range("white", [0, 0, 170], [180, 55, 255])
        self.red_low_lower, self.red_low_upper = self.color_range("red_low", [0, 110, 90], [10, 255, 255])
        self.red_high_lower, self.red_high_upper = self.color_range("red_high", [170, 110, 90], [180, 255, 255])
        self._lane_diagnostic = "No valid image yet"
        self._camera_error = "Waiting for camera"
        self._stop_reason = "Waiting for camera"
        self._last_camera_stamp = None
        self._camera_valid = False
        self._camera_lock = threading.Lock()
        self.status_publisher = rospy.Publisher(
            f"/{self.vehicle_name}/lane_follower/status", String, queue_size=1, latch=True
        )

        if self.show_debug:
            cv2.namedWindow("duckiebot_camera", cv2.WINDOW_NORMAL)
            cv2.namedWindow("duckiebot_lane_mask", cv2.WINDOW_NORMAL)

        # Initialize all callback state before subscribing to the live camera.
        self._wheel_lock = threading.RLock()
        self._stopping = False
        self._last_frame_time = time.monotonic()
        self._camera_timeout = 0.5
        self.subscriber = rospy.Subscriber(
            self.camera_topic,
            CompressedImage,
            self.callback,
            queue_size=1,
            buff_size=2 ** 24,
        )

        rospy.on_shutdown(self.on_shutdown)
        self.command_subscriber = rospy.Subscriber(
            f"/{self.vehicle_name}/lane_follower/command", String,
            self.command_callback, queue_size=10,
        )
        self._camera_watchdog = rospy.Timer(
            rospy.Duration(0.1), self.check_camera_timeout
        )

        rospy.loginfo(f"LaneFollowerNode started for {self.vehicle_name}")
        rospy.loginfo(f"Camera topic: {self.camera_topic}")
        rospy.loginfo(f"Wheels topic: {self.wheels_topic}")
        rospy.loginfo(f"drive_enabled={self.drive_enabled}, show_debug={self.show_debug}")

    @staticmethod
    def number_param(name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError("%s must be a finite number" % name)
        return float(value)

    def validate_settings(self):
        for name in ("drive_enabled", "show_debug", "flip_steering", "route_enabled",
                     "junctions_calibrated", "auto_continue", "require_client_heartbeat", "obstacle_enabled"):
            if type(getattr(self, name)) is not bool:
                raise ValueError("~%s must be a boolean" % name)
        if not 0 < self.base_speed <= self.max_speed <= 1 or self.lost_speed != 0:
            raise ValueError("Require 0 < base_speed <= max_speed <= 1 and lost_speed = 0")
        if not (0 <= self.deadband < 1 and 0 <= self.k_p <= 1
                and 0 <= self.max_steering <= self.max_speed
                and 0 < self.max_steering_change <= 1 and 0 < self.acceleration_limit <= 1):
            raise ValueError("Invalid steering or acceleration settings")
        if not 0 <= self.roi_y0_fraction < self.roi_y1_fraction <= 1:
            raise ValueError("Require 0 <= roi_y0_fraction < roi_y1_fraction <= 1")
        if not 0 <= self.red_stop_y_fraction < .95:
            raise ValueError("Require 0 <= red_stop_y_fraction < 0.95")
        for name in ("yellow_right_cutoff", "white_left_cutoff", "fallback_lane_width_fraction",
                     "red_stop_min_width_fraction"):
            if not 0 < getattr(self, name) < 1:
                raise ValueError("~%s must be between 0 and 1" % name)
        if not 0 <= self.junction_bias <= self.junction_speed <= self.max_speed:
            raise ValueError("Require junction_bias <= junction_speed <= max_speed")

    @staticmethod
    def color_range(name, lower, upper):
        bounds = [rospy.get_param("~%s_%s" % (name, suffix), default)
                  for suffix, default in (("lower", lower), ("upper", upper))]
        for bound in bounds:
            if (not isinstance(bound, (list, tuple)) or len(bound) != 3
                    or any(type(v) is not int or not 0 <= v <= limit
                           for v, limit in zip(bound, (180, 255, 255)))):
                raise ValueError("~%s bounds must be three HSV integers" % name)
        if any(lo > hi for lo, hi in zip(*bounds)):
            raise ValueError("~%s lower bound exceeds upper bound" % name)
        return tuple(np.array(bound, dtype=np.uint8) for bound in bounds)

    @staticmethod
    def validate_image(image):
        if (not isinstance(image, np.ndarray) or image.dtype != np.uint8
                or image.ndim != 3 or image.shape[2] != 3
                or image.shape[0] < 20 or image.shape[1] < 20):
            raise ValueError("Expected a nonempty BGR uint8 image at least 20 by 20 pixels")

    def reset_steering(self):
        self.filtered_error = self.prev_steering = 0.0
        self._last_control_time = time.monotonic()

    def detect_lane_bgr(self, img_bgr):
        """
        Detect the right-lane center from a real Duckiebot BGR camera image.

        Return:
            lane_error: normalized lane-center error, or None if no lane is found
            debug_image: BGR image with visual overlays
            debug_mask: BGR mask showing yellow and white detections
        """

        self.validate_image(img_bgr)
        h, w = img_bgr.shape[:2]
        self._lane_limits = None

        roi_y0 = int(h * self.roi_y0_fraction)
        roi_y1 = int(h * self.roi_y1_fraction)
        roi = img_bgr[roi_y0:roi_y1, :]

        if roi.size == 0:
            raise ValueError("Image region is empty")
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Yellow lane marking. These values will likely need tuning on the real track.
        lower_yellow = self.yellow_lower
        upper_yellow = self.yellow_upper
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # White lane border. These values will likely need tuning on the real track.
        lower_white = self.white_lower
        upper_white = self.white_upper
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        # In the right lane, the yellow line is expected left/middle and white on the right.
        yellow_mask[:, int(self.yellow_right_cutoff * w):] = 0
        white_mask[:, :int(self.white_left_cutoff * w)] = 0

        kernel = np.ones((5, 5), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)

        def weighted_center_x(mask, min_pixels):
            ys, xs = np.where(mask > 0)

            if len(xs) < min_pixels:
                return None, None

            # Lower pixels are closer to the bot, so they are weighted more strongly.
            weights = ((ys + 1) / mask.shape[0]) ** 3
            center_x = float(np.average(xs, weights=weights))
            center_y = float(np.average(ys + roi_y0, weights=weights))

            return center_x, center_y

        self._lane_both_visible = False
        yellow_x, yellow_y = weighted_center_x(yellow_mask, min_pixels=60)
        white_x, white_y = weighted_center_x(white_mask, min_pixels=100)

        debug_mask = np.zeros((yellow_mask.shape[0], yellow_mask.shape[1], 3), dtype=np.uint8)
        debug_mask[yellow_mask > 0] = (0, 255, 255)
        debug_mask[white_mask > 0] = (255, 255, 255)

        debug_image = img_bgr.copy()
        cv2.rectangle(debug_image, (0, roi_y0), (w-1, roi_y1-1), (255, 0, 0), 2)
        cv2.rectangle(debug_image, (int(.35*w), int(self.red_stop_y_fraction*h)),
                      (int(.95*w), int(.95*h)), (0, 0, 255), 2)
        for x, y, color in ((yellow_x, yellow_y, (0, 255, 255)),
                            (white_x, white_y, (255, 255, 255))):
            if x is not None:
                cv2.circle(debug_image, (int(x), int(y)), 5, color, -1)
        self._lane_diagnostic = "Both boundaries"


        if yellow_x is not None and white_x is not None:
            # A reversed or implausibly narrow pair is not an outgoing right lane.
            if white_x - yellow_x < 0.15 * w:
                self._lane_diagnostic = "Lane lost: reversed or narrow boundaries"
                return None, debug_image, debug_mask
            self._lane_both_visible = True
            lane_center_x = (yellow_x + white_x) / 2.0
        elif yellow_x is not None:
            self._lane_diagnostic = "Yellow-only fallback"
            lane_center_x = yellow_x + self.fallback_lane_width_fraction * w
        elif white_x is not None:
            self._lane_diagnostic = "White-only fallback"
            lane_center_x = white_x - self.fallback_lane_width_fraction * w
        else:
            self._lane_diagnostic = "Lane lost: no boundaries"
            return None, debug_image, debug_mask

        if not math.isfinite(lane_center_x) or not 0 <= lane_center_x < w:
            self._lane_both_visible = False
            self._lane_diagnostic = "Lane lost: centre outside image"
            return None, debug_image, debug_mask

        if yellow_x is not None and white_x is not None:
            self._lane_limits = (yellow_x, white_x)
        else:
            half_width = self.fallback_lane_width_fraction * w
            self._lane_limits = (lane_center_x - half_width, lane_center_x + half_width)

        target_center_x = 0.50 * w
        lane_error = (lane_center_x - target_center_x) / (w / 2)

        # Draw debug overlays.
        cv2.rectangle(debug_image, (0, roi_y0), (w, roi_y1), (255, 0, 0), 2)
        cv2.line(
            debug_image,
            (int(target_center_x), roi_y0),
            (int(target_center_x), roi_y1),
            (0, 0, 255),
            2,
        )
        cv2.circle(
            debug_image,
            (int(lane_center_x), int((roi_y0 + roi_y1) / 2)),
            7,
            (255, 0, 0),
            -1,
        )

        if yellow_x is not None:
            cv2.circle(debug_image, (int(yellow_x), int(yellow_y)), 5, (0, 255, 255), -1)

        if white_x is not None:
            cv2.circle(debug_image, (int(white_x), int(white_y)), 5, (255, 255, 255), -1)

        return lane_error, debug_image, debug_mask

    def detect_obstacle_bgr(self, img_bgr):
        """Provisional compact bright/colored-object detector for the lane corridor.

        It is not a general object recognizer: dark obstacles can be invisible,
        and road reflections can be false positives. Calibrate with real frames.
        """
        if not self.obstacle_enabled:
            return None
        self.validate_image(img_bgr)
        h, w = img_bgr.shape[:2]
        left, right = self._lane_limits or (0.30 * w, 0.85 * w)
        margin = (right - left) * 0.20
        x0, x1 = max(0, int(left + margin)), min(w, int(right - margin))
        y0, y1 = int(h * 0.55), int(h * 0.95)
        if x1 <= x0:
            return None
        hsv = cv2.cvtColor(img_bgr[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
        colored = cv2.inRange(hsv, np.array([0, 90, 70]), np.array([180, 255, 255]))
        bright = cv2.inRange(hsv, np.array([0, 0, 140]), np.array([180, 90, 255]))
        mask = cv2.morphologyEx(colored | bright, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, width, height = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            if (width >= 0.04*w and height >= 0.08*h
                    and 0.4 <= width / height <= 2.8
                    and area >= 0.004*h*w and area >= 0.25*width*height
                    and y0+y+height >= 0.68*h):
                return x0+x, y0+y, width, height
        return None

    def update_obstacle(self, box):
        self._obstacle_box = box
        self._obstacle_visible = box is not None
        if box is not None:
            self._obstacle_clear_since = None
            if not self.obstacle_stop_latched:
                self._control_epoch += 1
                rospy.loginfo("Obstacle candidate in lane; stop requires explicit Continue")
            self.obstacle_stop_latched = True
            self.manual_stop = True
            if self.navigation_state in ("crossing", "reacquiring"):
                self.navigation_fault("Obstacle interrupted the junction; position must be reset")
            self.publish_wheels(0.0, 0.0)
        elif self._obstacle_clear_since is None:
            self._obstacle_clear_since = time.monotonic()

    def detect_red_stop(self, img_bgr):
        """Find a nearby transverse red marking in the forward driving corridor.

        Fractions are initial calibration values, not a measured stopping distance.
        Reject thin vertical markings and small red blobs such as distant signs.
        """
        self.validate_image(img_bgr)
        h, w = img_bgr.shape[:2]
        y0, y1 = int(h * self.red_stop_y_fraction), int(h * 0.95)
        x0, x1 = int(w * 0.35), int(w * 0.95)
        roi = img_bgr[y0:y1, x0:x1]
        if roi.size == 0:
            raise ValueError("Image region is empty")
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.red_low_lower, self.red_low_upper)
        mask |= cv2.inRange(hsv, self.red_high_lower, self.red_high_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            _, _, width, height = cv2.boundingRect(contour)
            if (width >= w * self.red_stop_min_width_fraction
                    and width >= 3 * height
                    and cv2.contourArea(contour) >= h * w * 0.003):
                return True
        return False

    def compute_wheel_speeds(self, lane_error):
        now = time.monotonic()
        dt = min(max(now - self._last_control_time, 0.001), 0.1)
        self._last_control_time = now
        # Preserve the previous tuning at 30 Hz, independent of actual frame rate.
        alpha = 1.0 - (1.0 - self.alpha) ** (dt * 30.0)
        steering_change = self.max_steering_change * dt * 30.0
        if lane_error is None or not math.isfinite(lane_error):
            self.reset_steering()
            steering = 0.0
            base_speed = self.lost_speed
        else:
            base_speed = self.base_speed * self.speed_scale
            self.filtered_error = (
                (1.0 - alpha) * self.filtered_error + alpha * lane_error
            )

            if abs(self.filtered_error) < self.deadband:
                control_error = 0.0
            else:
                control_error = self.filtered_error

            raw_steering = self.k_p * control_error
            raw_steering = float(np.clip(raw_steering, -self.max_steering, self.max_steering))

            steering = float(np.clip(
                raw_steering,
                self.prev_steering - steering_change,
                self.prev_steering + steering_change,
            ))

            self.prev_steering = steering

        if self.flip_steering:
            steering = -steering

        left_speed = base_speed - steering
        right_speed = base_speed + steering

        left_speed = float(np.clip(left_speed, 0.0, self.max_speed))
        right_speed = float(np.clip(right_speed, 0.0, self.max_speed))

        if not self.drive_enabled:
            left_speed = 0.0
            right_speed = 0.0

        return left_speed, right_speed, steering

    def publish_wheels(self, left_speed, right_speed):
        with self._wheel_lock:
            # Final gate also prevents an in-flight callback restarting the wheels.
            if (self._stopping or not self.drive_enabled or self.red_stop_latched
                    or self.manual_stop or self.obstacle_stop_latched
                    or self.client_expired()
                    or (self.require_client_heartbeat and self._active_client_id is None)
                    or self.navigation_state in
                    ("awaiting_route", "fault", "route_complete", "red_stop")):
                left_speed = right_speed = 0.0
            if not self._camera_valid or time.monotonic() - self._last_frame_time > self._camera_timeout:
                left_speed = right_speed = 0.0
            if not all(math.isfinite(v) for v in (left_speed, right_speed)):
                left_speed = right_speed = 0.0
            left_speed = max(0.0, min(left_speed, self.max_speed))
            right_speed = max(0.0, min(right_speed, self.max_speed))
            if left_speed == 0 and right_speed == 0:
                self.reset_steering()
            self._stop_reason = self.stop_reason() if left_speed == right_speed == 0 else None
            now = time.monotonic()
            dt = min(max(now - self._last_publish_time, 0.0), 0.1)
            # Ramp increases; stop commands and decreases take effect immediately.
            step = self.acceleration_limit * dt
            left_speed = min(left_speed, self._last_wheel_speeds[0] + step)
            right_speed = min(right_speed, self._last_wheel_speeds[1] + step)
            self._last_publish_time = now
            self._last_wheel_speeds = (left_speed, right_speed)
            msg = WheelsCmdStamped()
            msg.header.stamp = rospy.Time.now()
            msg.vel_left = left_speed
            msg.vel_right = right_speed
            self.publisher.publish(msg)

    def stop_reason(self):
        if self._stopping:
            return "Shutting down"
        if not self.drive_enabled:
            return "Driving disabled"
        if not self._camera_valid or time.monotonic() - self._last_frame_time > self._camera_timeout:
            return self._camera_error or "Camera timeout"
        if self._fault_reason:
            return self._fault_reason
        if self.obstacle_stop_latched:
            return "Obstacle candidate"
        if self.red_stop_latched:
            return "Red stop line"
        if self.manual_stop:
            return "Manual stop"
        if self.client_expired() or (self.require_client_heartbeat and self._active_client_id is None):
            return "Waiting for connected client"
        if self.navigation_state in ("awaiting_route", "fault", "route_complete", "red_stop"):
            return self.navigation_state
        return self._lane_diagnostic if self._last_lane_error is None else "Zero wheel request"

    def status(self):
        return {
            "vehicle": self.vehicle_name, "state": self.navigation_state,
            "drive_enabled": self.drive_enabled, "manual_stop": self.manual_stop,
            "red_stop": self.red_stop_latched, "speed_scale": self.speed_scale,
            "wheel_speeds": list(self._last_wheel_speeds),
            "camera_age": max(0.0, time.monotonic() - self._last_frame_time),
            "route_enabled": self.route_enabled, "route": list(self.route),
            "next_junction": self.route[self.route_index] if self.route_enabled else None,
            "route_index": self.route_index, "active_turn": self._active_turn,
            "junctions_calibrated": self.junctions_calibrated,
            "fault": self._fault_reason, "last_command": self._last_command,
            "control_epoch": self._control_epoch,
            "active_client": self._active_client_id,
            "client_connection_lost": self._client_connection_lost,
            "require_client_heartbeat": self.require_client_heartbeat,
            "obstacle_stop": self.obstacle_stop_latched,
            "obstacle_visible": self._obstacle_visible,
            "obstacle_box": self._obstacle_box,
            "camera_valid": self._camera_valid,
            "camera_error": self._camera_error,
            "lane_diagnostic": self._lane_diagnostic,
            "stop_reason": self._stop_reason,
            "lane_error": self._last_lane_error,
        }

    def publish_status(self):
        self.status_publisher.publish(String(data=json.dumps(self.status(), allow_nan=False)))

    def navigation_fault(self, reason):
        self.navigation_state = "fault"
        self.manual_stop = True
        self._fault_reason = reason
        self.publish_wheels(0.0, 0.0)

    def client_expired(self):
        if self._active_client_id is None:
            return False
        _, last_seen = self._heartbeat_clients.get(self._active_client_id, (0, -float("inf")))
        return time.monotonic() - last_seen > self._client_timeout

    def check_client_connection(self):
        if self.client_expired() and not self._client_connection_lost:
            self._client_connection_lost = True
            self._control_epoch += 1
            self.manual_stop = True
            if self.navigation_state in ("crossing", "reacquiring"):
                self.navigation_fault("Laptop connection lost during junction; position must be reset")
            self.publish_wheels(0.0, 0.0)

    def check_camera_timeout(self, event):
        with self._wheel_lock:
            if self._stopping:
                return
            self.check_client_connection()
            if time.monotonic() - self._last_frame_time > self._camera_timeout:
                self._camera_valid = False
                self._camera_error = "Camera timeout"
                self._obstacle_clear_since = None
                if self.navigation_state in ("crossing", "reacquiring"):
                    self.navigation_fault("Camera lost during junction; position must be reset")
                self.publish_wheels(0.0, 0.0)
            self.publish_status()

    def command_callback(self, msg):
        """Validated JSON commands; the conversational client never sends wheel speeds."""
        command_id = None
        with self._wheel_lock:
            try:
                command = json.loads(msg.data)
                if not isinstance(command, dict):
                    raise ValueError("Expected a JSON object")
                command_id = command.get("id")
                if not isinstance(command_id, str) or not 1 <= len(command_id) <= 100:
                    raise ValueError("A command id is required")
                if command_id in self._seen_commands:
                    self._last_command = self._seen_commands[command_id]
                    self.publish_status()
                    return
                action = command.get("action")
                if self._stopping:
                    raise ValueError("Node is shutting down")
                if action != "stop":
                    issued = command.get("issued_at")
                    if (isinstance(issued, bool) or not isinstance(issued, (float, int))
                            or not math.isfinite(issued) or not -2 <= time.time() - issued <= 5):
                        raise ValueError("Command expired or laptop clock is not synchronized")
                client_id = command.get("client_id")
                if action == "heartbeat":
                    if not isinstance(client_id, str) or not 1 <= len(client_id) <= 100:
                        raise ValueError("Heartbeat needs a client id")
                    age = time.time() - issued
                    if not -0.5 <= age <= 1.5:
                        raise ValueError("Heartbeat is stale; synchronize laptop and robot clocks")
                    previous, _ = self._heartbeat_clients.get(client_id, (-float("inf"), 0))
                    if issued > previous:
                        self._heartbeat_clients[client_id] = (issued, time.monotonic() - max(0.0, age))
                    if len(self._heartbeat_clients) > 10:
                        del self._heartbeat_clients[next(iter(self._heartbeat_clients))]
                    # Heartbeats must not overwrite a user's command acknowledgment.
                    return
                if action != "stop":
                    if client_id is not None:
                        if not isinstance(client_id, str) or not 1 <= len(client_id) <= 100:
                            raise ValueError("Invalid client id")
                        if (time.monotonic() - self._heartbeat_clients.get(client_id, (0, -float("inf")))[1]
                                > 1.5):
                            raise ValueError("A fresh laptop heartbeat is required")
                    elif self.require_client_heartbeat:
                        raise ValueError("Connect the laptop before controlling this mode")
                    if (self._active_client_id is not None and client_id != self._active_client_id
                            and (not self.manual_stop or client_id is None)):
                        raise ValueError("Another laptop is controlling the bot; Stop is always available")
                if (action != "stop" and "expected_control_epoch" in command
                        and command["expected_control_epoch"] != self._control_epoch):
                    raise ValueError("A stop superseded this command; read fresh status")
                if action == "stop":
                    self._control_epoch += 1
                    self._active_client_id = None
                    self._client_connection_lost = False
                    if self.navigation_state in ("crossing", "reacquiring"):
                        self.navigation_fault("Stopped within junction; position must be reset")
                    self.manual_stop = True
                    self.publish_wheels(0.0, 0.0)
                elif action in ("slow_down", "speed_up", "speed_scale"):
                    if action == "speed_scale":
                        scale = command.get("value")
                    else:
                        scale = self.speed_scale * (0.8 if action == "slow_down" else 1.2)
                    if (isinstance(scale, bool) or not isinstance(scale, (int, float))
                            or not math.isfinite(scale) or not 0.25 <= scale <= 1.5):
                        raise ValueError("Speed scale must be between 0.25 and 1.5")
                    self.speed_scale = float(scale)
                elif action == "set_route":
                    if (not self.route_enabled or self.navigation_state in ("crossing", "reacquiring")
                            or any(self._last_wheel_speeds)):
                        raise ValueError("Stop in route mode before setting the starting position")
                    route = validate_route(command.get("route"))
                    if command.get("position_confirmed") is not True:
                        raise ValueError("Confirm placement after the first junction, toward the second")
                    self.route, self.route_index = route, 1
                    self.navigation_state = "following"
                    self.manual_stop = True
                    self.red_stop_latched = False
                    self._fault_reason = None
                    self._active_turn = None
                elif action == "turn":
                    if not self.route_enabled or self.navigation_state not in ("following", "red_stop"):
                        raise ValueError("Set a route before changing the next turn")
                    if ("expected_route_index" in command
                            and (command["expected_route_index"] != self.route_index
                                 or command.get("expected_next_junction") != self.route[self.route_index])):
                        raise ValueError("The next junction changed while interpreting the request")
                    turn = command.get("value")
                    if turn not in ("left", "right", "straight"):
                        raise ValueError("Turn must be left, right, or straight")
                    self.route = route_via_turn(self.route, self.route_index, turn)
                elif action == "continue":
                    if not self.drive_enabled:
                        raise ValueError("Driving is disabled by the launcher")
                    if self.navigation_state in ("awaiting_route", "fault", "route_complete"):
                        raise ValueError("Set the route and confirm the starting position first")
                    if not self._camera_valid or time.monotonic() - self._last_frame_time > self._camera_timeout:
                        raise ValueError("Waiting for a fresh camera frame")
                    if (self.obstacle_stop_latched and (
                            self._obstacle_visible or self._obstacle_clear_since is None
                            or time.monotonic() - self._obstacle_clear_since < 0.5)):
                        raise ValueError("Waiting for the obstacle corridor to remain clear")
                    if self.red_stop_latched:
                        self.begin_crossing()
                    self.obstacle_stop_latched = False
                    self.manual_stop = False
                else:
                    raise ValueError("Unknown action")
                if action != "stop" and client_id is not None:
                    self._active_client_id = client_id
                    self._client_connection_lost = False
                self._last_command = {"id": command_id, "accepted": True, "reason": "Applied"}
            except (ValueError, TypeError, KeyError, OverflowError) as error:
                self._last_command = {"id": command_id, "accepted": False, "reason": str(error)}
            if command_id is not None and isinstance(command_id, str):
                self._seen_commands[command_id] = self._last_command
                if len(self._seen_commands) > 100:
                    del self._seen_commands[next(iter(self._seen_commands))]
            self.publish_status()

    def begin_crossing(self):
        if not self.route_enabled or not self.junctions_calibrated:
            raise ValueError("Junction control requires route mode and calibrated turn settings")
        if self._stop_started is None or time.monotonic() - self._stop_started < self.stop_hold_seconds:
            raise ValueError("The mandatory red-line stop is not finished")
        if self.route_index >= len(self.route) - 1:
            raise ValueError("Route destination reached")
        self._active_turn = junction_turn(*self.route[self.route_index-1:self.route_index+2])
        self._crossing_started = time.monotonic()
        self._crossing_updated = self._crossing_started
        self._crossing_progress = 0.0
        self._red_clear_since = self._lane_good_since = None
        self._departed_red = False
        self.navigation_state = "crossing"
        self.red_stop_latched = False

    def navigation_wheels(self, lane_error, red_visible):
        now = time.monotonic()
        self.check_client_connection()
        state = self.navigation_state
        if (state == "red_stop" and self.route_enabled and self.junctions_calibrated
                and self.auto_continue and not self.manual_stop
                and self._stop_started is not None
                and now - self._stop_started >= self.stop_hold_seconds):
            self.begin_crossing()
            return 0.0, 0.0, 0.0
        if state in ("crossing", "reacquiring"):
            if red_visible and self._departed_red:
                self.navigation_fault("Unexpected red line during junction; position must be reset")
                return 0.0, 0.0, 0.0
            if not red_visible:
                if self._red_clear_since is None:
                    self._red_clear_since = now
                self._departed_red |= now - self._red_clear_since >= 0.2
            else:
                self._red_clear_since = None
            duration = (self.junction_straight_seconds if self._active_turn == "straight"
                        else self.junction_turn_seconds)
            dt = min(max(now - self._crossing_updated, 0.0), 0.1)
            self._crossing_updated = now
            self._crossing_progress += dt * min(self.speed_scale, 1.0)
            elapsed = self._crossing_progress
            if now - self._crossing_started > (
                    (self.junction_entry_seconds + duration) / 0.25
                    + self.junction_reacquire_timeout + 2.0):
                self.navigation_fault("Junction traversal exceeded its time limit")
                return 0.0, 0.0, 0.0
            if elapsed >= self.junction_entry_seconds + duration:
                if state == "crossing":
                    self.navigation_state = "reacquiring"
                    self._reacquire_started = now
                good_lane = (self._departed_red and self._lane_both_visible
                             and lane_error is not None and abs(lane_error) < 0.35)
                if good_lane:
                    if self._lane_good_since is None:
                        self._lane_good_since = now
                    if now - self._lane_good_since >= 0.3:
                        self.route_index += 1
                        self.navigation_state = "following"
                        self._active_turn = None
                        self.filtered_error = self.prev_steering = 0.0
                        return self.compute_wheel_speeds(lane_error)
                else:
                    self._lane_good_since = None
                if now - self._reacquire_started >= self.junction_reacquire_timeout:
                    self.navigation_fault("Outgoing lane was not reacquired; position must be reset")
                    return 0.0, 0.0, 0.0
                if lane_error is None:
                    return 0.0, 0.0, 0.0
                left, right, steering = self.compute_wheel_speeds(lane_error)
                ratio = min(1.0, self.junction_speed / max(left, right, 0.001))
                return left * ratio, right * ratio, steering
            bias = 0.0
            if elapsed >= self.junction_entry_seconds:
                bias = {"left": self.junction_bias, "right": -self.junction_bias,
                        "straight": 0.0}[self._active_turn]
            # Positive right-left difference turns left in differential drive.
            speed = self.junction_speed * min(self.speed_scale, 1.0)
            bias *= min(self.speed_scale, 1.0)
            return max(0.0, speed - bias), max(0.0, speed + bias), bias
        if red_visible and not self.red_stop_latched:
            self.red_stop_latched = True
            self._stop_started = now
            if self.route_enabled and self.route_index == len(self.route)-1:
                self.navigation_state = "route_complete"
            elif state == "following":
                self.navigation_state = "red_stop"
            rospy.loginfo("Red stop line detected; holding position")
        return self.compute_wheel_speeds(lane_error)

    def camera_stamp(self, msg, received_at):
        try:
            stamp = msg.header.stamp.to_sec()
        except (AttributeError, TypeError, ValueError):
            raise ValueError("Camera timestamp is missing")
        now = rospy.Time.now().to_sec()
        if not math.isfinite(stamp) or stamp <= 0:
            raise ValueError("Camera timestamp is invalid or zero")
        if now - stamp > self._camera_timeout or stamp - now > .1:
            raise ValueError("Camera timestamp is stale or ahead of ROS time")
        if self._last_camera_stamp is not None and stamp <= self._last_camera_stamp:
            raise ValueError("Camera timestamp is repeated or out of order")
        if time.monotonic() - received_at > self._camera_timeout:
            raise ValueError("Camera processing exceeded freshness timeout")
        return stamp

    def reject_camera(self, reason):
        with self._wheel_lock:
            self._camera_valid = False
            self._camera_error = str(reason)
            self._lane_limits = None
            self._lane_both_visible = False
            self._last_lane_error = None
            self._lane_diagnostic = "Image rejected"
            self._obstacle_clear_since = None
            if self.navigation_state in ("crossing", "reacquiring"):
                self.navigation_fault("Invalid camera image during junction: " + str(reason))
            self.publish_wheels(0.0, 0.0)

    def callback(self, msg):
        received_at = time.monotonic()
        # Serialize image callbacks without blocking Stop, shutdown or the watchdog.
        with self._camera_lock:
            if self._stopping:
                return
            try:
                stamp = self.camera_stamp(msg, received_at)
                img_bgr = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
                self.validate_image(img_bgr)
                # Detection mutates only this private snapshot until all checks pass.
                perception = copy.copy(self)
                red_visible = perception.detect_red_stop(img_bgr)
                lane_error, debug_image, debug_mask = perception.detect_lane_bgr(img_bgr)
                obstacle_box = perception.detect_obstacle_bgr(img_bgr)
                with self._wheel_lock:
                    if self._stopping:
                        return
                    self.camera_stamp(msg, received_at)
                    if received_at - self._last_frame_time > self._camera_timeout:
                        self._obstacle_clear_since = None
                    self._last_frame_time = time.monotonic() - max(
                        time.monotonic() - received_at, rospy.Time.now().to_sec() - stamp, 0.0)
                    self._last_camera_stamp = stamp
                    self._camera_valid = True
                    self._camera_error = None
                    self._last_lane_error = lane_error
                    self._lane_limits = perception._lane_limits
                    self._lane_both_visible = perception._lane_both_visible
                    self._lane_diagnostic = perception._lane_diagnostic
                    self.update_obstacle(obstacle_box)
                    left_speed, right_speed, steering = self.navigation_wheels(lane_error, red_visible)
                    self.publish_wheels(left_speed, right_speed)
            except (CvBridgeError, cv2.error, ValueError, TypeError, AttributeError) as error:
                rospy.logwarn("Could not process camera image: %s", error)
                self.reject_camera(error)
                return

        if self.show_debug and not self._stopping:
            cv2.putText(debug_image, self._lane_diagnostic, (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 0), 1)
            cv2.putText(debug_image, "L %.3f  R %.3f | %s" % (
                *self._last_wheel_speeds, self._stop_reason or "Following"), (10, 115),
                cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 0), 1)
            if self._obstacle_box is not None:
                x, y, width, height = self._obstacle_box
                cv2.rectangle(debug_image, (x, y), (x+width, y+height), (255, 0, 255), 2)
                cv2.putText(debug_image, "STOP: obstacle candidate", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            if self.red_stop_latched:
                cv2.putText(debug_image, "STOP: red line", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow("duckiebot_camera", debug_image)
            cv2.imshow("duckiebot_lane_mask", debug_mask)
            cv2.waitKey(1)

        if self.frame_count % 20 == 0:
            lane_error_str = "None" if lane_error is None else f"{lane_error:.3f}"
            rospy.loginfo(
                f"lane_error={lane_error_str}, "
                f"filtered_error={self.filtered_error:.3f}, "
                f"steering={steering:.3f}, "
                f"left={self._last_wheel_speeds[0]:.3f}, "
                f"right={self._last_wheel_speeds[1]:.3f}, "
                f"drive_enabled={self.drive_enabled}, red_stop={self.red_stop_latched}"
            )

        self.frame_count += 1

    def on_shutdown(self):
        with self._wheel_lock:
            if self._stopping:
                return
            self._stopping = True
            self.publish_wheels(0.0, 0.0)
        if self.show_debug:
            cv2.destroyAllWindows()
        rospy.loginfo("LaneFollowerNode stopped; wheels set to zero")


if __name__ == "__main__":
    lane_follower_node = LaneFollowerNode(node_name="lane_follower_node")
    rospy.spin()
