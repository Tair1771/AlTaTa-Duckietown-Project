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
from duckie_lane_follower.route_map import (
    MAP_ID, MAP_PORTS, junction_turn, route_via_turn, validate_route)


class LaneFollowerNode(DTROS):
    def __init__(self, node_name):
        super(LaneFollowerNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION,
        )

        self.vehicle_name = os.environ["VEHICLE_NAME"]
        self.camera_topic = self.topic_param(
            "~camera_topic", f"/{self.vehicle_name}/camera_node/image/compressed"
        )
        self.wheels_topic = self.topic_param(
            "~wheels_topic", f"/{self.vehicle_name}/wheels_driver_node/wheels_cmd"
        )

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
        # Opt in only for the bounded camera-guided tests until track verified.
        self.smooth_steering_deadband = rospy.get_param("~smooth_steering_deadband", False)
        self.k_p = self.number_param("~k_p", 0.20)
        # Optional gain scheduling keeps small straight-line corrections gentle
        # while retaining full authority for an established curve. Defaults
        # preserve the original single-gain controller.
        self.near_center_k_p = self.number_param("~near_center_k_p", self.k_p)
        self.full_gain_error = self.number_param("~full_gain_error", self.deadband)
        self.max_steering = self.number_param("~max_steering", 0.08)
        self.min_active_wheel_speed = self.number_param(
            "~min_active_wheel_speed", 0.0)
        self.taper_inner_wheel_floor = rospy.get_param(
            "~taper_inner_wheel_floor", False)
        # Opt-in sharp-right corner mode for the bounded physical-test preset.
        # Ordinary launchers leave it disabled until the state machine has
        # completed supervised track validation.
        self.sharp_corner_enabled = rospy.get_param(
            "~sharp_corner_enabled", False)
        self.sharp_corner_confirm_seconds = self.number_param(
            "~sharp_corner_confirm_seconds", 0.20)
        self.sharp_corner_recent_lane_seconds = self.number_param(
            "~sharp_corner_recent_lane_seconds", 1.0)
        self.sharp_corner_approach_seconds = self.number_param(
            "~sharp_corner_approach_seconds", 0.15)
        self.sharp_corner_turn_speed = self.number_param(
            "~sharp_corner_turn_speed", 0.15)
        self.sharp_corner_min_turn_seconds = self.number_param(
            "~sharp_corner_min_turn_seconds", 1.0)
        self.sharp_corner_white_confirm_seconds = self.number_param(
            "~sharp_corner_white_confirm_seconds", 0.10)
        self.sharp_corner_pivot_seconds = self.number_param(
            "~sharp_corner_pivot_seconds", 0.45)
        self.sharp_corner_relief_seconds = self.number_param(
            "~sharp_corner_relief_seconds", 0.25)
        self.sharp_corner_relief_inner_speed = self.number_param(
            "~sharp_corner_relief_inner_speed", 0.0)
        self.sharp_corner_max_turn_seconds = self.number_param(
            "~sharp_corner_max_turn_seconds", 3.5)
        self.sharp_corner_reacquire_seconds = self.number_param(
            "~sharp_corner_reacquire_seconds", 0.30)
        self.sharp_corner_trigger_error = self.number_param(
            "~sharp_corner_trigger_error", 0.10)
        self.sharp_corner_exit_error = self.number_param(
            "~sharp_corner_exit_error", 0.13)
        self.steering_bias = self.number_param("~steering_bias", 0.0)
        self.max_steering_change = self.number_param("~max_steering_change", 0.004)

        # Image-processing parameters copied from the current simulator version.
        self.roi_y0_fraction = self.number_param("~roi_y0_fraction", 0.50)
        self.roi_y1_fraction = self.number_param("~roi_y1_fraction", 0.95)
        self.yellow_right_cutoff = self.number_param("~yellow_right_cutoff", 0.60)
        self.white_left_cutoff = self.number_param("~white_left_cutoff", 0.35)
        self.fallback_lane_width_fraction = self.number_param("~fallback_lane_width_fraction", 0.27)
        # Test-only opt in: bridge short dashed-line gaps with the most recent
        # lane width measured while both boundaries were visible.
        self.temporal_lane_width_fallback = rospy.get_param(
            "~temporal_lane_width_fallback", False)
        self.temporal_lane_width_timeout = self.number_param(
            "~temporal_lane_width_timeout", 0.30)
        # A sharp right bend can temporarily move only the right white border
        # outside the image while the yellow centre boundary remains usable.
        # The default preserves the original symmetric behavior; bounded test
        # launchers may opt into a longer yellow-only interval.
        self.temporal_yellow_only_timeout = self.number_param(
            "~temporal_yellow_only_timeout", self.temporal_lane_width_timeout)
        # Test-only guard: a lone white boundary near the detector's left
        # cutoff means the robot is approaching or crossing that border.
        self.boundary_risk_stop = rospy.get_param("~boundary_risk_stop", False)
        self.white_boundary_risk_fraction = self.number_param(
            "~white_boundary_risk_fraction", 0.43)
        self._lane_half_width_px = None
        self._lane_half_width_time = None
        self.lane_target_fraction = self.number_param("~lane_target_fraction", 0.50)

        self.filtered_error = 0.0
        self.prev_steering = 0.0
        self.frame_count = 0

        # A stop stays latched until the route/command layer explicitly releases it.
        self.red_stop_latched = False
        self.red_stop_y_fraction = self.number_param("~red_stop_y_fraction", 0.65)
        # Keep the existing search region, but expose a separate proximity
        # threshold. A calibrated test can wait until the line is lower in
        # the image without treating disappearance as permission to proceed.
        self.red_stop_trigger_bottom_fraction = self.number_param(
            "~red_stop_trigger_bottom_fraction", self.red_stop_y_fraction)
        self.red_stop_min_width_fraction = self.number_param("~red_stop_min_width_fraction", 0.18)
        self._red_line_visible = False
        self._red_line_detection = None
        self.acceleration_limit = self.number_param("~acceleration_limit", 0.15)
        self._last_control_time = time.monotonic()
        self._last_publish_time = time.monotonic()
        self._last_wheel_speeds = (0.0, 0.0)

        self.route_enabled = rospy.get_param("~route_enabled", False)
        self.junctions_calibrated = rospy.get_param("~junctions_calibrated", False)
        self.route = validate_route(rospy.get_param("~route", ["A", "D", "C", "E", "A"]))
        self.route_index = 1
        self.route_map_id = MAP_ID
        self.start_approach = "%s->%s" % tuple(self.route[:2])
        self.destination_approach = "%s->%s" % tuple(self.route[-2:])
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
        # Direction-specific settings default to the original shared values.
        self.junction_left_seconds = self.number_param(
            "~junction_left_seconds", self.junction_turn_seconds)
        self.junction_right_seconds = self.number_param(
            "~junction_right_seconds", self.junction_turn_seconds)
        self.junction_straight_speed = self.number_param(
            "~junction_straight_speed", self.junction_speed)
        self.junction_left_speed = self.number_param(
            "~junction_left_speed", self.junction_speed)
        self.junction_right_speed = self.number_param(
            "~junction_right_speed", self.junction_speed)
        self.junction_left_bias = self.number_param(
            "~junction_left_bias", self.junction_bias)
        self.junction_right_bias = self.number_param(
            "~junction_right_bias", self.junction_bias)
        self._stop_started = None
        self._crossing_started = None
        self._reacquire_started = None
        self._lane_good_since = None
        self._red_clear_since = None
        self._departed_red = False
        self._active_turn = None
        self._junction_phase = "idle"
        self._junction_deadline_at = None
        self._last_junction_result = None
        self._last_lane_error = None
        self._lane_both_visible = False
        self._yellow_boundary_visible = False
        self._white_boundary_visible = False
        self._sharp_corner_state = "idle"
        self._sharp_corner_phase = "idle"
        self._sharp_corner_candidate_since = None
        self._sharp_corner_state_since = None
        self._sharp_corner_reacquire_since = None
        self._sharp_corner_white_since = None
        self._sharp_corner_last_both_time = None
        self._sharp_corner_cooldown_until = None
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
        # Experimental passing is opt-in after physical calibration on this bot.
        self.duck_lower, self.duck_upper = self.color_range("duck", [15, 90, 70], [40, 255, 255])
        self.avoidance_enabled = rospy.get_param("~avoidance_enabled", False)
        self.avoidance_calibrated = rospy.get_param("~avoidance_calibrated", False)
        self.avoidance_speed = self.number_param("~avoidance_speed", 0.04)
        self.avoidance_timeout = self.number_param("~avoidance_timeout", 12.0)
        self.avoidance_clear_seconds = self.number_param("~avoidance_clear_seconds", 1.0)
        if type(self.avoidance_enabled) is not bool or type(self.avoidance_calibrated) is not bool:
            raise ValueError("Avoidance flags must be booleans")
        if self.avoidance_enabled and (not self.avoidance_calibrated or not self.obstacle_enabled):
            raise ValueError("Avoidance requires obstacle detection and physical calibration")
        if not (0 < self.avoidance_speed <= min(self.base_speed, self.max_speed)
                and 0.5 <= self.avoidance_clear_seconds <= 5
                and self.avoidance_clear_seconds + 1 < self.avoidance_timeout <= 30):
            raise ValueError("Invalid avoidance speed, clearance duration or timeout")
        self.avoidance_state = "idle"
        self._avoidance_started = None
        self._avoidance_stable_since = None
        self._avoidance_clear_since = None
        self._duck_passed_edge = False
        self._avoidance_clear_progress = 0.0
        self._avoidance_updated = None
        self._duck_boxes = []
        self._road_geometry = None
        self._avoidance_reason = ("Waiting for a suitable passing scene" if self.avoidance_enabled
                                  else "Passing disabled until calibrated")
        self._seen_commands = {}
        self._fault_reason = None
        # Reject settings that could invalidate timing or the wheel-command gate.
        for value in (self.base_speed, self.max_speed, self.acceleration_limit,
                      self.stop_hold_seconds, self.junction_entry_seconds,
                      self.junction_turn_seconds, self.junction_straight_seconds,
                      self.junction_left_seconds, self.junction_right_seconds,
                      self.junction_reacquire_timeout, self.junction_speed,
                      self.junction_straight_speed, self.junction_left_speed,
                      self.junction_right_speed):
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

    @staticmethod
    def topic_param(name, default):
        value = rospy.get_param(name, default)
        if type(value) is not str or not value.startswith("/") or any(c.isspace() for c in value):
            raise ValueError("%s must be an absolute ROS topic name" % name)
        return value

    def validate_settings(self):
        for name in ("drive_enabled", "show_debug", "flip_steering", "route_enabled",
                     "junctions_calibrated", "auto_continue", "require_client_heartbeat", "obstacle_enabled",
                     "smooth_steering_deadband", "temporal_lane_width_fallback",
                     "boundary_risk_stop", "taper_inner_wheel_floor",
                     "sharp_corner_enabled"):
            if type(getattr(self, name)) is not bool:
                raise ValueError("~%s must be a boolean" % name)
        if not 0 < self.base_speed <= self.max_speed <= 1 or self.lost_speed != 0:
            raise ValueError("Require 0 < base_speed <= max_speed <= 1 and lost_speed = 0")
        if not 0 <= self.min_active_wheel_speed <= self.base_speed:
            raise ValueError(
                "~min_active_wheel_speed must be between zero and base_speed")
        if not (0 <= self.deadband < 1 and 0 <= self.near_center_k_p <= self.k_p <= 1
                and self.deadband <= self.full_gain_error < 1
                and 0 <= self.max_steering <= self.max_speed
                and abs(self.steering_bias) <= self.max_steering
                and 0 < self.max_steering_change <= 1 and 0 < self.acceleration_limit <= 1):
            raise ValueError("Invalid steering or acceleration settings")
        if not 0 <= self.roi_y0_fraction < self.roi_y1_fraction <= 1:
            raise ValueError("Require 0 <= roi_y0_fraction < roi_y1_fraction <= 1")
        if not 0 <= self.red_stop_y_fraction < .95:
            raise ValueError("Require 0 <= red_stop_y_fraction < 0.95")
        if not (self.red_stop_y_fraction
                <= self.red_stop_trigger_bottom_fraction < .95):
            raise ValueError(
                "Require red_stop_y_fraction <= "
                "red_stop_trigger_bottom_fraction < 0.95")
        for name in ("yellow_right_cutoff", "white_left_cutoff", "fallback_lane_width_fraction",
                     "lane_target_fraction",
                     "red_stop_min_width_fraction"):
            if not 0 < getattr(self, name) < 1:
                raise ValueError("~%s must be between 0 and 1" % name)
        if not 0 < self.temporal_lane_width_timeout <= .5:
            raise ValueError("~temporal_lane_width_timeout must be in (0, 0.5]")
        if not 0 < self.temporal_yellow_only_timeout <= 5.0:
            raise ValueError("~temporal_yellow_only_timeout must be in (0, 5.0]")
        if not self.white_left_cutoff < self.white_boundary_risk_fraction < 1:
            raise ValueError(
                "~white_boundary_risk_fraction must exceed white_left_cutoff and be below 1")
        if not (0.10 <= self.sharp_corner_confirm_seconds <= 0.50
                and 0 <= self.sharp_corner_approach_seconds <= 1.0
                and 0.50 <= self.sharp_corner_recent_lane_seconds <= 1.50
                and 0 < self.sharp_corner_turn_speed <= self.max_speed
                and 0 <= self.sharp_corner_min_turn_seconds <= 3.0
                and 0.05 <= self.sharp_corner_white_confirm_seconds <= 0.30
                and 0.20 <= self.sharp_corner_pivot_seconds <= 0.75
                and 0.10 <= self.sharp_corner_relief_seconds <= 0.50
                and 0 <= self.sharp_corner_relief_inner_speed <= self.base_speed
                and 0.5 <= self.sharp_corner_max_turn_seconds <= 10.0
                and 0.20 <= self.sharp_corner_reacquire_seconds <= 1.0
                and 0 < self.sharp_corner_trigger_error < 1
                and 0 < self.sharp_corner_exit_error < 0.35):
            raise ValueError("Invalid sharp-corner settings")
        if self.sharp_corner_enabled and (self.route_enabled or self.avoidance_enabled):
            raise ValueError("Sharp-corner test mode cannot be combined with routes or passing")
        if not 0 <= self.junction_bias <= self.junction_speed <= self.max_speed:
            raise ValueError("Require junction_bias <= junction_speed <= max_speed")
        for turn, speed, bias in (
                ("left", self.junction_left_speed, self.junction_left_bias),
                ("right", self.junction_right_speed, self.junction_right_bias)):
            if not (math.isfinite(bias) and 0 <= bias <= speed
                    and speed + bias <= self.max_speed):
                raise ValueError(
                    "Require junction_%s_bias <= junction_%s_speed and "
                    "their sum <= max_speed" % (turn, turn))
        if not self.junction_straight_speed <= self.max_speed:
            raise ValueError("Require junction_straight_speed <= max_speed")

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

        for x, y, bw, bh in self._duck_boxes:
            a, b = max(0, y - roi_y0), min(roi_y1 - roi_y0, y + bh - roi_y0)
            if b > a:
                yellow_mask[a:b, x:x+bw] = 0

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
        self._yellow_boundary_visible = yellow_x is not None
        self._white_boundary_visible = white_x is not None

        debug_mask = np.zeros((yellow_mask.shape[0], yellow_mask.shape[1], 3), dtype=np.uint8)
        debug_mask[yellow_mask > 0] = (0, 255, 255)
        debug_mask[white_mask > 0] = (255, 255, 255)

        debug_image = img_bgr.copy()
        cv2.rectangle(debug_image, (0, roi_y0), (w-1, roi_y1-1), (255, 0, 0), 2)
        cv2.rectangle(debug_image, (int(.35*w), int(self.red_stop_y_fraction*h)),
                      (int(.95*w), int(.95*h)), (0, 0, 255), 2)
        cv2.line(debug_image,
                 (int(.35*w), int(self.red_stop_trigger_bottom_fraction*h)),
                 (int(.95*w), int(self.red_stop_trigger_bottom_fraction*h)),
                 (0, 80, 255), 1)
        for x, y, color in ((yellow_x, yellow_y, (0, 255, 255)),
                            (white_x, white_y, (255, 255, 255))):
            if x is not None:
                cv2.circle(debug_image, (int(x), int(y)), 5, color, -1)
        self._lane_diagnostic = "Both boundaries"


        if yellow_x is not None and white_x is not None:
            # A reversed or implausibly narrow pair is not an outgoing right lane.
            if white_x - yellow_x < 0.15 * w:
                self._lane_half_width_px = None
                self._lane_half_width_time = None
                self._lane_diagnostic = "Lane lost: reversed or narrow boundaries"
                return None, debug_image, debug_mask
            self._lane_both_visible = True
            lane_center_x = (yellow_x + white_x) / 2.0
            half_width = (white_x - yellow_x) / 2.0
            if self.temporal_lane_width_fallback:
                self._lane_half_width_px = half_width
                self._lane_half_width_time = time.monotonic()
        elif yellow_x is not None:
            use_temporal = (self.temporal_lane_width_fallback
                            and self._lane_half_width_px is not None
                            and self._lane_half_width_time is not None
                            and time.monotonic() - self._lane_half_width_time
                            <= self.temporal_yellow_only_timeout)
            if self.temporal_lane_width_fallback and not use_temporal:
                self._lane_half_width_px = None
                self._lane_half_width_time = None
                self._lane_diagnostic = "Lane lost: boundary gap exceeded"
                return None, debug_image, debug_mask
            half_width = (self._lane_half_width_px if use_temporal
                          else self.fallback_lane_width_fraction * w)
            self._lane_diagnostic = ("Yellow-only temporal fallback" if use_temporal
                                     else "Yellow-only fallback")
            lane_center_x = yellow_x + half_width
        elif white_x is not None:
            use_temporal = (self.temporal_lane_width_fallback
                            and self._lane_half_width_px is not None
                            and self._lane_half_width_time is not None
                            and time.monotonic() - self._lane_half_width_time
                            <= self.temporal_lane_width_timeout)
            if self.temporal_lane_width_fallback and not use_temporal:
                self._lane_half_width_px = None
                self._lane_half_width_time = None
                self._lane_diagnostic = "Lane lost: boundary gap exceeded"
                return None, debug_image, debug_mask
            if (self.boundary_risk_stop
                    and white_x <= self.white_boundary_risk_fraction * w):
                self._lane_half_width_px = None
                self._lane_half_width_time = None
                self._lane_diagnostic = "Lane lost: white boundary risk"
                return None, debug_image, debug_mask
            half_width = (self._lane_half_width_px if use_temporal
                          else self.fallback_lane_width_fraction * w)
            self._lane_diagnostic = ("White-only temporal fallback" if use_temporal
                                     else "White-only fallback")
            lane_center_x = white_x - half_width
        else:
            self._lane_half_width_px = None
            self._lane_half_width_time = None
            self._lane_diagnostic = "Lane lost: no boundaries"
            return None, debug_image, debug_mask

        if not math.isfinite(lane_center_x) or not 0 <= lane_center_x < w:
            self._lane_both_visible = False
            self._lane_half_width_px = None
            self._lane_half_width_time = None
            self._lane_diagnostic = "Lane lost: centre outside image"
            return None, debug_image, debug_mask

        if yellow_x is not None and white_x is not None:
            self._lane_limits = (yellow_x, white_x)
        else:
            self._lane_limits = (lane_center_x - half_width, lane_center_x + half_width)

        target_center_x = self.lane_target_fraction * w
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

    def detect_ducks_bgr(self, image):
        """Compact yellow candidates, not semantic duck recognition or distance."""
        self.validate_image(image)
        h, w = image.shape[:2]
        mask = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV),
                           self.duck_lower, self.duck_upper)
        mask[:int(.45*h)] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for contour in contours:
            x, y, bw, bh = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            # Thin lane paint is rejected. A wide dash or merged duck/paint may
            # remain ambiguous; colour and silhouette cannot prove object identity.
            if (bw >= .045*w and bh >= .065*h and .45 <= bw/bh <= 2.5
                    and area >= .003*h*w and area >= .35*bw*bh):
                boxes.append((x, y, bw, bh))
        return sorted(boxes, key=lambda b: b[1]+b[3], reverse=True)

    def detect_road_geometry(self, image):
        """Require three ordered markings in three near-field image bands.

        No inferred lane widths or single-line fallback are allowed for passing.
        Pixel margins are provisional and must be measured at camera height.
        """
        h, w = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        yellow = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        white = cv2.inRange(hsv, self.white_lower, self.white_upper)
        for x, y, bw, bh in self._duck_boxes:
            yellow[y:y+bh, x:x+bw] = 0
            white[y:y+bh, x:x+bw] = 0

        def stripes(mask):
            columns = np.flatnonzero(np.count_nonzero(mask, axis=0) >= max(2, mask.shape[0]//3))
            groups = np.split(columns, np.where(np.diff(columns) > 1)[0]+1)
            return [float(np.mean(g)) for g in groups if 2 <= len(g) <= .055*w]

        samples = []
        for fraction in (.62, .74, .86):
            row = int(fraction*h)
            ys = stripes(yellow[row:row+max(4, int(.04*h))])
            ws = stripes(white[row:row+max(4, int(.04*h))])
            if len(ys) != 1 or len(ws) != 2:
                return None
            left, middle, right = ws[0], ys[0], ws[1]
            widths = (middle-left, right-middle)
            if not (0 < left < middle < right < w-1 and
                    all(.12*w <= v <= .48*w for v in widths)
                    and .5 <= widths[0]/widths[1] <= 2):
                return None
            samples.append((left, middle, right))
        # Tight turns and inconsistent geometry stop passing instead of extrapolating.
        if np.max(np.ptp(np.array(samples), axis=0)) > .08*w:
            return None
        return tuple(float(v) for v in np.mean(samples, axis=0))

    def avoidance_fault(self, reason):
        if self.avoidance_state not in ("idle", "fault"):
            self._control_epoch += 1
        self.avoidance_state = "fault"
        self._avoidance_reason = reason + "; stop and confirm placement before restart"
        self.obstacle_stop_latched = True
        self.manual_stop = True
        self.reset_steering()

    def avoidance_wheels(self, image, red_visible, obstacle_box, passing_lane_box):
        """Return None for normal navigation; otherwise own this frame's control.

        This is a restricted, experimental image-feedback pass on a straight,
        empty two-lane section. It never advances route position or changes turns.
        """
        active = self.avoidance_state != "idle"
        h, w = image.shape[:2]
        road = self._road_geometry
        now = time.monotonic()
        if not active:
            if not self.avoidance_enabled or not self.drive_enabled or obstacle_box is None:
                return None
            if (self.manual_stop or self.obstacle_stop_latched or self.red_stop_latched
                    or red_visible or self.navigation_state != "following"
                    or road is None or self.client_expired()
                    or (self.require_client_heartbeat and self._active_client_id is None)):
                return None
            left, middle, right = road
            targets = [b for b in self._duck_boxes
                       if b[0] > middle+.035*w and b[0]+b[2] < right-.035*w]
            if (len(self._duck_boxes) != 1 or len(targets) != 1 or obstacle_box not in targets
                    or targets[0][1]+targets[0][3] > .88*h):
                return None
            self.avoidance_state = "shift_left"
            self._avoidance_started = now
            self._avoidance_updated = now
            self._avoidance_clear_progress = 0.0
            self._avoidance_stable_since = self._avoidance_clear_since = None
            self._duck_passed_edge = False
            self._avoidance_reason = "Experimental pass: acquiring left lane"
            self.reset_steering()
        if self.avoidance_state == "fault":
            return 0.0, 0.0, 0.0
        if red_visible:
            self.red_stop_latched = True
            self._stop_started = now
        if (road is None or red_visible or self.red_stop_latched or self.manual_stop
                or self.navigation_state != "following" or self.client_expired()
                or now-self._avoidance_started > self.avoidance_timeout
                or not self.drive_enabled):
            self.avoidance_fault("Passing interrupted or road geometry unavailable")
            return 0.0, 0.0, 0.0
        left, middle, right = road
        # Actual camera centre must remain within the observed outer road edges.
        if not left+.035*w < .5*w < right-.035*w:
            self.avoidance_fault("Insufficient visible road margin")
            return 0.0, 0.0, 0.0
        if len(self._duck_boxes) > 1 or any(
                b[0] < middle+.025*w or b[0]+b[2] > right-.025*w
                for b in self._duck_boxes):
            self.avoidance_fault("Duck position or neighbouring lane clearance ambiguous")
            return 0.0, 0.0, 0.0
        if passing_lane_box is not None:
            self.avoidance_fault("Candidate in passing lane")
            return 0.0, 0.0, 0.0
        dt = min(max(now-self._avoidance_updated, 0.0), .1)
        self._avoidance_updated = now
        state = self.avoidance_state
        target = (middle+right)/2 if state == "return_right" else (left+middle)/2
        error = (target-.5*w)/(.5*w)
        if state == "shift_left":
            if not self._duck_boxes:
                self.avoidance_fault("Lost duck before establishing passing lane")
                return 0.0, 0.0, 0.0
            if self._duck_boxes[0][1]+self._duck_boxes[0][3] > .94*h:
                self.avoidance_fault("Duck too near before lane change completed")
                return 0.0, 0.0, 0.0
            if abs(error) < .08:
                if self._avoidance_stable_since is None:
                    self._avoidance_stable_since = now
                if now-self._avoidance_stable_since >= .3:
                    self.avoidance_state = "passing"
                    self._avoidance_stable_since = None
            else:
                self._avoidance_stable_since = None
        elif state == "passing":
            if self._duck_boxes:
                box = self._duck_boxes[0]
                self._duck_passed_edge = box[1]+box[3] >= .92*h and box[0] > .55*w
                self._avoidance_clear_since = None
                self._avoidance_clear_progress = 0.0
            elif not self._duck_passed_edge:
                self.avoidance_fault("Duck disappeared before side passage was observed")
                return 0.0, 0.0, 0.0
            else:
                if abs(error) >= .08:
                    self._avoidance_clear_since = None
                    self._avoidance_clear_progress = 0.0
                elif self._avoidance_clear_since is None:
                    self._avoidance_clear_since = now
                else:
                    # Command-weighted allowance handles speed scaling/ramp-up;
                    # it remains an estimate, not odometry or proof of clearance.
                    fraction = min(self._last_wheel_speeds) / self.avoidance_speed
                    self._avoidance_clear_progress += dt * min(1.0, fraction)
                    if self._avoidance_clear_progress >= self.avoidance_clear_seconds:
                        self.avoidance_state = "return_right"
                        self._avoidance_stable_since = None
                        self.reset_steering()
        elif state == "return_right":
            if self._duck_boxes or obstacle_box is not None:
                self.avoidance_fault("Return corridor contains an obstacle candidate")
                return 0.0, 0.0, 0.0
            if abs(error) < .08:
                if self._avoidance_stable_since is None:
                    self._avoidance_stable_since = now
                if now-self._avoidance_stable_since >= .3:
                    self.avoidance_state = "idle"
                    self._avoidance_reason = "Right lane reacquired; previous route retained"
                    self.reset_steering()
                    return 0.0, 0.0, 0.0
            else:
                self._avoidance_stable_since = None
        self._avoidance_reason = "Experimental pass: " + self.avoidance_state
        speeds = self.compute_wheel_speeds(error)
        cap = self.avoidance_speed * min(1.0, self.speed_scale)
        ratio = min(1.0, cap / max(speeds[0], speeds[1], .001))
        return speeds[0]*ratio, speeds[1]*ratio, speeds[2]

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
        for box in self._duck_boxes:
            x, y, bw, bh = box
            if x < x1 and x+bw > x0 and y+bh >= .68*h:
                return box
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
        candidates = []
        for contour in contours:
            x, y, width, height = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            if (width >= w * self.red_stop_min_width_fraction
                    and width >= 3 * height
                    and area >= h * w * 0.003):
                bottom_fraction = float(y0 + y + height) / h
                candidates.append({
                    "box": [int(x0 + x), int(y0 + y), int(width), int(height)],
                    "bottom_fraction": bottom_fraction,
                    "width_fraction": float(width) / w,
                    "area_fraction": float(area) / (h * w),
                })
        if not candidates:
            self._red_line_visible = False
            self._red_line_detection = None
            return False
        # The lowest valid transverse line is the nearest candidate.
        detection = max(
            candidates, key=lambda item: (item["bottom_fraction"], item["area_fraction"]))
        detection["trigger_fraction"] = self.red_stop_trigger_bottom_fraction
        detection["triggered"] = (
            detection["bottom_fraction"] >= self.red_stop_trigger_bottom_fraction)
        self._red_line_visible = True
        self._red_line_detection = detection
        return detection["triggered"]

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
                if self.smooth_steering_deadband:
                    # Attenuate small errors without a jump at +/-deadband.
                    # e*(2r-r^2), r=abs(e)/deadband, joins e with equal
                    # value and slope at the boundary; curve authority outside
                    # this region is unchanged. deadband=0 skips this branch.
                    ratio = abs(self.filtered_error) / self.deadband
                    control_error = self.filtered_error * ratio * (2.0 - ratio)
                else:
                    control_error = 0.0
            else:
                control_error = self.filtered_error

            effective_k_p = self.k_p
            if (self.near_center_k_p < self.k_p
                    and self.full_gain_error > self.deadband):
                magnitude = abs(self.filtered_error)
                if magnitude <= self.deadband:
                    effective_k_p = self.near_center_k_p
                elif magnitude < self.full_gain_error:
                    ratio = ((magnitude - self.deadband)
                             / (self.full_gain_error - self.deadband))
                    # Smoothstep avoids a gain jump at either boundary.
                    blend = ratio * ratio * (3.0 - 2.0 * ratio)
                    effective_k_p = (self.near_center_k_p
                                     + (self.k_p - self.near_center_k_p) * blend)

            raw_steering = effective_k_p * control_error + self.steering_bias
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

        if (self.drive_enabled and lane_error is not None
                and math.isfinite(lane_error)):
            active_floor = self.min_active_wheel_speed
            if self.taper_inner_wheel_floor and self.max_steering > 0:
                turn_fraction = min(1.0, abs(steering) / self.max_steering)
                # Keep the calibrated floor through ordinary corrections. Only
                # taper it during the final 10% of steering authority, where a
                # sharp bend needs pivot-like curvature. At full steering the
                # inner wheel may reach zero; the supervisor's encoder guard
                # still requires the commanded outside wheel to keep moving.
                taper_fraction = float(np.clip(
                    (turn_fraction - 0.90) / 0.10, 0.0, 1.0
                ))
                active_floor *= 1.0 - taper_fraction
            left_speed = max(left_speed, active_floor)
            right_speed = max(right_speed, active_floor)
        else:
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
        now = time.monotonic()
        junction_elapsed = (
            max(0.0, now - self._crossing_started)
            if (self._crossing_started is not None
                and self.navigation_state in ("crossing", "reacquiring"))
            else None)
        junction_deadline_remaining = (
            max(0.0, self._junction_deadline_at - now)
            if self._junction_deadline_at is not None
            else None)
        return {
            "vehicle": self.vehicle_name, "state": self.navigation_state,
            "drive_enabled": self.drive_enabled, "manual_stop": self.manual_stop,
            "red_stop": self.red_stop_latched, "speed_scale": self.speed_scale,
            "wheel_speeds": list(self._last_wheel_speeds),
            "camera_age": max(0.0, time.monotonic() - self._last_frame_time),
            "route_enabled": self.route_enabled, "route": list(self.route),
            "route_map_id": self.route_map_id,
            "start_approach": self.start_approach,
            "destination_approach": self.destination_approach,
            "next_junction": self.route[self.route_index] if self.route_enabled else None,
            "route_index": self.route_index, "active_turn": self._active_turn,
            "junctions_calibrated": self.junctions_calibrated,
            "junction_phase": self._junction_phase,
            "junction_elapsed_seconds": junction_elapsed,
            "junction_progress_seconds": (
                self._crossing_progress
                if self.navigation_state in ("crossing", "reacquiring") else None),
            "junction_deadline_remaining_seconds": junction_deadline_remaining,
            "junction_last_result": self._last_junction_result,
            "junction_settings": {
                "entry_seconds": self.junction_entry_seconds,
                "straight_seconds": self.junction_straight_seconds,
                "left_seconds": self.junction_left_seconds,
                "right_seconds": self.junction_right_seconds,
                "reacquire_timeout": self.junction_reacquire_timeout,
                "straight_speed": self.junction_straight_speed,
                "left_speed": self.junction_left_speed,
                "right_speed": self.junction_right_speed,
                "left_bias": self.junction_left_bias,
                "right_bias": self.junction_right_bias,
            },
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
            "lane_target_fraction": self.lane_target_fraction,
            "k_p": self.k_p,
            "near_center_k_p": self.near_center_k_p,
            "full_gain_error": self.full_gain_error,
            "max_steering": self.max_steering,
            "min_active_wheel_speed": self.min_active_wheel_speed,
            "taper_inner_wheel_floor": self.taper_inner_wheel_floor,
            "steering_bias": self.steering_bias,
            "smooth_steering_deadband": self.smooth_steering_deadband,
            "temporal_lane_width_fallback": self.temporal_lane_width_fallback,
            "temporal_lane_width_timeout": self.temporal_lane_width_timeout,
            "temporal_yellow_only_timeout": self.temporal_yellow_only_timeout,
            "boundary_risk_stop": self.boundary_risk_stop,
            "white_boundary_risk_fraction": self.white_boundary_risk_fraction,
            "lane_half_width_px": self._lane_half_width_px,
            "lane_half_width_age": (None if self._lane_half_width_time is None else
                                    max(0.0, time.monotonic() - self._lane_half_width_time)),
            "lane_both_visible": self._lane_both_visible,
            "yellow_boundary_visible": self._yellow_boundary_visible,
            "white_boundary_visible": self._white_boundary_visible,
            "red_line_visible": self._red_line_visible,
            "red_line_detection": self._red_line_detection,
            "red_stop_trigger_bottom_fraction": self.red_stop_trigger_bottom_fraction,
            "lane_limits": (None if self._lane_limits is None else list(self._lane_limits)),
            "sharp_corner_enabled": self.sharp_corner_enabled,
            "sharp_corner_state": self._sharp_corner_state,
            "sharp_corner_phase": self._sharp_corner_phase,
            "sharp_corner_confirm_seconds": self.sharp_corner_confirm_seconds,
            "sharp_corner_recent_lane_seconds": self.sharp_corner_recent_lane_seconds,
            "sharp_corner_approach_seconds": self.sharp_corner_approach_seconds,
            "sharp_corner_turn_speed": self.sharp_corner_turn_speed,
            "sharp_corner_min_turn_seconds": self.sharp_corner_min_turn_seconds,
            "sharp_corner_white_confirm_seconds": self.sharp_corner_white_confirm_seconds,
            "sharp_corner_pivot_seconds": self.sharp_corner_pivot_seconds,
            "sharp_corner_relief_seconds": self.sharp_corner_relief_seconds,
            "sharp_corner_relief_inner_speed": self.sharp_corner_relief_inner_speed,
            "sharp_corner_max_turn_seconds": self.sharp_corner_max_turn_seconds,
            "sharp_corner_reacquire_seconds": self.sharp_corner_reacquire_seconds,
            "sharp_corner_trigger_error": self.sharp_corner_trigger_error,
            "sharp_corner_exit_error": self.sharp_corner_exit_error,
            "yellow_lower": self.yellow_lower.tolist(),
            "yellow_upper": self.yellow_upper.tolist(),
            "white_lower": self.white_lower.tolist(),
            "white_upper": self.white_upper.tolist(),
            "filtered_lane_error": self.filtered_error,
            "steering_before_flip": self.prev_steering,
            "stop_reason": self._stop_reason,
            "lane_error": self._last_lane_error,
            "duck_candidates": list(self._duck_boxes),
            "road_geometry": self._road_geometry,
            "avoidance_enabled": self.avoidance_enabled,
            "avoidance_state": self.avoidance_state,
            "avoidance_reason": self._avoidance_reason,
        }

    def publish_status(self):
        self.status_publisher.publish(String(data=json.dumps(self.status(), allow_nan=False)))

    def navigation_fault(self, reason):
        self.navigation_state = "fault"
        self._junction_phase = "fault"
        self._last_junction_result = {
            "outcome": "fault", "reason": str(reason),
            "turn": self._active_turn,
        }
        self._junction_deadline_at = None
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
            if self._sharp_corner_state in ("approach", "turning"):
                self.sharp_corner_fault(
                    "Laptop connection lost during sharp corner; position must be reset")
            self.publish_wheels(0.0, 0.0)

    def check_camera_timeout(self, event):
        with self._wheel_lock:
            if self._stopping:
                return
            self.check_client_connection()
            if time.monotonic() - self._last_frame_time > self._camera_timeout:
                self._camera_valid = False
                self._camera_error = "Camera timeout"
                if self.avoidance_state not in ("idle", "fault"):
                    self.avoidance_fault("Camera timeout during passing")
                self._obstacle_clear_since = None
                if self.navigation_state in ("crossing", "reacquiring"):
                    self.navigation_fault("Camera lost during junction; position must be reset")
                if self._sharp_corner_state in ("approach", "turning"):
                    self.sharp_corner_fault(
                        "Camera lost during sharp corner; position must be reset")
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
                if self.avoidance_state != "idle" and action != "stop":
                    raise ValueError("Passing owns movement; stop and confirm placement before changing commands")
                if action == "stop":
                    if self.avoidance_state not in ("idle", "fault"):
                        self.avoidance_fault("Manual stop during passing")
                    self._control_epoch += 1
                    self._active_client_id = None
                    self._client_connection_lost = False
                    if self.navigation_state in ("crossing", "reacquiring"):
                        self.navigation_fault("Stopped within junction; position must be reset")
                    if self._sharp_corner_state in ("approach", "turning"):
                        self.sharp_corner_fault(
                            "Stopped during sharp corner; position must be reset")
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
                    if command.get("map_id", MAP_ID) != MAP_ID:
                        raise ValueError("Route map version does not match the controller")
                    start_approach = "%s->%s" % tuple(route[:2])
                    destination_approach = "%s->%s" % tuple(route[-2:])
                    if command.get("start_approach", start_approach) != start_approach:
                        raise ValueError("Starting lane does not match the route")
                    if command.get("destination_approach", destination_approach) != destination_approach:
                        raise ValueError("Destination red line does not match the route")
                    if command.get("position_confirmed") is not True:
                        raise ValueError("Confirm the directed starting lane and position outside a junction")
                    self.route, self.route_index = route, 1
                    self.route_map_id = MAP_ID
                    self.start_approach = start_approach
                    self.destination_approach = destination_approach
                    self.navigation_state = "following"
                    self.manual_stop = True
                    self.red_stop_latched = False
                    self._fault_reason = None
                    self._active_turn = None
                    self._junction_phase = "idle"
                    self._junction_deadline_at = None
                    self._last_junction_result = None
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
        duration, _, _ = self.junction_profile(self._active_turn)
        # Slow-down commands can stretch progress by at most four times.  The
        # wall-clock deadline remains independent and includes reacquisition.
        self._junction_deadline_at = (
            self._crossing_started
            + (self.junction_entry_seconds + duration) / 0.25
            + self.junction_reacquire_timeout + 2.0)
        self._junction_phase = "entry"
        self._last_junction_result = None
        self._red_clear_since = self._lane_good_since = None
        self._departed_red = False
        self.navigation_state = "crossing"
        self.red_stop_latched = False

    def junction_profile(self, turn=None):
        """Return direction-specific duration, centre speed and signed bias."""
        turn = self._active_turn if turn is None else turn
        if turn == "straight":
            return self.junction_straight_seconds, self.junction_straight_speed, 0.0
        if turn == "left":
            return self.junction_left_seconds, self.junction_left_speed, self.junction_left_bias
        if turn == "right":
            return self.junction_right_seconds, self.junction_right_speed, -self.junction_right_bias
        raise ValueError("Junction turn must be left, right, or straight")

    def planned_junction_wheels(self, apply_turn=True):
        """Continue an authorized crossing without relying on lane markings."""
        _, speed, bias = self.junction_profile()
        if not apply_turn:
            bias = 0.0
        scale = min(self.speed_scale, 1.0)
        speed *= scale
        bias *= scale
        return max(0.0, speed - bias), max(0.0, speed + bias), bias

    def reset_sharp_corner(self, state="idle"):
        self._sharp_corner_state = state
        self._sharp_corner_phase = state
        self._sharp_corner_candidate_since = None
        self._sharp_corner_state_since = None
        self._sharp_corner_reacquire_since = None
        self._sharp_corner_white_since = None
        self.reset_steering()

    def sharp_corner_fault(self, reason):
        self.reset_sharp_corner("fault")
        self.navigation_fault(reason)

    def sharp_corner_wheels(self, lane_error, red_visible):
        """Own a confirmed sharp right corner until its outgoing lane is stable.

        This is deliberately opt-in. A missing white boundary alone never
        starts motion: the node must have just seen a complete lane, retain a
        current yellow divider and observe a right-turn error for several
        frames. The maneuver remains bounded and preserves all final wheel
        publication gates.
        """
        if not self.sharp_corner_enabled:
            return None

        now = time.monotonic()
        active = self._sharp_corner_state in ("approach", "turning", "reacquiring")
        if red_visible:
            if active:
                self.reset_sharp_corner()
                self.red_stop_latched = True
                self._stop_started = now
                self.navigation_state = "red_stop"
                return 0.0, 0.0, 0.0
            return None
        if active and (self.manual_stop or self.obstacle_stop_latched
                       or self.navigation_state != "following"
                       or self.client_expired()
                       or (self.require_client_heartbeat
                           and self._active_client_id is None)):
            self.sharp_corner_fault(
                "Sharp corner interrupted; position must be reset")
            return 0.0, 0.0, 0.0
        if self._sharp_corner_state == "fault":
            return 0.0, 0.0, 0.0

        if self._sharp_corner_state == "cooldown":
            if now >= self._sharp_corner_cooldown_until:
                self.reset_sharp_corner()
            return None

        if self._sharp_corner_state == "idle":
            if self._lane_both_visible:
                self._sharp_corner_last_both_time = now
                self._sharp_corner_candidate_since = None
                return None
            recent_complete_lane = (
                self._sharp_corner_last_both_time is not None
                and now - self._sharp_corner_last_both_time
                <= self.sharp_corner_recent_lane_seconds
            )
            candidate = (
                recent_complete_lane
                and self._yellow_boundary_visible
                and not self._white_boundary_visible
                and lane_error is not None
                and math.isfinite(lane_error)
                and lane_error >= self.sharp_corner_trigger_error
            )
            if not candidate:
                self._sharp_corner_candidate_since = None
                return None
            if self._sharp_corner_candidate_since is None:
                self._sharp_corner_candidate_since = now
                return None
            if now - self._sharp_corner_candidate_since < self.sharp_corner_confirm_seconds:
                return None
            self._sharp_corner_state = "approach"
            self._sharp_corner_phase = "approach"
            self._sharp_corner_state_since = now
            self._sharp_corner_reacquire_since = None
            self.reset_steering()

        if self._sharp_corner_state == "approach":
            if not self._yellow_boundary_visible:
                self.sharp_corner_fault(
                    "Yellow boundary lost during sharp-corner approach; position must be reset")
                return 0.0, 0.0, 0.0
            if now - self._sharp_corner_state_since < self.sharp_corner_approach_seconds:
                speed = self.base_speed * min(self.speed_scale, 1.0)
                return speed, speed, 0.0
            self._sharp_corner_state = "turning"
            self._sharp_corner_phase = "pivot"
            self._sharp_corner_state_since = now
            self._sharp_corner_reacquire_since = None
            self._sharp_corner_white_since = None

        if now - self._sharp_corner_state_since > self.sharp_corner_max_turn_seconds:
            self.sharp_corner_fault(
                "Sharp corner exceeded its turn limit; position must be reset")
            return 0.0, 0.0, 0.0

        if self._sharp_corner_state == "turning":
            turn_elapsed = now - self._sharp_corner_state_since
            white_handoff = (
                turn_elapsed >= self.sharp_corner_min_turn_seconds
                and self._white_boundary_visible
            )
            if white_handoff:
                if self._sharp_corner_white_since is None:
                    self._sharp_corner_white_since = now
                elif (now - self._sharp_corner_white_since
                      >= self.sharp_corner_white_confirm_seconds):
                    self._sharp_corner_state = "reacquiring"
                    self._sharp_corner_phase = "lane_reacquire"
                    self._sharp_corner_reacquire_since = None
                    self._sharp_corner_white_since = None
                    self.reset_steering()
            else:
                self._sharp_corner_white_since = None
                if not self._yellow_boundary_visible:
                    self.sharp_corner_fault(
                        "Lane boundaries lost during sharp corner; position must be reset")
                    return 0.0, 0.0, 0.0

        if self._sharp_corner_state == "reacquiring":
            if lane_error is None or not math.isfinite(lane_error):
                self.sharp_corner_fault(
                    "Outgoing lane lost during sharp-corner reacquisition; position must be reset")
                return 0.0, 0.0, 0.0
            aligned = (self._lane_both_visible
                       and abs(lane_error) <= self.sharp_corner_exit_error)
            if aligned:
                if self._sharp_corner_reacquire_since is None:
                    self._sharp_corner_reacquire_since = now
                elif (now - self._sharp_corner_reacquire_since
                      >= self.sharp_corner_reacquire_seconds):
                    self._sharp_corner_state = "cooldown"
                    self._sharp_corner_phase = "cooldown"
                    self._sharp_corner_cooldown_until = now + 1.0
                    self._sharp_corner_candidate_since = None
                    self._sharp_corner_state_since = None
                    self._sharp_corner_reacquire_since = None
                    self.reset_steering()
            else:
                self._sharp_corner_reacquire_since = None
            return self.compute_wheel_speeds(lane_error)

        speed_scale = min(self.speed_scale, 1.0)
        speed = self.sharp_corner_turn_speed * speed_scale
        cycle_seconds = (self.sharp_corner_pivot_seconds
                         + self.sharp_corner_relief_seconds)
        cycle_phase = ((now - self._sharp_corner_state_since)
                       % cycle_seconds)
        if (self.sharp_corner_relief_inner_speed > 0
                and cycle_phase >= self.sharp_corner_pivot_seconds):
            # A short rolling interval releases the scrub load created by a
            # stationary inside wheel. The outside wheel stays at the full
            # turn request, so the confirmed corner remains latched.
            inner = self.sharp_corner_relief_inner_speed * speed_scale
            self._sharp_corner_phase = "rolling_relief"
            return speed, inner, -(speed - inner) / 2.0
        self._sharp_corner_phase = "pivot"
        # A right pivot drives the outside (left) wheel and releases the inner
        # wheel. This interval is deliberately shorter than the measured
        # loaded-stall interval; the encoder watchdog remains authoritative.
        return speed, 0.0, -speed / 2.0

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
            duration, _, _ = self.junction_profile()
            dt = min(max(now - self._crossing_updated, 0.0), 0.1)
            self._crossing_updated = now
            self._crossing_progress += dt * min(self.speed_scale, 1.0)
            elapsed = self._crossing_progress
            if (self._junction_deadline_at is None
                    or now >= self._junction_deadline_at):
                self.navigation_fault("Junction traversal exceeded its time limit")
                return 0.0, 0.0, 0.0
            if elapsed >= self.junction_entry_seconds + duration:
                if state == "crossing":
                    self.navigation_state = "reacquiring"
                    self._reacquire_started = now
                    self._junction_phase = "searching"
                good_lane = (self._departed_red and self._lane_both_visible
                             and lane_error is not None and abs(lane_error) < 0.35)
                if good_lane:
                    self._junction_phase = "aligning"
                    if self._lane_good_since is None:
                        self._lane_good_since = now
                    if now - self._lane_good_since >= 0.3:
                        completed_turn = self._active_turn
                        self.route_index += 1
                        self.navigation_state = "following"
                        self._active_turn = None
                        self._junction_phase = "complete"
                        self._junction_deadline_at = None
                        self._last_junction_result = {
                            "outcome": "reacquired",
                            "turn": completed_turn,
                            "route_index": self.route_index,
                        }
                        self.filtered_error = self.prev_steering = 0.0
                        return self.compute_wheel_speeds(lane_error)
                else:
                    self._lane_good_since = None
                    self._junction_phase = "searching"
                if now - self._reacquire_started >= self.junction_reacquire_timeout:
                    self.navigation_fault("Outgoing lane was not reacquired; position must be reset")
                    return 0.0, 0.0, 0.0
                # Fresh unmarked intersection images are expected here. Keep
                # executing the selected bounded profile. Once both outgoing
                # borders are visible, use lane steering to settle centrally.
                if not self._lane_both_visible or lane_error is None:
                    return self.planned_junction_wheels()
                left, right, steering = self.compute_wheel_speeds(lane_error)
                _, junction_speed, _ = self.junction_profile()
                ratio = min(1.0, junction_speed / max(left, right, 0.001))
                return left * ratio, right * ratio, steering
            self._junction_phase = (
                "entry" if elapsed < self.junction_entry_seconds
                else "straight" if self._active_turn == "straight" else "turning")
            return self.planned_junction_wheels(
                apply_turn=elapsed >= self.junction_entry_seconds)
        if red_visible and not self.red_stop_latched:
            self.red_stop_latched = True
            self._stop_started = now
            if self.route_enabled and self.route_index == len(self.route)-1:
                self.navigation_state = "route_complete"
                self._junction_phase = "route_complete"
            elif state == "following":
                self.navigation_state = "red_stop"
                self._junction_phase = "stopped"
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
            if self.avoidance_state not in ("idle", "fault"):
                self.avoidance_fault("Invalid camera during passing")
            self._lane_limits = None
            self._lane_both_visible = False
            self._yellow_boundary_visible = False
            self._white_boundary_visible = False
            self._red_line_visible = False
            self._red_line_detection = None
            self._lane_half_width_px = None
            self._lane_half_width_time = None
            self._last_lane_error = None
            self._lane_diagnostic = "Image rejected"
            self._obstacle_clear_since = None
            if self.navigation_state in ("crossing", "reacquiring"):
                self.navigation_fault("Invalid camera image during junction: " + str(reason))
            if self._sharp_corner_state in ("approach", "turning"):
                self.sharp_corner_fault(
                    "Invalid camera during sharp corner; position must be reset")
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
                # Disabled obstacle handling must not remove yellow divider
                # pixels from ordinary lane perception. Duck candidates are
                # only relevant when obstacle handling has been deliberately
                # enabled for a calibrated session.
                perception._duck_boxes = (
                    perception.detect_ducks_bgr(img_bgr)
                    if perception.obstacle_enabled else []
                )
                perception._road_geometry = perception.detect_road_geometry(img_bgr)
                red_visible = perception.detect_red_stop(img_bgr)
                lane_error, debug_image, debug_mask = perception.detect_lane_bgr(img_bgr)
                obstacle_box = perception.detect_obstacle_bgr(img_bgr)
                passing_lane_box = None
                if perception._road_geometry is not None:
                    limits = perception._lane_limits
                    perception._lane_limits = perception._road_geometry[:2]
                    passing_lane_box = perception.detect_obstacle_bgr(img_bgr)
                    perception._lane_limits = limits
                with self._wheel_lock:
                    if self._stopping:
                        return
                    self.camera_stamp(msg, received_at)
                    if received_at - self._last_frame_time > self._camera_timeout:
                        self._obstacle_clear_since = None
                        if self.avoidance_state not in ("idle", "fault"):
                            self.avoidance_fault("Camera gap during passing")
                    self._last_frame_time = time.monotonic() - max(
                        time.monotonic() - received_at, rospy.Time.now().to_sec() - stamp, 0.0)
                    self._last_camera_stamp = stamp
                    self._camera_valid = True
                    self._camera_error = None
                    self._last_lane_error = lane_error
                    self._lane_limits = perception._lane_limits
                    self._lane_both_visible = perception._lane_both_visible
                    self._yellow_boundary_visible = perception._yellow_boundary_visible
                    self._white_boundary_visible = perception._white_boundary_visible
                    self._red_line_visible = perception._red_line_visible
                    self._red_line_detection = perception._red_line_detection
                    self._lane_half_width_px = perception._lane_half_width_px
                    self._lane_half_width_time = perception._lane_half_width_time
                    self._lane_diagnostic = perception._lane_diagnostic
                    self._duck_boxes = perception._duck_boxes
                    self._road_geometry = perception._road_geometry
                    self.check_client_connection()
                    avoidance = self.avoidance_wheels(img_bgr, red_visible, obstacle_box, passing_lane_box)
                    if avoidance is None:
                        self.update_obstacle(obstacle_box)
                        corner = self.sharp_corner_wheels(lane_error, red_visible)
                        if corner is None:
                            left_speed, right_speed, steering = self.navigation_wheels(
                                lane_error, red_visible)
                        else:
                            left_speed, right_speed, steering = corner
                    else:
                        self._obstacle_box = obstacle_box
                        self._obstacle_visible = obstacle_box is not None
                        self._obstacle_clear_since = None
                        left_speed, right_speed, steering = avoidance
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
            cv2.putText(debug_image, self._avoidance_reason, (10, 140),
                        cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 0, 255), 1)
            if self._road_geometry is not None:
                for boundary in self._road_geometry:
                    cv2.line(debug_image, (int(boundary), int(.62*img_bgr.shape[0])),
                             (int(boundary), int(.90*img_bgr.shape[0])), (255, 200, 0), 1)
            for x, y, bw, bh in self._duck_boxes:
                cv2.rectangle(debug_image, (x, y), (x+bw, y+bh), (0, 165, 255), 2)
            if self._obstacle_box is not None:
                x, y, width, height = self._obstacle_box
                cv2.rectangle(debug_image, (x, y), (x+width, y+height), (255, 0, 255), 2)
                cv2.putText(debug_image, "Obstacle candidate", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            if self.red_stop_latched:
                cv2.putText(debug_image, "STOP: red line", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            if self._red_line_detection is not None:
                x, y, width, height = self._red_line_detection["box"]
                cv2.rectangle(debug_image, (x, y), (x+width, y+height),
                              (0, 80, 255), 2)
                cv2.putText(
                    debug_image,
                    "red bottom %.3f / trigger %.3f" % (
                        self._red_line_detection["bottom_fraction"],
                        self.red_stop_trigger_bottom_fraction),
                    (10, 165), cv2.FONT_HERSHEY_SIMPLEX, .45,
                    (0, 80, 255), 1)
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
        # DTROS invokes this hook even when construction failed before the
        # wheel lock exists. There is no publisher to stop in that case.
        if not hasattr(self, "_wheel_lock"):
            return
        with self._wheel_lock:
            if self._stopping:
                return
            self._stopping = True
            if self.avoidance_state not in ("idle", "fault"):
                self.avoidance_fault("Shutdown during passing")
            self.publish_wheels(0.0, 0.0)
        if getattr(self, "show_debug", False):
            cv2.destroyAllWindows()
        rospy.loginfo("LaneFollowerNode stopped; wheels set to zero")


if __name__ == "__main__":
    lane_follower_node = LaneFollowerNode(node_name="lane_follower_node")
    rospy.spin()
