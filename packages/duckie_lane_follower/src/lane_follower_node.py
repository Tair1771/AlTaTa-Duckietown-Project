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
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from duckie_lane_follower.route_map import (
    MAP_ID, MAP_PORTS, junction_turn, route_via_turn, validate_route)
from duckie_lane_follower.live_session import LiveSession, PROFILES, STRAIGHT_WHEEL_CAP


# Test-only visual right-junction clearance after confirmed white-boundary loss.
# One second carried the robot too far into the intersection in the recorded run.
RIGHT_JUNCTION_CLEARANCE_SECONDS = 0.25


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
        self.junction_right_tracking_trim = self.number_param("~junction_right_tracking_trim", 0.0)
        self.junction_right_encoder_assist = rospy.get_param("~junction_right_encoder_assist", False)
        if type(self.junction_right_encoder_assist) is not bool:
            raise ValueError("Right encoder assist must be boolean")
        self._right_alignment_reference = None
        self._right_alignment_adjustment = 0.0
        if not 0.0 <= self.junction_right_tracking_trim <= 0.01:
            raise ValueError("Right junction tracking trim must be in [0, 0.01]")
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
        self.live = LiveSession()
        self.stop_hold_seconds = self.number_param("~stop_hold_seconds", 2.0)
        self.junction_entry_seconds = self.number_param("~junction_entry_seconds", 0.5)
        self.junction_turn_seconds = self.number_param("~junction_turn_seconds", 1.6)
        self.junction_straight_seconds = self.number_param("~junction_straight_seconds", 1.0)
        self.junction_reacquire_timeout = self.number_param("~junction_reacquire_timeout", 3.0)
        self.junction_reacquire_max_error = self.number_param(
            "~junction_reacquire_max_error", 0.35)
        self.junction_speed = self.number_param("~junction_speed", 0.05)
        self.junction_bias = self.number_param("~junction_bias", 0.025)
        # Direction-specific settings default to the original shared values.
        self.junction_left_seconds = self.number_param(
            "~junction_left_seconds", self.junction_turn_seconds)
        self.junction_right_seconds = self.number_param(
            "~junction_right_seconds", self.junction_turn_seconds)
        self.junction_straight_speed = self.number_param(
            "~junction_straight_speed", self.junction_speed)
        # Optional straight-intersection aids. They remain disabled by default
        # and are enabled only by the bounded, physically supervised preset.
        self.junction_straight_approach_max_steering = self.number_param(
            "~junction_straight_approach_max_steering", self.max_steering)
        self.junction_straight_visual_approach = rospy.get_param(
            "~junction_straight_visual_approach", False)
        self.junction_white_boundary_guard = rospy.get_param(
            "~junction_white_boundary_guard", False)
        self.junction_straight_lane_target_fraction = self.number_param(
            "~junction_straight_lane_target_fraction", self.lane_target_fraction)
        self.junction_straight_lateral_gain = self.number_param(
            "~junction_straight_lateral_gain", 0.0)
        self.junction_straight_heading_gain = self.number_param(
            "~junction_straight_heading_gain", 0.0)
        self.junction_straight_departure_max_heading = self.number_param(
            "~junction_straight_departure_max_heading", 0.20)
        self.junction_straight_reacquire_max_lateral = self.number_param(
            "~junction_straight_reacquire_max_lateral", 0.12)
        self.junction_straight_reacquire_max_heading = self.number_param(
            "~junction_straight_reacquire_max_heading", 0.10)
        self.junction_straight_reacquire_seconds = self.number_param(
            "~junction_straight_reacquire_seconds", 0.30)
        self.junction_straight_settle_max_lateral = self.number_param(
            "~junction_straight_settle_max_lateral", 0.08)
        self.junction_straight_settle_max_heading = self.number_param(
            "~junction_straight_settle_max_heading", 0.06)
        self.junction_straight_settle_max_steering = self.number_param(
            "~junction_straight_settle_max_steering", 0.03)
        self.junction_straight_settle_seconds = self.number_param(
            "~junction_straight_settle_seconds", 0.50)
        self.junction_straight_encoder_balance = rospy.get_param(
            "~junction_straight_encoder_balance", False)
        self.junction_straight_encoder_balance_gain = self.number_param(
            "~junction_straight_encoder_balance_gain", 0.0)
        self.junction_straight_encoder_balance_max = self.number_param(
            "~junction_straight_encoder_balance_max", 0.0)
        self.junction_straight_encoder_balance_min_ticks = self.number_param(
            "~junction_straight_encoder_balance_min_ticks", 12.0)
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
        self._junction_red_reappeared = False
        self._junction_right_white_seen = False
        self._junction_right_absent_since = None
        self._junction_right_advance_started = None
        self._junction_right_turn_origin = None
        self._junction_right_visual_entry = False
        self._junction_right_corridor_since = None
        self._junction_right_near_since = None
        self._junction_right_red_rearmed = False
        self._left_encoder_tick = self._right_encoder_tick = None
        self._left_encoder_time = self._right_encoder_time = None
        self._junction_encoder_start = None
        self._junction_encoder_adjustment = 0.0
        self._junction_lane_geometry = None
        self._junction_white_geometry = None
        self._junction_white_width_reference = None
        self._junction_white_guard_used = False
        self._junction_white_guard_reason = None
        self._junction_exit_geometry = None
        self._junction_straight_visual_entry = False
        self._junction_straight_corridor_since = None
        self._junction_approach_geometry = None
        self._junction_approach_geometry_time = None
        self._junction_approach_active = False
        self._junction_stop_geometry = None
        self._junction_approach_steering = 0.0
        self._junction_approach_control_time = time.monotonic()
        self._junction_settle_good_since = None
        self._junction_settled = False
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
                      self.junction_right_speed,
                      self.junction_straight_encoder_balance_min_ticks):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Speed and junction settings must be finite and positive")
        if not 0 < self.alpha <= 1:
            raise ValueError("alpha must be in (0, 1]")
        if not 0 < self.junction_reacquire_max_error <= 1:
            raise ValueError("Junction reacquisition error must be in (0, 1]")
        self.validate_settings()
        self.yellow_lower, self.yellow_upper = self.color_range("yellow", [24, 140, 120], [36, 255, 255])
        self.white_lower, self.white_upper = self.color_range("white", [0, 0, 170], [180, 55, 255])
        # Keep the ordinary-road centroid calibrated to the accepted bright
        # border, independently of the lower threshold needed at dim corners.
        self.road_white_reference_value = self.number_param(
            "~road_white_reference_value", float(self.white_lower[2]))
        if not self.white_lower[2] <= self.road_white_reference_value <= self.white_upper[2]:
            raise ValueError("road_white_reference_value must be within white value bounds")
        self._road_white_reference_used = False
        self.road_heading_guard = rospy.get_param("~road_heading_guard", False)
        self.road_left_lookahead = rospy.get_param("~road_left_lookahead", False)
        if type(self.road_left_lookahead) is not bool:
            raise ValueError("Road left lookahead must be boolean")
        self.road_left_curve_boost = rospy.get_param("~road_left_curve_boost", False)
        if type(self.road_left_curve_boost) is not bool:
            raise ValueError("Road left curve boost must be boolean")
        self._junction_forward_geometry = None
        self._junction_forward_since = None
        self._junction_forward_last_seen = None
        self._junction_forward_used = False
        self._road_left_shape = None
        self._road_left_boost_since = None
        self._road_left_boost_active = False
        self._road_left_boost_last_seen = None
        self._road_left_boost_white_trace = None
        self._road_left_boost_gap_active = False
        self._road_curve_geometry = None
        self._road_left_lookahead_used = False
        self._left_curve_since = None
        self._left_curve_confirmed_at = None
        self._left_curve_steering = None
        self._left_curve_white_x = None
        self._road_left_gap_active = False
        self.junction_left_visual_latch = rospy.get_param("~junction_left_visual_latch", False)
        if type(self.road_heading_guard) is not bool or type(self.junction_left_visual_latch) is not bool:
            raise ValueError("Road heading guard and left visual latch must be boolean")
        self._road_heading_guard_used = False
        self._junction_left_visual_entry = False
        self._junction_left_corridor_since = None
        self._junction_left_near_since = None
        self._junction_left_red_rearmed = False
        self._junction_stop_white_seen = False
        self.red_low_lower, self.red_low_upper = self.color_range("red_low", [0, 110, 90], [10, 255, 255])
        self.red_high_lower, self.red_high_upper = self.color_range("red_high", [170, 110, 90], [180, 255, 255])
        self._lane_diagnostic = "No valid image yet"
        self._camera_error = "Waiting for camera"
        self._stop_reason = "Waiting for camera"
        self._last_camera_stamp = None
        self._camera_processing_started_at = None
        self._last_camera_received_at = None
        self._camera_processing_seconds = None
        self._last_camera_timeout = None
        self._camera_valid = False
        self._camera_lock = threading.Lock()
        self._executed_wheels = None
        self._executed_wheels_time = None
        self._executed_samples = 0
        self._executed_zero_samples = 0
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
        self.left_encoder_subscriber = rospy.Subscriber(
            f"/{self.vehicle_name}/left_wheel_encoder_node/tick",
            WheelEncoderStamped, self.left_encoder_callback, queue_size=10,
        )
        self.right_encoder_subscriber = rospy.Subscriber(
            f"/{self.vehicle_name}/right_wheel_encoder_node/tick",
            WheelEncoderStamped, self.right_encoder_callback, queue_size=10,
        )
        self.executed_wheels_subscriber = rospy.Subscriber(
            f"/{self.vehicle_name}/wheels_driver_node/wheels_cmd_executed",
            WheelsCmdStamped, self.executed_wheels_callback, queue_size=20,
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
                     "sharp_corner_enabled", "junction_straight_encoder_balance",
                     "junction_straight_visual_approach", "junction_white_boundary_guard"):
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
        if self.sharp_corner_enabled and self.avoidance_enabled:
            raise ValueError("Sharp-corner mode cannot be combined with obstacle passing")
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
        if not (0 <= self.junction_straight_approach_max_steering <= self.max_steering
                and 0 <= self.junction_straight_encoder_balance_gain <= 1
                and 0 <= self.junction_straight_encoder_balance_max
                <= min(self.junction_straight_speed,
                       self.max_speed - self.junction_straight_speed)
                and self.junction_straight_encoder_balance_min_ticks >= 1):
            raise ValueError(
                "Invalid straight-junction steering or encoder-balance settings")
        if (self.junction_straight_encoder_balance
                and (self.junction_straight_encoder_balance_gain <= 0
                     or self.junction_straight_encoder_balance_max <= 0)):
            raise ValueError(
                "Straight-junction encoder balancing needs positive gain and limit")
        if not (0.25 <= self.junction_straight_lane_target_fraction <= 0.75
                and 0 <= self.junction_straight_lateral_gain <= 1
                and 0 <= self.junction_straight_heading_gain <= 1
                and 0 < self.junction_straight_departure_max_heading <= 0.5
                and 0 < self.junction_straight_reacquire_max_lateral <= 0.5
                and 0 < self.junction_straight_reacquire_max_heading <= 0.5
                and 0.2 <= self.junction_straight_reacquire_seconds <= 1.0
                and 0 < self.junction_straight_settle_max_lateral
                <= self.junction_straight_reacquire_max_lateral
                and 0 < self.junction_straight_settle_max_heading
                <= self.junction_straight_reacquire_max_heading
                and 0 <= self.junction_straight_settle_max_steering
                <= self.max_steering
                and 0.3 <= self.junction_straight_settle_seconds <= 2.0):
            raise ValueError("Invalid straight-junction visual alignment settings")
        if (self.junction_straight_visual_approach
                and (self.junction_straight_lateral_gain <= 0
                     or self.junction_straight_heading_gain <= 0)):
            raise ValueError("Visual straight-junction control needs positive gains")

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
        self._junction_white_guard_used = False
        self._junction_white_guard_reason = None
        self._junction_white_width_reference = None
        self._junction_forward_since = None
        self._junction_forward_last_seen = None
        self._junction_forward_used = False
        self._road_left_boost_since = None
        self._road_left_boost_active = False
        self._road_left_boost_last_seen = None
        self._road_left_boost_white_trace = None
        self._road_left_boost_gap_active = False
        self._left_curve_since = self._left_curve_confirmed_at = None
        self._left_curve_steering = None
        self._road_left_gap_active = False
        self._right_alignment_reference = None
        self._right_alignment_adjustment = 0.0
        self.filtered_error = self.prev_steering = 0.0
        self._last_control_time = time.monotonic()

    def left_encoder_callback(self, msg):
        self._encoder_callback("left", msg)

    def executed_wheels_callback(self, msg):
        """Read-only driver evidence; these values never command the motors."""
        try:
            values = (float(msg.vel_left), float(msg.vel_right))
            if not all(math.isfinite(v) for v in values):
                return
        except (AttributeError, TypeError, ValueError):
            return
        with self._wheel_lock:
            self._executed_wheels = values
            self._executed_wheels_time = time.monotonic()
            self._executed_samples += 1
            self._executed_zero_samples += int(values == (0.0, 0.0))

    def right_encoder_callback(self, msg):
        self._encoder_callback("right", msg)

    def _encoder_callback(self, side, msg):
        """Keep the latest raw tick count for bounded straight-crossing balance."""
        try:
            value = int(msg.data)
        except (AttributeError, TypeError, ValueError, OverflowError):
            return
        with self._wheel_lock:
            setattr(self, "_%s_encoder_tick" % side, value)
            setattr(self, "_%s_encoder_time" % side, time.monotonic())

    def upcoming_junction_turn(self):
        if not self.route_enabled or self.route_index >= len(self.route) - 1:
            return None
        try:
            return junction_turn(
                *self.route[self.route_index-1:self.route_index+2])
        except (KeyError, TypeError, ValueError):
            return None

    def initial_straight_approach_active(self):
        """Explicitly confirmed straight start, only before the first junction."""
        return (self.live.enabled and self.live.active and self.live.center_initial_straight
                and self.route_index == 1 and self.navigation_state == "following")

    def straight_approach_wheels(self, left, right, steering):
        """Use row-matched lane geometry throughout an authorized approach."""
        if (not self.junctions_calibrated
                or not self.junction_straight_visual_approach
                or (not self.live.active and self.upcoming_junction_turn() not in ("straight", "left", "right"))):
            return left, right, steering
        # Ordinary lane following owns roads and curves between junctions.
        # Once a transverse red line becomes visible, latch the dedicated
        # approach controller so brief red-detector flicker cannot switch it.
        initial_straight = self.initial_straight_approach_active()
        if not self._junction_approach_active:
            if not self._red_line_visible and not initial_straight:
                return left, right, steering
            self._junction_approach_active = True
            # Do not seed a confirmed straight with centroid/trim steering.
            self._junction_approach_steering = 0.0 if initial_straight else steering
            self._junction_approach_control_time = time.monotonic()
        geometry = self._junction_lane_geometry
        if not self.junction_geometry_steering_valid(geometry):
            # A normal lane estimate still keeps lane-loss stopping active, but
            # an untrustworthy row fit must not inherit centroid steering or trim.
            if self._last_lane_error is None:
                return 0.0, 0.0, 0.0
            desired = 0.0
        else:
            self._junction_approach_geometry = copy.deepcopy(geometry)
            self._junction_approach_geometry_time = time.monotonic()
            desired = self.straight_visual_steering(
                geometry, self.junction_straight_approach_max_steering)
        now = time.monotonic()
        dt = min(max(now - self._junction_approach_control_time, 0.001), 0.1)
        change = self.max_steering_change * dt * 30.0
        self._junction_approach_control_time = now
        if initial_straight:
            # Smooth row-fit jitter with the existing frame-rate-aware filter;
            # keep the same gains, steering cap, lane target and mean speed.
            weight = 1.0 - (1.0 - self.alpha) ** (dt * 30.0)
            desired = self._junction_approach_steering + weight * (desired - self._junction_approach_steering)
        limited = float(np.clip(
            desired,
            self._junction_approach_steering - change,
            self._junction_approach_steering + change,
        ))
        self._junction_approach_steering = limited
        centre = self.base_speed * self.speed_scale
        return self.right_junction_tracking_wheels(
            float(np.clip(centre - limited, 0.0, self.max_speed)),
            float(np.clip(centre + limited, 0.0, self.max_speed)), limited)

    def right_junction_tracking_wheels(self, left, right, steering):
        """Small final wheel-space trim; never used by the fixed pivot."""
        if (not self.junction_straight_visual_approach or not self.junctions_calibrated
                or (self._active_turn or self.upcoming_junction_turn()) != "right"
                or min(left, right) <= 0.0):
            return left, right, steering
        trim = self.junction_right_tracking_trim
        left = max(min(left, self.min_active_wheel_speed), left - trim)
        right = min(self.max_speed, right + trim)
        return left, right, (right - left) / 2.0

    def straight_visual_steering(self, geometry, limit=None):
        """Convert separate lateral and heading evidence into final steering."""
        if not self.junction_geometry_steering_valid(geometry):
            return 0.0
        # A positive lateral error means the lane is to the image right. A
        # positive heading error means its near centre is right of its far
        # centre, which indicates the robot points toward the right boundary.
        raw = (self.junction_straight_lateral_gain * geometry["lateral_error"]
               - self.junction_straight_heading_gain * geometry["heading_error"])
        if self.flip_steering:
            raw = -raw
        return float(np.clip(
            raw,
            -(self.max_steering if limit is None else limit),
            self.max_steering if limit is None else limit,
        ))

    def straight_forward_geometry(self, image):
        """Observe the distant straight exit above the normal road ROI.

        Only supports bounded aiming during an authorized straight crossing.
        It never counts as near-lane reacquisition or advances route progress.
        """
        h, w = image.shape[:2]
        start, end = int(.29*h), int(min(.50, self.roi_y0_fraction)*h)
        result = dict(usable=False, source="distant_straight_exit", pairs=[])
        if end-start < 20:
            return result
        hsv = cv2.cvtColor(image[start:end], cv2.COLOR_BGR2HSV)
        yellow = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        white = cv2.inRange(hsv, self.white_lower, self.white_upper)
        pairs = []
        for row in range(2, end-start-2, 3):
            def runs(mask, max_width):
                columns = np.flatnonzero(np.count_nonzero(mask[row-1:row+2], axis=0) >= 2)
                return [(float(group.mean()), len(group)) for group in
                        np.split(columns, np.where(np.diff(columns)>1)[0]+1)
                        if 2 <= len(group) <= max_width*w]
            choices = []
            for yx, _ in runs(yellow, .05):
                for wx, _ in runs(white, .08):
                    center = (yx+wx)/2
                    if (yx < self.yellow_right_cutoff*w and wx >= self.white_left_cutoff*w
                            and .08*w <= wx-yx <= .55*w and .25*w <= center <= .75*w):
                        choices.append((yx, wx, center))
            if not choices:
                continue
            reference = pairs[-1]["center_x"] if pairs else .5*w
            yx, wx, center = min(choices, key=lambda v: abs(v[2]-reference))
            if pairs and (abs(center-reference) > .035*w
                          or start+row-pairs[-1]["row"] > .035*h
                          or wx-yx < pairs[-1]["width"]-.04*w):
                continue
            pairs.append(dict(row=start+row, yellow_x=yx, white_x=wx,
                              center_x=center, width=wx-yx))
        result["pairs"] = pairs
        if len(pairs) < 6 or pairs[-1]["row"]-pairs[0]["row"] < .04*h:
            return result
        # A corner fragment at the far end can spoil an otherwise consistent
        # outgoing corridor. Only discard a small prefix; never search arbitrary
        # subsets, widen the ROI, or relax ordering, span and fit requirements.
        for trimmed in range(min(2, len(pairs)//5) + 1):
            corridor = pairs[trimmed:]
            if len(corridor) < 6 or corridor[-1]["row"]-corridor[0]["row"] < .04*h:
                continue
            rows = np.array([p["row"] for p in corridor], dtype=float)
            fits = {}
            for key in ("yellow_x", "white_x", "center_x"):
                values = np.array([p[key] for p in corridor], dtype=float)
                fits[key] = np.polyfit(rows, values, 1)
                if np.max(np.abs(values-np.polyval(fits[key], rows))) > .015*w:
                    break
            else:
                # Discarded rows must actually disagree with the retained fit.
                # This prevents trimming merely to satisfy the opening gate.
                if trimmed and not any(
                        abs(p[key]-np.polyval(fits[key], p["row"])) > .015*w
                        for p in pairs[:trimmed] for key in fits):
                    continue
                # A longitudinal corridor opens toward the camera; reject
                # parallel transverse fragments and implausible narrowing.
                if corridor[-1]["width"] < corridor[0]["width"]+.02*w:
                    continue
                result.update(
                    usable=True, pairs=corridor, trimmed_far_pairs=trimmed,
                    center_x=float(np.median([p["center_x"] for p in corridor])),
                    image_width=w)
                break
        return result

    def straight_forward_wheels(self, speed):
        """Low-authority aiming; ordinary lane steering takes priority later."""
        now = time.monotonic()
        g = self._junction_forward_geometry or {}
        eligible = (self._active_turn == "straight"
                    and self.navigation_state in ("crossing", "reacquiring")
                    and (self.require_client_heartbeat or self.live.enabled)
                    and g.get("usable", False) and self._camera_valid
                    and 0 <= now-self._last_frame_time <= self._camera_timeout)
        self._junction_forward_used = False
        if not eligible:
            self._junction_forward_since = self._junction_forward_last_seen = None
            return None
        # Confirmation duration is not a minimum camera frame rate. A fresh
        # 0.25/0.30-second callback cadence must not repeatedly erase an already
        # observed corridor. Missing/invalid geometry still clears it above.
        if (self._junction_forward_since is None or self._junction_forward_last_seen is None
                or not 0 <= now-self._junction_forward_last_seen <= self._camera_timeout):
            self._junction_forward_since = now
        self._junction_forward_last_seen = now
        if now-self._junction_forward_since < .2-1e-9:
            return None
        # Aim at the optical centre, without importing the near-road lateral
        # target/trim into this distant bearing. Six-pixel-scale deadband avoids
        # steering on tiny dash-edge shifts in an already aligned exit.
        error = (g["center_x"]-.5*g["image_width"])/(.5*g["image_width"])
        excess = math.copysign(max(0.0, abs(error)-.02), error)
        steering = float(np.clip(.25*excess, -.025, .025))
        if self.flip_steering:
            steering = -steering
        self._junction_forward_used = True
        self._junction_encoder_start = None
        self._junction_encoder_adjustment = 0.0
        return (float(np.clip(speed-steering, self.min_active_wheel_speed, self.max_speed)),
                float(np.clip(speed+steering, self.min_active_wheel_speed, self.max_speed)), steering)

    def straight_visual_wheels(self, speed):
        """Steer from a usable multi-row corridor without global trim.

        Steering may begin from two separated, correctly ordered row pairs so
        duck2 can correct before the outgoing markings reach the bottom of the
        camera image.  Route progress still uses the stricter ``valid`` and
        ``near_support`` checks in :meth:`straight_reacquisition_good`.
        """
        geometry = self._junction_lane_geometry
        if not self.junction_geometry_steering_valid(geometry):
            return self.straight_forward_wheels(speed)
        self._junction_forward_used = False
        self._junction_forward_since = self._junction_forward_last_seen = None
        steering = self.straight_visual_steering(geometry)
        left = float(np.clip(speed - steering, 0.0, self.max_speed))
        right = float(np.clip(speed + steering, 0.0, self.max_speed))
        left = max(left, self.min_active_wheel_speed)
        right = max(right, self.min_active_wheel_speed)
        self._junction_encoder_start = None
        self._junction_encoder_adjustment = 0.0
        return left, right, steering

    @staticmethod
    def right_white_boundary_geometry(mask, image_width):
        """Trace a nearby longitudinal white stripe independently of yellow.

        Its perspective slope is not a measured heading. This observation can
        keep an encroaching right edge out of the forward path, never confirm
        an outgoing lane or select a junction turn.
        """
        height = mask.shape[0]
        pairs = []
        for fraction in (.90, .78, .66, .54, .42, .30, .18):
            row = int(round(fraction*(height-1)))
            columns = np.flatnonzero(np.count_nonzero(mask[max(0,row-2):row+3], axis=0) >= 2)
            runs = [(float(group.mean()), len(group)) for group in
                    np.split(columns, np.where(np.diff(columns)>1)[0]+1)
                    if 5 <= len(group) <= .14*image_width]
            if not pairs:
                runs = [r for r in runs if r[0] >= .35*image_width]
                if fraction < .78 or not runs:
                    continue
                value = max(runs, key=lambda r: r[1])[0]
            else:
                if not runs or pairs[-1]["row_fraction"]-fraction > .24+1e-9:
                    continue
                expected = pairs[-1]["white_x"]
                if len(pairs) > 1:
                    previous, last = pairs[-2:]
                    expected += ((fraction-last["row_fraction"])*
                                 (last["white_x"]-previous["white_x"])/
                                 (last["row_fraction"]-previous["row_fraction"]))
                value = min(runs, key=lambda r: abs(r[0]-expected))[0]
                if abs(value-expected) > .12*image_width:
                    continue
            pairs.append(dict(row_fraction=fraction, white_x=value))
        pairs.reverse()
        result = dict(usable=False, image_width=image_width, pairs=pairs)
        if len(pairs) < 5:
            return result
        rows = np.array([p["row_fraction"] for p in pairs])
        values = np.array([p["white_x"] for p in pairs])
        if rows[-1] < .78 or rows[-1]-rows[0] < .48-1e-9:
            return result
        fit = np.polyfit(rows, values, 1)
        if (np.max(np.abs(values-np.polyval(fit, rows))) > .025*image_width
                or not -.02*image_width <= values[-1]-values[0] <= .65*image_width):
            return result
        result.update(usable=True, near_x_fraction=float(np.median(values[-2:])/image_width))
        return result

    def junction_white_guard_wheels(self, left, right, steering):
        """One-sided boundary protection added after existing junction control.

        Preserve zero, intentional turns and ordinary road following. Only a
        currently observed longitudinal right stripe can request a bounded
        left correction; absence never adds motion or advances the route.
        """
        self._junction_white_guard_used = False
        self._junction_white_guard_reason = None
        now = time.monotonic()
        approach = (self.navigation_state == "following" and self._junction_approach_active
                    and self._active_turn is None)
        crossing = (self.navigation_state in ("crossing", "reacquiring")
                    and self._active_turn == "straight")
        g = self._junction_white_geometry or {}
        if not (self.junction_white_boundary_guard and self.junctions_calibrated
                and self.junction_straight_visual_approach and (approach or crossing)
                and self._sharp_corner_state in ("idle", "cooldown")
                and self._camera_valid and 0 <= now-self._last_frame_time <= self._camera_timeout
                and self.drive_enabled and not self.manual_stop and not self.red_stop_latched
                and (not self.live.enabled or (self.live.active and self.live.paused_at is None))
                and min(left, right) > 0 and g.get("usable", False)):
            return left, right, steering
        paired = self._junction_lane_geometry or {}
        if (paired.get("valid") and paired.get("near_support")
                and abs(paired["lateral_error"]) <= self.junction_straight_settle_max_lateral
                and abs(paired["heading_error"]) <= self.junction_straight_settle_max_heading):
            # A complete, aligned corridor is stronger evidence than the
            # conservative white-only envelope, especially on a narrow view.
            return left, right, steering
        width = g["image_width"]
        # A nearby right edge inside the forward corridor deserves a small
        # left correction even when no paired lane-width estimate survives.
        near = [p["white_x"] for p in g["pairs"] if p["row_fraction"] >= .66]
        if not near:
            return left, right, steering
        intrusion = max(0.0, .70-float(np.median(near))/width)
        reason = "near right white boundary enters forward corridor"
        reference = self._junction_white_width_reference
        if (reference and reference["image_width"] == width
                and 0 <= now-reference["time"] <= .8):
            prior = reference["pairs"]
            rows = [p["row_fraction"] for p in prior]
            widths = [p["white_x"]-p["yellow_x"] for p in prior]
            # Compare only actually supported depths. Never extrapolate a far
            # fragment down to the robot or use a mixed-depth centroid width.
            offsets = [(self.junction_straight_lane_target_fraction*width
                        + .5*float(np.interp(p["row_fraction"], rows, widths))-p["white_x"])/width
                       for p in g["pairs"] if p["row_fraction"] >= .66
                       and rows[0] <= p["row_fraction"] <= rows[-1]]
            if offsets:
                relative = max(0.0, float(np.median(offsets))-.025)
                if relative > intrusion:
                    intrusion = relative
                    reason = "right white boundary intrudes into recent paired corridor"
        if intrusion <= 0:
            return left, right, steering
        correction = min(.03, self.junction_straight_approach_max_steering,
                         self.max_steering, 2*self.junction_straight_lateral_gain*intrusion)
        # Work in wheel space: positive always means the right wheel faster.
        # Do not add a trim to an already adequate or stronger left correction.
        current = (right-left)/2.0
        centre = (left+right)/2.0
        correction = min(correction, centre-self.min_active_wheel_speed, self.max_speed-centre)
        if correction <= max(current, 0.0):
            return left, right, steering
        self._junction_white_guard_used = True
        self._junction_white_guard_reason = reason
        self._junction_encoder_start = None
        self._junction_encoder_adjustment = 0.0
        return centre-correction, centre+correction, correction

    @staticmethod
    def junction_geometry_steering_valid(geometry):
        """Return whether row-matched evidence is sufficient for bounded steering.

        Hand-written test doubles from before this diagnostic field was added
        remain compatible by falling back to the established ``valid`` flag.
        """
        return bool(geometry is not None and geometry.get(
            "steering_valid", geometry.get("valid", False)))

    def straight_encoder_adjustment(self):
        """Return a small correction from cumulative crossing wheel progress."""
        self._junction_encoder_adjustment = 0.0
        if (not self.junction_straight_encoder_balance
                or self._active_turn != "straight"):
            return 0.0
        now = time.monotonic()
        if (self._left_encoder_tick is None or self._right_encoder_tick is None
                or self._left_encoder_time is None or self._right_encoder_time is None
                or now - self._left_encoder_time > 0.3
                or now - self._right_encoder_time > 0.3):
            return 0.0
        if self._junction_encoder_start is None:
            self._junction_encoder_start = (
                self._left_encoder_tick, self._right_encoder_tick)
            return 0.0
        left_start, right_start = self._junction_encoder_start
        left_ticks = abs(self._left_encoder_tick - left_start)
        right_ticks = abs(self._right_encoder_tick - right_start)
        if min(left_ticks, right_ticks) < self.junction_straight_encoder_balance_min_ticks:
            return 0.0
        imbalance = float(left_ticks - right_ticks) / max(left_ticks + right_ticks, 1)
        self._junction_encoder_adjustment = float(np.clip(
            self.junction_straight_encoder_balance_gain * imbalance,
            -self.junction_straight_encoder_balance_max,
            self.junction_straight_encoder_balance_max,
        ))
        return self._junction_encoder_adjustment

    def right_alignment_encoder_wheels(self, left, right):
        """Bounded proportional assistance; never integrate or increase power."""
        now = time.monotonic()
        previous = self._right_alignment_reference
        stamps = (self._left_encoder_time, self._right_encoder_time)
        ticks = (self._left_encoder_tick, self._right_encoder_tick)
        target = (left - right) / max(left + right, 0.001)
        eligible = (self.junction_right_encoder_assist
                    and self.junction_straight_visual_approach
                    and self._active_turn == "right" and self._junction_right_visual_entry
                    and self.navigation_state == "reacquiring"
                    and min(left, right) > 0 and abs(target) > 0.03
                    and all(v is not None for v in stamps + ticks)
                    and all(0 <= now - stamp <= 0.3 for stamp in stamps)
                    and abs(stamps[0] - stamps[1]) <= 0.1
                    and self.junction_geometry_steering_valid(self._junction_lane_geometry))
        if not eligible:
            self._right_alignment_reference = None
            self._right_alignment_adjustment = 0.0
            return left, right
        if (previous is None or now - previous[0] > 0.6
                or target * previous[3] <= 0 or abs(target - previous[3]) > 0.05
                or ticks[0] < previous[1] or ticks[1] < previous[2]):
            self._right_alignment_reference = (now, *ticks, target)
            self._right_alignment_adjustment = 0.0
            return left, right
        elapsed = now - previous[0]
        if elapsed >= 0.25:
            dl, dr = ticks[0] - previous[1], ticks[1] - previous[2]
            self._right_alignment_reference = (now, *ticks, target)
            self._right_alignment_adjustment = 0.0
            # No compensation for stalled wheels or implausible counter jumps.
            if min(dl, dr) >= 4 and max(dl, dr) <= 500:
                measured = (dl - dr) / (dl + dr)
                shortfall = (abs(target) - measured * (1 if target > 0 else -1))
                self._right_alignment_adjustment = min(0.015, max(0.0, 0.10 * shortfall))
        amount = self._right_alignment_adjustment
        if target > 0:
            right = max(min(right, self.min_active_wheel_speed), right - amount)
        else:
            left = max(min(left, self.min_active_wheel_speed), left - amount)
        return left, right

    def junction_lane_geometry(self, yellow_mask, white_mask, image_width,
                               row_fractions=(0.18, 0.36, 0.54, 0.72, 0.88)):
        """Fit a lane corridor from yellow/white samples at shared image rows."""
        height = yellow_mask.shape[0]
        pairs = []
        half_band = max(3, int(round(height * 0.025)))
        for fraction in row_fractions:
            row = int(round(fraction * (height - 1)))
            start, end = max(0, row - half_band), min(height, row + half_band + 1)
            yellow_x = np.where(yellow_mask[start:end] > 0)[1]
            white_x = np.where(white_mask[start:end] > 0)[1]
            if len(yellow_x) < 8 or len(white_x) < 8:
                continue
            # Broad horizontal fragments are not longitudinal lane borders.
            if (np.percentile(yellow_x, 90) - np.percentile(yellow_x, 10)
                    > 0.16 * image_width
                    or np.percentile(white_x, 90) - np.percentile(white_x, 10)
                    > 0.16 * image_width):
                continue
            yellow = float(np.median(yellow_x))
            white = float(np.median(white_x))
            width = white - yellow
            # Perspective can spread a nearby lane across almost the full
            # image. A fixed 70% ceiling discarded exactly the near-field
            # evidence required for reacquisition.
            if not 0.15 * image_width <= width <= 0.95 * image_width:
                continue
            pairs.append({
                "row_fraction": float(fraction),
                "yellow_x": yellow,
                "white_x": white,
                "center_x": (yellow + white) / 2.0,
                "width": width,
            })
        row_span = float(pairs[-1]["row_fraction"] - pairs[0]["row_fraction"]
                         if pairs else 0.0)
        steering_valid = len(pairs) >= 2 and row_span >= 0.18 - 1e-9
        valid = (len(pairs) >= 3
                 and pairs[-1]["row_fraction"] >= 0.72
                 and row_span >= 0.36 - 1e-9)
        result = {"valid": bool(valid), "image_width": image_width,
                  "steering_valid": bool(steering_valid),
                  "pair_count": len(pairs), "pairs": pairs,
                  "near_support": bool(pairs and pairs[-1]["row_fraction"] >= 0.72)}
        if not steering_valid:
            return result
        rows = np.asarray([item["row_fraction"] for item in pairs], dtype=float)
        centres = np.asarray([item["center_x"] for item in pairs], dtype=float)
        slope, intercept = np.polyfit(rows, centres, 1)
        # Do not extrapolate a two-row distant fit down to an unseen near row:
        # dash-edge noise was magnified almost fourfold by that extrapolation.
        near_center = float(slope * rows[-1] + intercept)
        far_center = float(slope * rows[0] + intercept)
        target = self.junction_straight_lane_target_fraction * image_width
        result.update({
            "near_center_x": near_center,
            "far_center_x": far_center,
            "lateral_error": (near_center - target) / (image_width / 2.0),
            "heading_error": (near_center - far_center) / (image_width / 2.0),
            "row_span": row_span,
        })
        return result

    def straight_exit_geometry(self, yellow_mask, white_mask, width, original):
        """Find real near-lane support between the five fixed sampling bands.

        Only app-controlled straight-junction exits use this fallback. Sampling
        a dash gap or a shadow at row .72 must not hide an otherwise continuous
        corridor reaching the middle/lower ROI. Retain observed road-entry
        evidence separately from full near support; never extrapolate a distant
        pair toward us. A stable road corridor commits the app handoff once.
        Ordinary lane centroids, masks and left/right junctions are unchanged.
        """
        if original.get("valid"):
            return original
        dense = self.junction_lane_geometry(
            yellow_mask, white_mask, width,
            row_fractions=tuple(i / 100.0 for i in range(6, 95, 4)))
        pairs = dense.get("pairs", [])
        # Keep genuinely observed mid/near road evidence even when the next
        # fixed band falls in a yellow dash gap. Previously this was discarded
        # unless the stricter near-field fit passed, delaying road ownership.
        supported = bool(
            dense.get("steering_valid") and len(pairs) >= 6
            and dense.get("row_span", 0.0) >= 0.36 - 1e-9
            and pairs[-1]["row_fraction"] >= .54
            and all(b["width"] >= a["width"] - .12 * width
                    and abs(b["center_x"] - a["center_x"]) <= .12 * width
                    for a, b in zip(pairs, pairs[1:])))
        if not supported:
            return original
        near = (dense.get("row_span", 0.0) >= .54 - 1e-9
                and pairs[-1]["row_fraction"] >= 2.0 / 3.0)
        dense.update(valid=near, near_support=near, sampling="dense_straight_exit")
        return dense

    def road_forward_geometry(self, image):
        """Trace genuine stripe runs from the near road into the curve entry.

        A whole-row white median can select walls beyond the road. Continue a
        nearby border instead, limiting jumps and rejecting broad fragments.
        No stripe is extrapolated into a reported boundary pair.
        """
        h, w = image.shape[:2]
        start = int(.4*h)
        hsv = cv2.cvtColor(image[start:int(.9*h)], cv2.COLOR_BGR2HSV)
        lower = self.white_lower.copy()
        lower[2] = self.road_white_reference_value
        white = cv2.inRange(hsv, lower, self.white_upper)
        yellow = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        wh, yh, pairs = [], [], []
        for fraction in (.85, .80, .75, .70, .65, .60, .55, .50, .45, .40):
            row = int(fraction*h)-start
            def runs(mask):
                band = mask[max(0,row-3):row+4]
                columns = np.flatnonzero(np.count_nonzero(band, axis=0) >= 3)
                return [float(np.mean(group)) for group in
                        np.split(columns, np.where(np.diff(columns)>1)[0]+1)
                        if 4 <= len(group) <= .18*w]
            def prediction(history, default):
                if not history:
                    return default
                value = history[-1][1]
                if len(history) > 1:
                    value += ((fraction-history[-1][0]) *
                              (history[-1][1]-history[-2][1]) /
                              (history[-1][0]-history[-2][0]))
                return value
            candidates = [x for x in runs(white) if x >= self.white_left_cutoff*w]
            if not candidates or (not wh and fraction < .65):
                continue
            expected = prediction(wh, .9*w)
            wx = min(candidates, key=lambda x: abs(x-expected))
            if wh and (abs(wx-expected) > .15*w or wh[-1][0]-fraction > .15+1e-9):
                continue
            wh.append((fraction,wx))
            candidates = [x for x in runs(yellow) if x < self.yellow_right_cutoff*w
                          and .15*w <= wx-x <= .95*w]
            if not candidates:
                continue
            expected = prediction(yh, .2*w)
            yx = min(candidates, key=lambda x: abs(x-expected))
            if yh and abs(yx-expected) > .2*w:
                continue
            yh.append((fraction,yx))
            pairs.append(dict(row_fraction=round((fraction-.4)/.5, 3),
                              yellow_x=yx, white_x=wx, center_x=(yx+wx)/2,
                              width=wx-yx))
        pairs.reverse()
        return dict(image_width=w, pairs=pairs, source="tracked_forward_borders")

    def detect_left_road_curve(self, image):
        """Recognize curvature, rather than mistaking a lateral error for a bend.

        Trace observed yellow/white pairs at matched depths. A slanted straight
        has linear borders; require leftward bow in BOTH borders, spread over
        the near and far road. This is image geometry, not a metric radius.
        """
        h, w = image.shape[:2]
        start = int(.4*h)
        hsv = cv2.cvtColor(image[start:int(.9*h)], cv2.COLOR_BGR2HSV)
        lower = self.white_lower.copy()
        lower[2] = self.road_white_reference_value
        white = cv2.inRange(hsv, lower, self.white_upper)
        yellow = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, np.ones((1, 7), np.uint8))
        wh, yh, pairs = [], [], []
        for fraction in np.linspace(.85, .4, 19):
            row = int(fraction*h)-start
            def runs(mask):
                band = mask[max(0, row-3):row+4]
                columns = np.flatnonzero(np.count_nonzero(band, axis=0) >= 3)
                return [(float(np.mean(group)), len(group)) for group in
                        np.split(columns, np.where(np.diff(columns)>1)[0]+1)
                        if 5 <= len(group) <= .25*w]
            def predict(history):
                value = history[-1][1]
                if len(history) > 1:
                    value += ((fraction-history[-1][0]) *
                              (history[-1][1]-history[-2][1]) /
                              (history[-1][0]-history[-2][0]))
                return value
            candidates = [run for run in runs(white)
                          if wh or run[0] >= self.white_left_cutoff*w]
            if not candidates or (not wh and fraction < .65):
                continue
            # Tiny glints next to a broad near stripe must not seed the trace.
            wx = (min(candidates, key=lambda run: abs(run[0]-predict(wh)))[0]
                  if wh else max(candidates, key=lambda run: run[1])[0])
            if wh and (abs(wx-predict(wh)) > .12*w or wh[-1][0]-fraction > .1+1e-9):
                continue
            wh.append((fraction, wx))
            candidates = [run for run in runs(yellow)
                          if run[0] < self.yellow_right_cutoff*w
                          and .15*w <= wx-run[0] <= .95*w]
            if not candidates:
                continue
            yx = min(candidates, key=lambda run: abs(run[0]-(predict(yh) if yh else .2*w)))[0]
            if yh and abs(yx-predict(yh)) > .15*w:
                continue
            yh.append((fraction, yx))
            pairs.append(dict(row_fraction=round((fraction-.4)/.5, 3),
                              yellow_x=yx, white_x=wx, center_x=(yx+wx)/2))
        pairs.reverse()
        result = dict(candidate=False, reason="insufficient shared depths", pairs=pairs,
                      image_width=w)
        # Independent observed white shape supports a SHORT yellow-dash gap
        # only after both boundaries have already confirmed this same curve.
        # It is never enough to recognize a new left turn by itself.
        trace = [dict(row_fraction=round((f-.4)/.5, 3), white_x=x)
                 for f, x in reversed(wh)]
        result["white_trace"] = trace
        if len(pairs) < 6:
            return result
        y = np.array([p["row_fraction"] for p in pairs])
        span = float(np.ptp(y))
        if span < .5 or y[0] > .35 or y[-1] < .7:
            return result
        metrics = {}
        for key in ("yellow_x", "white_x", "center_x"):
            x = np.array([p[key] for p in pairs])
            fit = np.polyfit(y, x, 2)
            metrics[key] = dict(bow=float(-fit[0]*span**2/(4*w)),
                                residual=float(np.max(np.abs(np.polyval(fit, y)-x))/w),
                                forward_left=float((x[-1]-x[0])/w))
        result.update(metrics=metrics, span=span, reason="borders do not confirm left curve")
        # Compare the robot's image centre with the corridor at SHARED near
        # depths. A centroid mixes near white with far yellow and can demand
        # a right turn even while both traced borders still bend left.
        near = [(p["center_x"]-.5*w)/(p["white_x"]-p["yellow_x"])
                for p in pairs if p["row_fraction"] >= .7
                and p["white_x"] > p["yellow_x"]]
        result["near_lateral_fraction"] = float(np.median(near)) if len(near) >= 2 else None
        yellow, white, center = (metrics[k] for k in ("yellow_x", "white_x", "center_x"))
        result["candidate"] = (yellow["bow"] >= .015 and white["bow"] >= .008
                               and yellow["residual"] <= .025 and white["residual"] <= .025
                               and center["forward_left"] >= .10)
        if result["candidate"]:
            result["reason"] = "both borders curve left"
        return result

    def left_road_yellow_gap_allowed(self, now):
        """Continue only recently confirmed curvature with a coherent white edge.

        The 0.8-second absolute limit is measured from the last complete curve
        observation. White-only frames cannot renew it. Stop/pause/fault resets
        the reference; straight white, a jumping edge and absent images fail.
        """
        shape = self._road_left_shape or {}
        last = getattr(self, "_road_left_boost_last_seen", None)
        if not (getattr(self, "road_left_curve_boost", False)
                and last is not None and 0 <= now-last <= .8
                and self.navigation_state == "following" and self._active_turn is None
                and self._junction_phase in ("idle", "complete", "route_complete")
                and not self._junction_approach_active and not self._red_line_visible
                and self._sharp_corner_state in ("idle", "cooldown")
                and self._white_boundary_visible and not self._yellow_boundary_visible
                and not self.manual_stop and not self.red_stop_latched
                and self.drive_enabled and self._camera_valid
                and 0 <= now-self._last_frame_time <= self._camera_timeout
                and (not self.live.enabled or (self.live.active and self.live.paused_at is None))):
            return False
        previous = self._road_left_boost_white_trace or []
        current = shape.get("white_trace", [])
        matches = [(p, q) for p in current for q in previous
                   if abs(p["row_fraction"]-q["row_fraction"]) < .001]
        # Compare observations at identical depths, never a centroid at another
        # image height. Large changes may be another edge or a relocated robot.
        width = shape.get("image_width", 0)
        if not (width > 0 and len(matches) >= 6
                and all(abs(p["white_x"]-q["white_x"]) <= .12*width
                        for p, q in matches)):
            return False
        rows = np.array([p["row_fraction"] for p, _ in matches])
        values = np.array([p["white_x"] for p, _ in matches])
        # Retain the recent per-row width through this short gap, not the
        # mixed-depth centroid width. A large move left of the corridor still
        # releases the left turn; white-only observations never renew memory.
        near = [(q["center_x"]+p["white_x"]-q["white_x"]-.5*width)
                / (q["white_x"]-q["yellow_x"])
                for p, q in matches if p["row_fraction"] >= .7
                and q["white_x"] > q["yellow_x"]]
        if len(near) < 2 or float(np.median(near)) > .10:
            return False
        span = float(np.ptp(rows))
        if span < .5 or rows[0] > .35 or rows[-1] < .7:
            return False
        # Judge curvature over the SAME depths that the complete corridor
        # supported, not newly visible far pixels with different perspective.
        fit = np.polyfit(rows, values, 2)
        return bool(-fit[0]*span**2/(4*width) >= .008
                    and np.max(np.abs(np.polyval(fit, rows)-values))/width <= .025
                    and (values[-1]-values[0])/width >= .10)

    def left_road_curve_boost_active(self, lane_error, now):
        """Only a fresh, sustained road-curve observation authorizes .03/.20."""
        evidence = self._road_left_shape or {}
        gap = self.left_road_yellow_gap_allowed(now)
        near = evidence.get("near_lateral_fraction")
        paired_curve = (evidence.get("candidate", False) and self._lane_both_visible
                        and near is not None and math.isfinite(near) and near <= .10)
        eligible = (self.road_left_curve_boost
                    and (paired_curve or gap)
                    and self.navigation_state == "following" and self._active_turn is None
                    and self._junction_phase in ("idle", "complete", "route_complete")
                    and not self._junction_approach_active and not self._red_line_visible
                    and self._sharp_corner_state in ("idle", "cooldown")
                    and self._camera_valid
                    and 0 <= now-self._last_frame_time <= self._camera_timeout
                    and not self.manual_stop and not self.red_stop_latched
                    and self.drive_enabled and self.speed_scale > 0
                    and (not self.live.enabled or (self.live.active and self.live.paused_at is None))
                    and lane_error is not None and math.isfinite(lane_error))
        if not eligible:
            self._road_left_boost_since = None
            self._road_left_boost_active = False
            self._road_left_boost_last_seen = None
            self._road_left_boost_white_trace = None
        else:
            if self._road_left_boost_since is None:
                self._road_left_boost_since = now
            self._road_left_boost_active = now-self._road_left_boost_since >= .3
            if self._road_left_boost_active and not gap:
                self._road_left_boost_last_seen = now
                self._road_left_boost_white_trace = evidence.get("pairs")
        self._road_left_boost_gap_active = bool(gap and self._road_left_boost_active)
        return self._road_left_boost_active

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
        self._junction_lane_geometry = self.junction_lane_geometry(
            yellow_mask, white_mask, w)
        if self.initial_straight_approach_active():
            # Find existing dashes between fixed bands without inventing
            # unseen near borders. This fallback is confined to the start leg.
            self._junction_lane_geometry = self.straight_exit_geometry(
                yellow_mask, white_mask, w, self._junction_lane_geometry)
        self._junction_white_geometry = None
        if getattr(self, "junction_white_boundary_guard", False):
            self._junction_white_geometry = self.right_white_boundary_geometry(white_mask, w)
            geometry = self._junction_lane_geometry
            if geometry.get("valid"):
                pairs = geometry["pairs"]
                widths = [p["white_x"]-p["yellow_x"] for p in pairs]
                if all(b >= a-.04*w for a, b in zip(widths, widths[1:])):
                    self._junction_white_width_reference = dict(
                        time=time.monotonic(), image_width=w, pairs=copy.deepcopy(pairs))
        # Extra samples confirm the exit only. They must never switch the
        # calibrated visual steering between differently fitted image depths.
        self._junction_exit_geometry = None
        self._junction_forward_geometry = None
        if (getattr(self, "navigation_state", None) in ("crossing", "reacquiring")
                and self._active_turn == "straight"
                and (self.require_client_heartbeat or self.live.enabled)):
            self._junction_exit_geometry = self.straight_exit_geometry(
                yellow_mask, white_mask, w, self._junction_lane_geometry)
            self._junction_forward_geometry = self.straight_forward_geometry(img_bgr)

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
        self._road_white_reference_used = False
        # Preserve the broad mask for junction geometry, visibility and
        # one-border fallback. Prefer the historical bright reference only
        # when a complete lane is visible and enough bright white survives.
        # Do not change the calibrated junction/corner handoff detector.
        straight_road_tracking = (
            getattr(self, "_junction_straight_visual_entry", False)
            and getattr(self, "_active_turn", None) == "straight"
            and getattr(self, "navigation_state", None) == "reacquiring")
        ordinary_road = (not getattr(self, "_red_line_visible", False)
                        and not getattr(self, "_junction_approach_active", False)
                        and (straight_road_tracking or (
                            getattr(self, "navigation_state", "following") in
                            ("following", "awaiting_route", "route_complete")
                            and getattr(self, "_junction_phase", "idle") in
                            ("idle", "complete", "route_complete")))
                        and getattr(self, "_sharp_corner_state", "idle") in ("idle", "cooldown"))
        self._road_left_shape = (self.detect_left_road_curve(img_bgr)
                                 if ordinary_road and getattr(self, "road_left_curve_boost", False) else None)
        self._yellow_boundary_visible = yellow_x is not None
        self._white_boundary_visible = white_x is not None
        confirmed_curve_gap = (ordinary_road and getattr(self, "road_left_curve_boost", False)
                               and self.left_road_yellow_gap_allowed(time.monotonic()))
        road_white_mask = white_mask
        if (ordinary_road and (yellow_x is not None or confirmed_curve_gap) and white_x is not None
                and self.road_white_reference_value > self.white_lower[2]):
            reference_lower = self.white_lower.copy()
            reference_lower[2] = self.road_white_reference_value
            reference = cv2.inRange(hsv, reference_lower, self.white_upper)
            reference[:, :int(self.white_left_cutoff * w)] = 0
            reference = cv2.morphologyEx(reference, cv2.MORPH_OPEN, kernel)
            reference = cv2.morphologyEx(reference, cv2.MORPH_CLOSE, kernel)
            reference_x, reference_y = weighted_center_x(reference, min_pixels=100)
            if reference_x is not None and (yellow_x is None or reference_x - yellow_x >= 0.15 * w):
                white_x, white_y = reference_x, reference_y
                road_white_mask = reference
                self._road_white_reference_used = True
        self._road_curve_geometry = None
        if ordinary_road and getattr(self, "road_left_lookahead", False):
            self._road_curve_geometry = self.junction_lane_geometry(
                yellow_mask, road_white_mask, w,
                row_fractions=(0.04, 0.12, 0.20, 0.40, 0.55, 0.70, 0.85))
            forward = self.road_forward_geometry(img_bgr)
            if (sum(p["row_fraction"] <= .10 for p in forward["pairs"]) >= 2
                    and sum(p["row_fraction"] >= .40 for p in forward["pairs"]) >= 2):
                self._road_curve_geometry = forward
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
        for pair in self._junction_lane_geometry["pairs"]:
            y = roi_y0 + int(pair["row_fraction"] * max(roi_y1 - roi_y0 - 1, 1))
            cv2.circle(debug_image, (int(pair["yellow_x"]), y), 3, (0, 200, 200), -1)
            cv2.circle(debug_image, (int(pair["white_x"]), y), 3, (200, 200, 200), -1)
            cv2.line(debug_image, (int(pair["yellow_x"]), y),
                     (int(pair["white_x"]), y), (160, 80, 0), 1)
        self._lane_diagnostic = "Both boundaries"
        self._road_left_gap_active = False
        self._road_left_boost_gap_active = False


        if yellow_x is not None and white_x is not None:
            # A reversed or implausibly narrow pair is not an outgoing right lane.
            if white_x - yellow_x < 0.15 * w:
                self._lane_half_width_px = None
                self._lane_half_width_time = None
                self._lane_diagnostic = "Lane lost: reversed or narrow boundaries"
                return None, debug_image, debug_mask
            self._lane_both_visible = True
            self._left_curve_white_x = white_x
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
            # Only continue a recently confirmed left curve. A generic missing
            # yellow line is not a turn instruction. The absolute expiry is
            # never refreshed by white-only frames, and a jumping white blob
            # cannot carry the old turn forward.
            curve_gap = (ordinary_road and getattr(self, "road_left_lookahead", False)
                         and not self.manual_stop
                         and self._left_curve_confirmed_at is not None
                         and 0 <= time.monotonic()-self._left_curve_confirmed_at <= .8
                         and self._left_curve_steering is not None
                         and self._left_curve_white_x is not None
                         and abs(white_x-self._left_curve_white_x) <= .12*w)
            boost_gap = (ordinary_road and getattr(self, "road_left_curve_boost", False)
                         and self.left_road_yellow_gap_allowed(time.monotonic()))
            white_timeout = (max(self.temporal_lane_width_timeout, .8)
                             if curve_gap or boost_gap else self.temporal_lane_width_timeout)
            use_temporal = (self.temporal_lane_width_fallback
                            and self._lane_half_width_px is not None
                            and self._lane_half_width_time is not None
                            and time.monotonic() - self._lane_half_width_time
                            <= white_timeout)
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
            if curve_gap and use_temporal:
                self._road_left_gap_active = True
                self._left_curve_white_x = white_x
                self._lane_diagnostic = "White-only confirmed left-curve continuation"
            if boost_gap and use_temporal:
                self._road_left_boost_gap_active = True
                self._lane_diagnostic = "White-only confirmed curve: bounded yellow gap"
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

    def live_straight_evidence(self):
        """Conservative image-space classification; never changes turn control."""
        g = self._junction_lane_geometry or {}
        pairs = g.get("pairs", [])
        if (self.navigation_state != "following" or self._active_turn is not None
                or self._sharp_corner_state not in ("idle", "cooldown")
                or self._junction_phase == "settling" or self._junction_approach_active
                or self._red_line_visible or not self._lane_both_visible
                or not g.get("valid") or not g.get("near_support")
                or len(pairs) < 3 or g.get("row_span", 0) < 0.54 - 1e-9
                or abs(g.get("heading_error", 1)) > 0.035
                or abs(g.get("lateral_error", 1)) > 0.12
                or abs(self.prev_steering) > 0.025
                or self._last_lane_error is None or abs(self._last_lane_error) > 0.08):
            return False
        width = g.get("image_width", 0)
        centres = [p["center_x"] for p in pairs]
        return width > 0 and (max(centres)-min(centres))/width <= 0.025

    def live_tick(self, now=None, observe=False):
        if not self.live.enabled:
            return
        now = time.monotonic() if now is None else now
        paused_time = self.live.account_pause(now)
        if paused_time:
            # Pausing must not consume a turn's travel time or complete its
            # timed entry while stationary. Sensor ages and the independent
            # watchdog stay on real time. Active-motion limits are unchanged.
            for name in ("_stop_started", "_crossing_started", "_reacquire_started",
                         "_junction_deadline_at", "_junction_right_advance_started",
                         "_sharp_corner_state_since", "_sharp_corner_cooldown_until"):
                value = getattr(self, name)
                if value is not None:
                    setattr(self, name, value + paused_time)
        healthy = (self._camera_valid and now-self._last_frame_time <= self._camera_timeout
                   and not self.client_expired() and not self._client_connection_lost
                   and not self._fault_reason and not self._stopping
                   and (not self.require_client_heartbeat or self._active_client_id is not None))
        if observe or not healthy or not self.live_straight_evidence():
            self.live.observe_straight(now, healthy and self.live_straight_evidence())
        approach = ("%s->%s" % tuple(self.route[self.route_index-1:self.route_index+1])
                    if self.route_enabled and 1 <= self.route_index < len(self.route) else None)
        self.live.tick(now, self.navigation_state, self.route_index, self._stop_started, healthy, approach)
        if not self.live.active:
            self.manual_stop = True
            if self.navigation_state != "fault":
                self.navigation_state = "route_complete"

    def live_command(self, action, command):
        if (not self.live.enabled or not self.live.active
                or command.get("run_id") != self.live.run_id):
            raise ValueError("The live run ended or changed; confirm placement and Start again")
        if command.get("expected_control_epoch") != self._control_epoch:
            raise ValueError("Control changed; read fresh status before sending this command")
        self.live_tick()
        if not self.live.active:
            raise ValueError(self.live.end_reason)
        if action == "pause":
            self.live.request_pause(command.get("seconds"), time.monotonic())
            self._control_epoch += 1
            self.publish_wheels(0.0, 0.0)
            # Require new continuity evidence after resuming; a still camera
            # must not complete an outgoing-lane or corner recognition window.
            for name in ("_lane_good_since", "_red_clear_since",
                         "_junction_straight_corridor_since",
                         "_junction_right_absent_since", "_junction_right_corridor_since",
                         "_junction_right_near_since", "_junction_left_corridor_since",
                         "_junction_left_near_since", "_junction_settle_good_since",
                         "_sharp_corner_candidate_since", "_sharp_corner_reacquire_since",
                         "_sharp_corner_white_since"):
                setattr(self, name, None)
            self._junction_encoder_start = None
            self._junction_encoder_adjustment = 0.0
            self._right_alignment_reference = None
        elif action == "resume":
            if (not self._camera_valid or time.monotonic()-self._last_frame_time > self._camera_timeout
                    or self.client_expired() or self._client_connection_lost or self._fault_reason):
                raise ValueError("Cannot resume without fresh camera and connection")
            self.live.resume()
        elif action == "straight_profile":
            profile = command.get("value")
            if profile not in PROFILES:
                raise ValueError("Choose slow, normal or fast")
            if (PROFILES[profile] > PROFILES[self.live.profile]
                    and not (self.live.straight and self.live_straight_evidence())):
                raise ValueError("Speed increases are available only on a confirmed straight section")
            self.live.profile = profile
        elif action == "junction_instruction":
            if self.live.stop_after_junction and command.get("value") != "straight":
                raise ValueError("This check permits one straight crossing only")
            if (self.navigation_state != "red_stop" or self.manual_stop
                    or self.live.paused_at is not None
                    or command.get("expected_route_index") != self.route_index):
                raise ValueError("Junction instruction needs the current red stop")
            previous, junction = self.route[self.route_index-1:self.route_index+1]
            if command.get("approach") != "%s->%s" % (previous, junction):
                raise ValueError("Starting approach changed")
            turn = command.get("value")
            exits = [n for n in MAP_PORTS[junction].values()
                     if n != previous and junction_turn(previous, junction, n) == turn]
            if len(exits) != 1:
                raise ValueError("That exit is unavailable on the map")
            if self._stop_started is None or time.monotonic()-self._stop_started < self.stop_hold_seconds:
                raise ValueError("The red-line dwell is not finished")
            blocker = self.junction_departure_blocker()
            if blocker:
                raise ValueError(blocker)
            original = self.route
            self.route = self.route[:self.route_index+1] + exits
            try:
                self.begin_crossing()
            except (ValueError, TypeError, KeyError):
                self.route = original
                raise
            self.live.instruction_id = command["id"]
            self.live.red_wait_at = self.live.red_index = None

    def guard_road_heading(self, raw_steering):
        """Limit an excessive centroid correction near a supported lane centre.

        Whole-mask centroids sample yellow dashes and white tape at different
        depths. A near-centred, aligned multi-row corridor can disprove their
        requested magnitude without retuning established curve authority.
        Values here are image errors, not measured physical angles.
        """
        geometry = self._junction_lane_geometry
        eligible = (self.road_heading_guard and self.navigation_state == "following"
                    and self._junction_phase in ("idle", "complete", "route_complete")
                    and not self._junction_approach_active and not self._red_line_visible
                    and self._sharp_corner_state in ("idle", "cooldown")
                    and self._lane_both_visible and geometry is not None
                    and geometry.get("valid") and geometry.get("near_support")
                    and abs(geometry["lateral_error"]) <= 0.06
                    and abs(geometry["heading_error"]) <= 0.08)
        if eligible:
            reference = (self.junction_straight_lateral_gain * geometry["lateral_error"]
                         - self.junction_straight_heading_gain * geometry["heading_error"])
            if raw_steering * reference < 0 or abs(raw_steering) > abs(reference):
                self._road_heading_guard_used = True
                return reference
        return raw_steering

    def anticipate_left_bend(self, raw_steering):
        """Use visible far/near border pairs before the near centroid turns left.

        This opt-in ordinary-road correction never invents a missing border or
        changes a junction/pivot maneuver. Shared rows avoid pairing a near
        yellow dash with distant white tape. Image heading is not a yaw angle.
        """
        g = self._road_curve_geometry
        if not (self.road_left_lookahead and self.navigation_state == "following"
                and self._junction_phase in ("idle", "complete", "route_complete")
                and not self._junction_approach_active and not self._red_line_visible
                and self._sharp_corner_state in ("idle", "cooldown")
                and self._lane_both_visible and g is not None):
            return raw_steering
        pairs = g.get("pairs", [])
        far = [p for p in pairs if p["row_fraction"] <= .20][:2]
        near = [p for p in pairs if p["row_fraction"] >= .40]
        if len(far) < 2 or len(near) < 2:
            return raw_steering
        # Reject an inverted perspective corridor and large left-of-lane
        # offsets, for which ordinary centering must retain right authority.
        if np.median([p["width"] for p in far]) >= np.median([p["width"] for p in near]):
            return raw_steering
        scale = g["image_width"] / 2.0
        far_center = float(np.median([p["center_x"] for p in far]))
        near_center = float(np.median([p["center_x"] for p in near]))
        lateral = (near_center - self.junction_straight_lane_target_fraction * g["image_width"]) / scale
        heading = (near_center - far_center) / scale
        if heading <= .03 or lateral > .10:
            return raw_steering
        ratio = min(1.0, (heading - .03) / .05)
        blend = ratio * ratio * (3.0 - 2.0 * ratio)
        reference = -self.k_p * heading * blend
        if reference < raw_steering:
            self._road_left_lookahead_used = True
            return reference
        return raw_steering

    def compute_wheel_speeds(self, lane_error):
        self._junction_forward_used = False
        now = time.monotonic()
        self._road_heading_guard_used = False
        self._road_left_lookahead_used = False
        curve_boost = self.left_road_curve_boost_active(lane_error, now)
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
            raw_steering = self.guard_road_heading(raw_steering)
            raw_steering = self.anticipate_left_bend(raw_steering)
            if (self._road_left_gap_active and self._left_curve_confirmed_at is not None
                    and 0 <= now-self._left_curve_confirmed_at <= .8):
                raw_steering = min(raw_steering, self._left_curve_steering)
            elif (self._lane_both_visible
                  and (self._road_left_lookahead_used
                       or (self._left_curve_confirmed_at is not None
                           and 0 <= now-self._left_curve_confirmed_at <= .8))
                  and self.navigation_state == "following"
                  and self._junction_phase in ("idle", "complete", "route_complete")
                  and not self._junction_approach_active and not self._red_line_visible
                  and self._sharp_corner_state in ("idle", "cooldown")
                  and raw_steering <= -.03 and not self.manual_stop):
                if self._left_curve_since is None:
                    self._left_curve_since = now
                if now-self._left_curve_since >= .2:
                    self._left_curve_confirmed_at = now
                    self._left_curve_steering = max(-self.max_steering, raw_steering)
            else:
                self._left_curve_since = self._left_curve_confirmed_at = None
                self._left_curve_steering = None
            if curve_boost:
                # .115 +/- .085 gives left .03 / right .20 after the existing
                # steering and output slew limits. Straight/junction tuning is
                # untouched; loss of current curve evidence removes this target.
                base_speed = .115
                raw_steering = -.085 if self.flip_steering else .085
            raw_steering = float(np.clip(raw_steering, -self.max_steering, self.max_steering))

            steering = float(np.clip(
                raw_steering,
                self.prev_steering - steering_change,
                self.prev_steering + steering_change,
            ))

            self.prev_steering = steering

        if self.flip_steering:
            steering = -steering

        straight_profile = (not curve_boost and self.live.enabled and self.live.active and self.live.straight
                            and self.live_straight_evidence() and abs(steering) <= 0.025)
        if straight_profile:
            base_speed = min(PROFILES[self.live.profile], STRAIGHT_WHEEL_CAP-abs(steering))

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
                    or (self.live.enabled and (not self.live.active or self.live.paused_at is not None))
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
            if (((self._active_turn == "right" and self._junction_right_visual_entry)
                 or (self._active_turn == "left" and self.junction_left_visual_latch
                     and self._junction_left_visual_entry))
                    and self.junction_straight_visual_approach
                    and self.navigation_state == "reacquiring"
                    and left_speed > 0 and right_speed > 0):
                # Independent ramps retained the turn's outside-wheel power
                # while its inside wheel restarted, adding an unintended turn.
                # A common scale preserves the requested visual steering ratio;
                # neither wheel can increase faster than the existing limit.
                scale = min(1.0,
                            (self._last_wheel_speeds[0] + step) / left_speed,
                            (self._last_wheel_speeds[1] + step) / right_speed)
                left_speed *= scale
                right_speed *= scale
            else:
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
        if self.live.enabled and self.live.end_reason:
            return self.live.end_reason
        if self.live.paused_at is not None:
            return "Paused by chat"
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

    def junction_reacquisition_blocker(self):
        """Explain the existing acceptance gates without changing navigation."""
        if self.navigation_state not in ("crossing", "reacquiring"):
            return None
        if (self._active_turn == "right" and self.junction_straight_visual_approach
                and self._junction_right_turn_origin is None):
            return "Waiting for white-boundary end and short forward clearance"
        duration, _, _ = self.junction_profile()
        if self._crossing_progress < self.junction_entry_seconds + duration:
            return "Minimum maneuver progress not reached"
        if not self._departed_red:
            return "Departure red line has not cleared"
        if self._active_turn in ("straight", "left", "right") and self.junction_straight_visual_approach:
            geometry = self.junction_exit_geometry()
            if geometry is None or not geometry.get("valid") or not geometry.get("near_support"):
                return "Waiting for a near-field multi-row outgoing corridor"
            if self._active_turn == "straight" and self._junction_straight_visual_entry:
                return "Road following active; confirming near-field outgoing lane"
            if self.straight_road_ready(geometry):
                return "Waiting for stable outgoing lane ready for road following"
            if abs(geometry["lateral_error"]) >= self.junction_straight_reacquire_max_lateral:
                return "Outgoing lane lateral alignment is outside the limit"
            if abs(geometry["heading_error"]) >= self.junction_straight_reacquire_max_heading:
                return "Outgoing lane heading is outside the limit"
            return "Waiting for stable outgoing position and heading"
        if not self._lane_both_visible or self._last_lane_error is None:
            return "Waiting for ordered yellow-left and white-right boundaries"
        if abs(self._last_lane_error) >= self.junction_reacquire_max_error:
            return "Outgoing lane needs alignment"
        return "Waiting for 0.3 seconds of stable outgoing lane"

    def junction_departure_blocker(self):
        """Hold a straight departure when the last trustworthy heading is unsafe."""
        if (not self.junction_straight_visual_approach
                or self.upcoming_junction_turn() != "straight"):
            return None
        geometry = self._junction_stop_geometry
        if geometry is not None and geometry.get("valid") and abs(
                geometry["heading_error"]) > self.junction_straight_departure_max_heading:
            return "Straight departure blocked: approach heading is outside the limit"
        return None

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
            "junction_departed_red": self._departed_red,
            "junction_red_reappeared": self._junction_red_reappeared,
            "junction_right_entry_stage": (
                "pivot_started" if self._junction_right_turn_origin is not None else
                "forward_clearance" if self._junction_right_advance_started is not None else
                "confirming_white_loss" if self._junction_right_absent_since is not None else
                "waiting_for_white_end" if self._junction_right_white_seen else
                "waiting_for_initial_white"),
            "junction_reacquisition_blocker": self.junction_reacquisition_blocker(),
            "junction_search_remaining_seconds": (
                max(0.0, self.junction_reacquire_timeout - (now - self._reacquire_started))
                if self.navigation_state == "reacquiring" else None),
            "junction_last_result": self._last_junction_result,
            "junction_encoder_adjustment": self._junction_encoder_adjustment,
            "junction_lane_geometry": self._junction_lane_geometry,
            "junction_exit_geometry": self._junction_exit_geometry,
            "junction_straight_visual_entry": self._junction_straight_visual_entry,
            "junction_white_boundary_guard": self.junction_white_boundary_guard,
            "junction_white_guard_used": self._junction_white_guard_used,
            "junction_white_guard_reason": self._junction_white_guard_reason,
            "junction_white_geometry": self._junction_white_geometry,
            "junction_right_visual_entry": self._junction_right_visual_entry,
            "junction_right_tracking_trim": self.junction_right_tracking_trim,
            "junction_right_red_rearmed": self._junction_right_red_rearmed,
            "junction_right_encoder_assist": self.junction_right_encoder_assist,
            "junction_right_encoder_adjustment": self._right_alignment_adjustment,
            "junction_stop_geometry": self._junction_stop_geometry,
            "junction_departure_blocker": self.junction_departure_blocker(),
            "junction_approach_active": self._junction_approach_active,
            "junction_settled": self._junction_settled,
            "junction_settings": {
                "right_entry_policy": "white_end_then_short_clearance",
                "right_clearance_seconds": RIGHT_JUNCTION_CLEARANCE_SECONDS,
                "entry_seconds": self.junction_entry_seconds,
                "straight_seconds": self.junction_straight_seconds,
                "left_seconds": self.junction_left_seconds,
                "right_seconds": self.junction_right_seconds,
                "reacquire_timeout": self.junction_reacquire_timeout,
                "reacquire_max_error": self.junction_reacquire_max_error,
                "straight_speed": self.junction_straight_speed,
                "straight_approach_max_steering": (
                    self.junction_straight_approach_max_steering),
                "straight_visual_approach": self.junction_straight_visual_approach,
                "straight_lane_target_fraction": (
                    self.junction_straight_lane_target_fraction),
                "straight_lateral_gain": self.junction_straight_lateral_gain,
                "straight_heading_gain": self.junction_straight_heading_gain,
                "straight_departure_max_heading": (
                    self.junction_straight_departure_max_heading),
                "straight_reacquire_max_lateral": (
                    self.junction_straight_reacquire_max_lateral),
                "straight_reacquire_max_heading": (
                    self.junction_straight_reacquire_max_heading),
                "straight_reacquire_seconds": self.junction_straight_reacquire_seconds,
                "straight_settle_max_lateral": (
                    self.junction_straight_settle_max_lateral),
                "straight_settle_max_heading": self.junction_straight_settle_max_heading,
                "straight_settle_max_steering": (
                    self.junction_straight_settle_max_steering),
                "straight_settle_seconds": self.junction_straight_settle_seconds,
                "straight_encoder_balance": self.junction_straight_encoder_balance,
                "straight_encoder_balance_gain": (
                    self.junction_straight_encoder_balance_gain),
                "straight_encoder_balance_max": (
                    self.junction_straight_encoder_balance_max),
                "straight_encoder_balance_min_ticks": (
                    self.junction_straight_encoder_balance_min_ticks),
                "left_speed": self.junction_left_speed,
                "right_speed": self.junction_right_speed,
                "left_bias": self.junction_left_bias,
                "right_bias": self.junction_right_bias,
            },
            "fault": self._fault_reason, "last_command": self._last_command,
            "control_epoch": self._control_epoch,
            "live_session": self.live.snapshot(time.monotonic()),
            "current_approach": "%s->%s" % tuple(self.route[self.route_index-1:self.route_index+1]),
            "junction_instruction_ready": (self.live.enabled and self.live.active
                and self.navigation_state == "red_stop" and not self.manual_stop
                and self.live.paused_at is None and self._stop_started is not None
                and time.monotonic()-self._stop_started >= self.stop_hold_seconds
                and self.junction_departure_blocker() is None),
            "active_client": self._active_client_id,
            "client_connection_lost": self._client_connection_lost,
            "require_client_heartbeat": self.require_client_heartbeat,
            "obstacle_stop": self.obstacle_stop_latched,
            "obstacle_visible": self._obstacle_visible,
            "obstacle_box": self._obstacle_box,
            "opencv_threads": cv2.getNumThreads(),
            "camera_processing_seconds": self._camera_processing_seconds,
            "last_camera_timeout": self._last_camera_timeout,
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
            "road_white_reference_value": self.road_white_reference_value,
            "road_white_reference_used": self._road_white_reference_used,
            "road_heading_guard": self.road_heading_guard,
            "road_heading_guard_used": self._road_heading_guard_used,
            "junction_forward_geometry": self._junction_forward_geometry,
            "junction_forward_used": self._junction_forward_used,
            "road_left_curve_boost": self.road_left_curve_boost,
            "road_left_curve_boost_active": self._road_left_boost_active,
            "road_left_curve_yellow_gap_active": self._road_left_boost_gap_active,
            "road_left_curve_confirmation_age": (
                max(0.0, now-self._road_left_boost_last_seen)
                if self._road_left_boost_last_seen is not None else None),
            "road_left_curve_shape": self._road_left_shape,
            "road_left_lookahead": self.road_left_lookahead,
            "road_left_lookahead_used": self._road_left_lookahead_used,
            "road_left_gap_active": self._road_left_gap_active,
            "road_curve_geometry": self._road_curve_geometry,
            "wheel_evidence": {
                "executed": self._executed_wheels,
                "executed_age": (None if self._executed_wheels_time is None
                                 else now-self._executed_wheels_time),
                "executed_samples": self._executed_samples,
                "executed_zero_samples": self._executed_zero_samples,
                "left_ticks": self._left_encoder_tick,
                "right_ticks": self._right_encoder_tick,
                "left_age": (None if self._left_encoder_time is None else now-self._left_encoder_time),
                "right_age": (None if self._right_encoder_time is None else now-self._right_encoder_time),
            },
            "junction_left_visual_latch": self.junction_left_visual_latch,
            "junction_left_visual_entry": self._junction_left_visual_entry,
            "junction_left_red_rearmed": self._junction_left_red_rearmed,
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
        if self.live.enabled:
            self.live.end(str(reason))
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
            self.live_tick()
            if self.live.enabled and (not self.live.active or self.live.paused_at is not None):
                self.publish_wheels(0.0, 0.0)
            if time.monotonic() - self._last_frame_time > self._camera_timeout:
                if self._camera_valid:
                    now = time.monotonic()
                    self._last_camera_timeout = {
                        "at": now, "frame_age": now-self._last_frame_time,
                        "receive_age": (None if self._last_camera_received_at is None
                                        else now-self._last_camera_received_at),
                        "processing_elapsed": (None if self._camera_processing_started_at is None
                                               else now-self._camera_processing_started_at),
                        "previous_processing_seconds": self._camera_processing_seconds,
                    }
                    rospy.logwarn("camera_timeout_details=%s", self._last_camera_timeout)
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
                    if self.live.enabled:
                        self.live.end("Run ended by Stop")
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
                elif action in ("pause", "resume", "straight_profile", "junction_instruction"):
                    self.live_command(action, command)
                elif action in ("slow_down", "speed_up", "speed_scale"):
                    if self.live.enabled:
                        raise ValueError("Live chat uses only the three straight-road speed profiles")
                    if action == "speed_scale":
                        scale = command.get("value")
                    else:
                        scale = self.speed_scale * (0.8 if action == "slow_down" else 1.2)
                    if (isinstance(scale, bool) or not isinstance(scale, (int, float))
                            or not math.isfinite(scale) or not 0.25 <= scale <= 1.5):
                        raise ValueError("Speed scale must be between 0.25 and 1.5")
                    self.speed_scale = float(scale)
                elif action == "set_route":
                    if self.live.enabled and self.live.active and not self.manual_stop:
                        raise ValueError("End the active live run with Stop before confirming a new starting lane")
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
                    managed = command.get("managed_session", False)
                    if not isinstance(managed, bool):
                        raise ValueError("managed_session must be boolean")
                    new_live = LiveSession()
                    if managed:
                        new_live.start(command.get("run_id"), command.get("stop_at_next_red", False),
                                       command.get("finish_approach"),
                                       command.get("stop_after_junction", False),
                                       command.get("finish_after_junction_red", False),
                                       command.get("center_initial_straight", False))
                    self.live = new_live
                    self.speed_scale = 1.0
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
                    self._junction_approach_geometry = None
                    self._junction_approach_geometry_time = None
                    self._junction_approach_active = False
                    self._junction_stop_geometry = None
                    self._junction_stop_white_seen = False
                    self._junction_approach_steering = 0.0
                    self._junction_settle_good_since = None
                    self._junction_settled = False
                    self._junction_exit_geometry = None
                    self._junction_straight_visual_entry = False
                    self._junction_straight_corridor_since = None
                    self._stop_started = self._crossing_started = self._reacquire_started = None
                    self._crossing_updated = None
                    self._crossing_progress = 0.0
                    self._lane_good_since = self._red_clear_since = None
                    self._departed_red = False
                    self._junction_encoder_start = None
                    self._junction_encoder_adjustment = 0.0
                    self._right_alignment_reference = None
                    self._junction_lane_geometry = None
                    self._lane_half_width_px = self._lane_half_width_time = None
                    self.reset_steering()
                    self.reset_sharp_corner()
                elif action == "turn":
                    if self.live.enabled:
                        raise ValueError("Edit the laptop turn queue; turns are delivered at red stops")
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
                    if self.live.enabled and (not self.live.active or self.red_stop_latched
                                              or self.live.paused_at is not None):
                        raise ValueError("Use resume or a validated red-stop instruction in live mode")
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
        blocker = self.junction_departure_blocker()
        if blocker is not None:
            self._junction_phase = "departure_blocked"
            raise ValueError(blocker)
        self._active_turn = junction_turn(*self.route[self.route_index-1:self.route_index+2])
        self._junction_straight_visual_entry = False
        self._junction_straight_corridor_since = None
        self._junction_exit_geometry = None
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
        self._junction_red_reappeared = False
        self._junction_right_white_seen = bool(
            self._white_boundary_visible or self._junction_stop_white_seen)
        self._junction_stop_white_seen = False
        self._right_alignment_reference = None
        self._right_alignment_adjustment = 0.0
        self._junction_right_near_since = None
        self._junction_right_red_rearmed = False
        self._junction_right_visual_entry = False
        self._junction_right_corridor_since = None
        self._junction_right_absent_since = None
        self._junction_right_advance_started = None
        self._junction_right_turn_origin = None
        self._junction_encoder_start = (
            (self._left_encoder_tick, self._right_encoder_tick)
            if self._left_encoder_tick is not None
            and self._right_encoder_tick is not None else None)
        self._junction_encoder_adjustment = 0.0
        self._junction_settle_good_since = None
        self._junction_settled = False
        self._junction_approach_active = False
        self._junction_left_visual_entry = False
        self._junction_left_corridor_since = None
        self._junction_left_near_since = None
        self._junction_left_red_rearmed = False
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
            if self._active_turn == "left" and self.junction_straight_visual_approach:
                # Increasing left-turn authority must not also accelerate the
                # brief straight entry into the intersection.
                speed = min(speed, self.base_speed)
        scale = min(self.speed_scale, 1.0)
        speed *= scale
        bias *= scale
        if self._active_turn == "straight":
            bias += self.straight_encoder_adjustment()
        return (float(np.clip(speed - bias, 0.0, self.max_speed)),
                float(np.clip(speed + bias, 0.0, self.max_speed)), bias)

    def junction_exit_geometry(self):
        if (self._active_turn == "straight"
                and (self.require_client_heartbeat or self.live.enabled)
                and self._junction_exit_geometry is not None):
            return self._junction_exit_geometry
        return self._junction_lane_geometry

    def straight_road_ready(self, geometry):
        """A near aligned corridor can already match the road follower's target.

        The ordinary target (.441) and junction fit target (.49) intentionally
        differ. Do not require a second lateral manoeuvre merely to satisfy the
        latter before allowing the normal road controller to take ownership.
        Heading, near support, ordered pairs and stability are still required.
        """
        return bool(
            self._active_turn == "straight"
            and (self.require_client_heartbeat or self.live.enabled)
            and geometry is not None and geometry.get("valid")
            and geometry.get("near_support") and self._lane_both_visible
            and self._last_lane_error is not None
            and math.isfinite(self._last_lane_error)
            and abs(self._last_lane_error) < self.junction_reacquire_max_error
            and abs(geometry["heading_error"])
            < self.junction_straight_reacquire_max_heading
            and abs(geometry["lateral_error"])
            < (self.junction_straight_reacquire_max_lateral
               + 2 * abs(self.junction_straight_lane_target_fraction
                         - self.lane_target_fraction)))

    def straight_road_corridor(self, geometry, near=False):
        """Recognize a trackable road, without demanding a straight centreline.

        Curve slope in image space is not a robot heading error. Requiring the
        straight alignment threshold here kept E's instruction alive on E->C.
        Paired longitudinal borders, visibility and bounded image errors still
        reject distant/transverse fragments and unrelated side lanes.
        """
        return bool(
            self._departed_red and self._lane_both_visible
            and self._last_lane_error is not None
            and math.isfinite(self._last_lane_error)
            and abs(self._last_lane_error) < .35
            and self.junction_geometry_steering_valid(geometry)
            and geometry.get("pair_count", 0) >= 3
            and geometry.get("row_span", 0.0) >= .36 - 1e-9
            and geometry.get("pairs")
            and geometry["pairs"][-1]["row_fraction"] >= .54
            and abs(geometry["lateral_error"]) < .45
            and abs(geometry["heading_error"]) < .35
            and (not near or (geometry.get("valid") and geometry.get("near_support"))))

    def update_straight_road_tracking(self, now):
        if (self._active_turn != "straight"
                or not (self.require_client_heartbeat or self.live.enabled)):
            return False
        if not self._junction_straight_visual_entry:
            if self.straight_road_corridor(self.junction_exit_geometry()):
                if self._junction_straight_corridor_since is None:
                    self._junction_straight_corridor_since = now
                elif now - self._junction_straight_corridor_since >= .30 - 1e-9:
                    self._junction_straight_visual_entry = True
                    self._junction_approach_active = False
                    self._junction_encoder_start = None
                    self._junction_encoder_adjustment = 0.0
                    self.reset_steering()
            else:
                self._junction_straight_corridor_since = None
        return self._junction_straight_visual_entry

    def straight_reacquisition_good(self):
        geometry = self.junction_exit_geometry()
        if self._active_turn == "straight" and self._junction_straight_visual_entry:
            return self.straight_road_corridor(geometry, near=True)
        return bool(
            self._departed_red
            and geometry is not None
            and geometry.get("valid")
            and geometry.get("near_support")
            and abs(geometry["heading_error"])
            < self.junction_straight_reacquire_max_heading
            and (abs(geometry["lateral_error"])
                 < self.junction_straight_reacquire_max_lateral
                 or self.straight_road_ready(geometry)))

    def left_junction_visual_entry(self, now):
        """Leave the fixed arc on stable outgoing evidence, then keep aligning.

        A mid-field corridor may be visible before the near yellow dash. Entry
        needs ordered rows and a plausible heading; full route acceptance still
        requires near-field centering, heading and the original stability time.
        """
        geometry = self._junction_lane_geometry
        candidate = bool(
            self._departed_red and self.junction_geometry_steering_valid(geometry)
            and geometry.get("pair_count", 0) >= 3
            and geometry.get("row_span", 0.0) >= 0.36 - 1e-9
            and geometry.get("pairs")
            and geometry["pairs"][-1]["row_fraction"] >= 0.54
            and abs(geometry["lateral_error"]) < 0.25
            and abs(geometry["heading_error"]) < 0.15)
        if not self._junction_left_visual_entry:
            if candidate:
                if self._junction_left_corridor_since is None:
                    self._junction_left_corridor_since = now
                elif now - self._junction_left_corridor_since >= 0.10 - 1e-9:
                    self._junction_left_visual_entry = True
                    self.reset_steering()
            else:
                self._junction_left_corridor_since = None
        if not self._junction_left_visual_entry:
            return None
        if not self.junction_geometry_steering_valid(geometry):
            self.navigation_fault("Outgoing corridor lost after left arc; position must be reset")
            return 0.0, 0.0, 0.0
        self._junction_phase = "aligning"
        _, speed, bias = self.junction_profile()
        cap = min(self.max_speed, (speed + abs(bias)) * min(self.speed_scale, 1.0))
        floor = min(cap, self.min_active_wheel_speed)
        steering = float(np.clip(self.straight_visual_steering(geometry),
                                 -(cap - floor) / 2.0, (cap - floor) / 2.0))
        centre = cap - abs(steering)
        return centre - steering, centre + steering, steering

    def right_junction_visual_entry(self, now):
        """End the pivot on a corridor; reserve near-field checks for completion."""
        geometry = self._junction_lane_geometry
        candidate = bool(
            self._departed_red and self.junction_geometry_steering_valid(geometry)
            and geometry.get("pair_count", 0) >= 3
            and geometry.get("row_span", 0.0) >= 0.36 - 1e-9
            and geometry.get("pairs")
            and geometry["pairs"][-1]["row_fraction"] >= 0.54
            and abs(geometry["lateral_error"]) < 0.15
            and abs(geometry["heading_error"]) < 0.15)
        if not self._junction_right_visual_entry:
            if candidate:
                if self._junction_right_corridor_since is None:
                    self._junction_right_corridor_since = now
                elif now - self._junction_right_corridor_since >= 0.10 - 1e-9:
                    self._junction_right_visual_entry = True
                    self._junction_approach_steering = 0.0
                    self.reset_steering()
            else:
                self._junction_right_corridor_since = None
        if not self._junction_right_visual_entry:
            return None
        # A missing near row (for example between yellow dashes) must never
        # restart the fixed pivot. Complete loss of usable visual evidence stops.
        if not self.junction_geometry_steering_valid(geometry):
            self.navigation_fault("Outgoing corridor lost after right pivot; position must be reset")
            return 0.0, 0.0, 0.0
        self._junction_phase = "aligning"
        _, speed, bias = self.junction_profile()
        cap = min(self.max_speed, (speed + abs(bias)) * min(self.speed_scale, 1.0))
        # cap is the OUTER wheel limit, not the centre speed. Mixing around
        # cap then clipping the outer wheel halves steering authority exactly
        # when the outgoing lane still requires heading correction.
        steering = self.straight_visual_steering(geometry)
        floor = min(cap, self.min_active_wheel_speed)
        steering = float(np.clip(steering, -(cap - floor) / 2.0,
                                 (cap - floor) / 2.0))
        centre = cap - abs(steering)
        left, right = centre - steering, centre + steering
        self._junction_encoder_start = None
        self._junction_encoder_adjustment = 0.0
        left, right = self.right_alignment_encoder_wheels(left, right)
        return self.right_junction_tracking_wheels(left, right, (right - left) / 2.0)

    def update_straight_settling(self, steering, now):
        """Require position, heading, and steering to settle together."""
        geometry = self._junction_lane_geometry
        good = bool(
            geometry is not None
            and geometry.get("valid")
            and abs(geometry["lateral_error"])
            < self.junction_straight_settle_max_lateral
            and abs(geometry["heading_error"])
            < self.junction_straight_settle_max_heading
            and abs(steering) < self.junction_straight_settle_max_steering)
        if not good:
            self._junction_settle_good_since = None
            self._junction_settled = False
            return
        if self._junction_settle_good_since is None:
            self._junction_settle_good_since = now
            return
        if now - self._junction_settle_good_since >= self.junction_straight_settle_seconds:
            self._junction_settled = True
            self._junction_phase = "complete"
            if self._last_junction_result is not None:
                self._last_junction_result["alignment"] = "settled"

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

        if self.live.enabled and (not self.live.active or self.live.paused_at is not None):
            return 0.0, 0.0, 0.0

        now = time.monotonic()
        active = self._sharp_corner_state in ("approach", "turning", "reacquiring")
        # Continuous routes may contain tested sharp road bends between
        # junctions. Recognition is allowed only during normal lane following;
        # an authorized intersection always remains owned by navigation_wheels.
        if not active and self.navigation_state != "following":
            return None
        if red_visible:
            if active:
                self.reset_sharp_corner()
                self.red_stop_latched = True
                self._stop_started = now
                self._junction_stop_white_seen = bool(self._white_boundary_visible)
                self.navigation_state = (
                    "route_complete" if self.route_enabled and not self.live.enabled
                    and self.route_index == len(self.route) - 1 else "red_stop")
                self._junction_phase = self.navigation_state
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
        self.live_tick(now)
        if self.live.enabled and (not self.live.active or self.live.paused_at is not None):
            return 0.0, 0.0, 0.0
        state = self.navigation_state
        if (state == "red_stop" and self.route_enabled and self.junctions_calibrated
                and self.auto_continue and not self.live.enabled and not self.manual_stop
                and self._stop_started is not None
                and now - self._stop_started >= self.stop_hold_seconds):
            blocker = self.junction_departure_blocker()
            if blocker is not None:
                self._junction_phase = "departure_blocked"
                return 0.0, 0.0, 0.0
            self.begin_crossing()
            return 0.0, 0.0, 0.0
        if state in ("crossing", "reacquiring"):
            # The course's cross traffic has transverse red lines that can
            # re-enter view after the departure line clears. In an explicitly
            # authorized, bounded crossing they are evidence only, not another
            # stop line. The camera, encoder, ownership and deadline guards
            # still apply; a lane is required before route progress advances.
            if red_visible and self._departed_red:
                self._junction_red_reappeared = True
            if not red_visible:
                if self._red_clear_since is None:
                    self._red_clear_since = now
                self._departed_red |= now - self._red_clear_since >= 0.2
            else:
                self._red_clear_since = None
            duration, _, _ = self.junction_profile()
            visual_turn = (self._active_turn == "right" and self._junction_right_visual_entry
                           or self._active_turn == "left" and self.junction_left_visual_latch
                           and self._junction_left_visual_entry)
            if visual_turn:
                turn = self._active_turn
                near_field = "_junction_%s_near_since" % turn
                rearmed_field = "_junction_%s_red_rearmed" % turn
                geometry = self._junction_lane_geometry or {}
                if geometry.get("valid") and geometry.get("near_support") and not red_visible:
                    if getattr(self, near_field) is None:
                        setattr(self, near_field, now)
                    if now - getattr(self, near_field) >= 0.30 - 1e-9:
                        setattr(self, rearmed_field, True)
                else:
                    setattr(self, near_field, None)
                if red_visible and getattr(self, rearmed_field):
                    # A stable near-field outgoing corridor followed by a red
                    # line is the next junction, even if strict centring did
                    # not finish on a short connecting road. Commit exactly
                    # one route edge and handle that new line as a normal stop.
                    self.red_stop_latched = True
                    # This outgoing corridor already established its white
                    # border. Retain that observation through the red dwell,
                    # even if the border ends underneath the camera meanwhile.
                    self._junction_stop_white_seen = True
                    self.route_index += 1
                    self._active_turn = None
                    self._junction_deadline_at = None
                    self._lane_good_since = None
                    self._stop_started = now
                    self.navigation_state = (
                        "route_complete" if not self.live.enabled and self.route_index == len(self.route) - 1
                        else "red_stop")
                    self._junction_phase = self.navigation_state
                    self._last_junction_result = {
                        "outcome": "reacquired_at_next_red",
                        "turn": turn,
                        "route_index": self.route_index,
                        "alignment": "contained_not_settled",
                    }
                    self.reset_steering()
                    return 0.0, 0.0, 0.0
            dt = min(max(now - self._crossing_updated, 0.0), 0.1)
            self._crossing_updated = now
            self._crossing_progress += dt * min(self.speed_scale, 1.0)
            elapsed = self._crossing_progress
            if (self._junction_deadline_at is None
                    or now >= self._junction_deadline_at):
                self.navigation_fault("Junction traversal exceeded its time limit")
                return 0.0, 0.0, 0.0
            if self._active_turn == "right" and self.junction_straight_visual_approach:
                if self._junction_right_turn_origin is None:
                    # A right exit starts at the end of the incoming white
                    # border, not a fixed interval after releasing the red stop.
                    # Require an observed border before accepting its absence;
                    # flicker or reappearance restarts the clearance sequence.
                    self._junction_phase = "entry"
                    if self._white_boundary_visible:
                        self._junction_right_white_seen = True
                        self._junction_right_absent_since = None
                        self._junction_right_advance_started = None
                    elif self._junction_right_white_seen:
                        if self._junction_right_absent_since is None:
                            self._junction_right_absent_since = now
                        elif now - self._junction_right_absent_since >= 0.15:
                            if self._junction_right_advance_started is None:
                                self._junction_right_advance_started = now
                            elif (now - self._junction_right_advance_started
                                  >= RIGHT_JUNCTION_CLEARANCE_SECONDS):
                                self._junction_right_turn_origin = self._crossing_progress
                                self.reset_steering()
                    if self._junction_right_turn_origin is None:
                        speed = self.base_speed * min(self.speed_scale, 1.0)
                        return self.right_junction_tracking_wheels(speed, speed, 0.0)
                # Waiting/clearance time cannot consume the minimum pivot
                # interval or start the outgoing-lane search timer early.
                elapsed = (self.junction_entry_seconds + self._crossing_progress
                           - self._junction_right_turn_origin)
                # A timed pivot is a search profile, not permission to turn
                # past a visible exit. Check the corridor during the pivot too.
                # Brief initial progress excludes the just-departed approach;
                # the corridor still needs departure clearance and stability.
                if (state == "crossing" and not self._junction_right_visual_entry
                        and self._crossing_progress - self._junction_right_turn_origin >= 0.30
                        and elapsed < self.junction_entry_seconds + duration):
                    visual = self.right_junction_visual_entry(now)
                    if visual is not None:
                        self.navigation_state = "reacquiring"
                        self._reacquire_started = now
                        return visual
            if (elapsed >= self.junction_entry_seconds + duration
                    or (self._active_turn == "right"
                        and self.junction_straight_visual_approach
                        and self._junction_right_visual_entry)):
                if state == "crossing":
                    self.navigation_state = "reacquiring"
                    self._reacquire_started = now
                    self._junction_phase = "searching"
                road_tracking = False
                if self.junction_straight_visual_approach:
                    road_tracking = self.update_straight_road_tracking(now)
                if (self._active_turn in ("straight", "left", "right")
                        and self.junction_straight_visual_approach):
                    good_lane = road_tracking or self.straight_reacquisition_good()
                else:
                    good_lane = (self._departed_red and self._lane_both_visible
                                 and lane_error is not None
                                 and abs(lane_error) < self.junction_reacquire_max_error)
                if good_lane:
                    self._junction_phase = "aligning"
                    if self._lane_good_since is None:
                        self._lane_good_since = now
                    # A stable road corridor finishes the whole straight
                    # maneuver. Do not leave a search deadline attached to the
                    # road follower while waiting for a second alignment gate.
                    required = (0.0 if road_tracking else self.junction_straight_reacquire_seconds
                                if self._active_turn in ("straight", "left", "right")
                                and self.junction_straight_visual_approach else 0.3)
                    if now - self._lane_good_since >= required:
                        completed_turn = self._active_turn
                        self.route_index += 1
                        self.navigation_state = "following"
                        self._active_turn = None
                        # App routes may curve immediately after the crossing.
                        # Stable reacquisition already established the outgoing
                        # lane: release junction steering to the proven road
                        # follower instead of demanding a straight settling road.
                        road_handoff = (completed_turn == "straight"
                                        and (self.require_client_heartbeat or self.live.enabled))
                        self._junction_phase = (
                            "settling" if completed_turn == "straight"
                            and self.junction_straight_visual_approach
                            and not road_handoff else "complete")
                        self._junction_deadline_at = None
                        self._last_junction_result = {
                            "outcome": "reacquired",
                            "turn": completed_turn,
                            "route_index": self.route_index,
                            "alignment": ("lane_following" if road_handoff else
                                          "settling" if completed_turn == "straight"
                                          and self.junction_straight_visual_approach
                                          else "not_required"),
                        }
                        if not road_tracking:
                            self.filtered_error = self.prev_steering = 0.0
                        self._junction_approach_steering = 0.0
                        if self.live.enabled and self.live.stop_after_junction:
                            # Stop in the same control tick as confirmed lane
                            # reacquisition, independently of laptop polling.
                            self.live_tick(now)
                            self.publish_wheels(0.0, 0.0)
                            return 0.0, 0.0, 0.0
                        if road_handoff:
                            self._crossing_started = self._reacquire_started = None
                            self._crossing_updated = None
                            self._crossing_progress = 0.0
                            self._lane_good_since = self._junction_straight_corridor_since = None
                            self._junction_approach_active = False
                            self._junction_encoder_start = None
                            self._junction_encoder_adjustment = 0.0
                            return self.compute_wheel_speeds(lane_error)
                        visual = self.straight_visual_wheels(self.base_speed * self.speed_scale)
                        if visual is not None and completed_turn == "straight":
                            self.update_straight_settling(visual[2], now)
                            return visual
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
                if (self._active_turn == "straight"
                        and self.junction_straight_visual_approach):
                    visual = self.straight_visual_wheels(
                        self.junction_straight_speed * min(self.speed_scale, 1.0))
                    if visual is not None:
                        return visual
                    return self.planned_junction_wheels()
                if (self._active_turn in ("left", "right")
                        and self.junction_straight_visual_approach):
                    if self._active_turn == "left" and self.junction_left_visual_latch:
                        visual = self.left_junction_visual_entry(now)
                        return visual if visual is not None else self.planned_junction_wheels()
                    if self._active_turn == "right":
                        visual = self.right_junction_visual_entry(now)
                        if visual is not None:
                            return visual
                        return self.planned_junction_wheels()
                    # A pair of centroids can belong to transverse road edges
                    # halfway through either turn. Keep the authorized arc or
                    # pivot until a near-field corridor has a plausible
                    # outgoing heading. Only then permit visual centering.
                    geometry = self._junction_lane_geometry
                    if (not self._departed_red or geometry is None
                            or not geometry.get("valid")
                            or not geometry.get("near_support")
                            or abs(geometry["heading_error"])
                            >= self.junction_straight_reacquire_max_heading):
                        return self.planned_junction_wheels()
                    _, speed, bias = self.junction_profile()
                    cap = min(self.max_speed, (speed + abs(bias))
                              * min(self.speed_scale, 1.0))
                    visual = self.straight_visual_wheels(cap)
                    if visual is None:
                        return self.planned_junction_wheels()
                    left, right, _ = visual
                    scale = cap / max(left, right, 0.001)
                    left = min(cap, max(self.min_active_wheel_speed, left * scale))
                    right = min(cap, max(self.min_active_wheel_speed, right * scale))
                    return left, right, (right - left) / 2.0
                if not self._lane_both_visible or lane_error is None:
                    return self.planned_junction_wheels()
                left, right, steering = self.compute_wheel_speeds(lane_error)
                _, junction_speed, junction_bias = self.junction_profile()
                # Keep outgoing alignment above the turn profile's proven
                # outer-wheel command. Scaling both wheels to the centre speed
                # put the first left test back into duck2's weak low-PWM range.
                alignment_cap = junction_speed + abs(junction_bias)
                ratio = alignment_cap / max(left, right, 0.001)
                left, right = left * ratio, right * ratio
                if lane_error is not None and math.isfinite(lane_error):
                    left = max(left, self.min_active_wheel_speed)
                    right = max(right, self.min_active_wheel_speed)
                return left, right, steering
            self._junction_phase = (
                "entry" if elapsed < self.junction_entry_seconds
                else "straight" if self._active_turn == "straight" else "turning")
            if (self._active_turn == "straight" and self.junction_straight_visual_approach
                    and elapsed >= self.junction_entry_seconds):
                forward = self.straight_forward_wheels(
                    self.junction_straight_speed * min(self.speed_scale, 1.0))
                if forward is not None:
                    return forward
            return self.planned_junction_wheels(
                apply_turn=elapsed >= self.junction_entry_seconds)
        if red_visible and not self.red_stop_latched:
            self._junction_stop_white_seen = bool(self._white_boundary_visible)
            if (state == "following" and self.junction_straight_visual_approach
                    and (self.live.active or self.upcoming_junction_turn() == "straight")):
                recent = (self._junction_approach_geometry is not None
                          and self._junction_approach_geometry_time is not None
                          and now - self._junction_approach_geometry_time <= 0.5)
                self._junction_stop_geometry = (
                    copy.deepcopy(self._junction_approach_geometry) if recent else None)
            self.red_stop_latched = True
            self._stop_started = now
            if self.route_enabled and not self.live.enabled and self.route_index == len(self.route)-1:
                self.navigation_state = "route_complete"
                self._junction_phase = "route_complete"
            elif state == "following":
                self.navigation_state = "red_stop"
                self._junction_phase = "stopped"
            rospy.loginfo("Red stop line detected; holding position")
        left, right, steering = self.compute_wheel_speeds(lane_error)
        if (state == "following" and self._junction_phase == "settling"
                and self._last_junction_result is not None
                and self._last_junction_result.get("turn") == "straight"):
            visual = self.straight_visual_wheels(self.base_speed * self.speed_scale)
            if visual is not None:
                self.update_straight_settling(visual[2], now)
                return visual
            self._junction_settle_good_since = None
            self._junction_settled = False
            return left, right, steering
        if state == "following":
            return self.straight_approach_wheels(left, right, steering)
        return left, right, steering

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
            self._junction_lane_geometry = None
            self._junction_exit_geometry = None
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
        self._last_camera_received_at = received_at
        # Serialize image callbacks without blocking Stop, shutdown or the watchdog.
        with self._camera_lock:
            if self._stopping:
                return
            self._camera_processing_started_at = received_at
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
                    self._junction_lane_geometry = perception._junction_lane_geometry
                    self._junction_exit_geometry = perception._junction_exit_geometry
                    self._junction_forward_geometry = perception._junction_forward_geometry
                    self._junction_white_geometry = perception._junction_white_geometry
                    self._junction_white_width_reference = perception._junction_white_width_reference
                    self._lane_both_visible = perception._lane_both_visible
                    self._yellow_boundary_visible = perception._yellow_boundary_visible
                    self._white_boundary_visible = perception._white_boundary_visible
                    self._red_line_visible = perception._red_line_visible
                    self._red_line_detection = perception._red_line_detection
                    self._lane_half_width_px = perception._lane_half_width_px
                    self._lane_half_width_time = perception._lane_half_width_time
                    self._lane_diagnostic = perception._lane_diagnostic
                    self._road_white_reference_used = perception._road_white_reference_used
                    self._road_curve_geometry = perception._road_curve_geometry
                    self._road_left_shape = perception._road_left_shape
                    self._road_left_gap_active = perception._road_left_gap_active
                    self._road_left_boost_gap_active = perception._road_left_boost_gap_active
                    self._left_curve_white_x = perception._left_curve_white_x
                    self._duck_boxes = perception._duck_boxes
                    self._road_geometry = perception._road_geometry
                    self.check_client_connection()
                    self.live_tick(observe=True)
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
                    left_speed, right_speed, steering = self.junction_white_guard_wheels(
                        left_speed, right_speed, steering)
                    self.publish_wheels(left_speed, right_speed)
            except (CvBridgeError, cv2.error, ValueError, TypeError, AttributeError) as error:
                rospy.logwarn("Could not process camera image: %s", error)
                self.reject_camera(error)
                return
            finally:
                self._camera_processing_seconds = time.monotonic()-received_at
                self._camera_processing_started_at = None

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

        if ((self.navigation_state in ("crossing", "reacquiring")
             or (self.junction_white_boundary_guard and self._junction_approach_active
                 and self.navigation_state == "following"))
                and self.frame_count % 5 == 0):
            rospy.loginfo("junction_trace=" + json.dumps({
                "state": self.navigation_state, "phase": self._junction_phase,
                "turn": self._active_turn, "route_index": self.route_index,
                "lane_error": lane_error, "lane_diagnostic": self._lane_diagnostic,
                "wheels": self._last_wheel_speeds,
                "steering_geometry": self._junction_lane_geometry,
                "exit_geometry": self._junction_exit_geometry,
                "forward_geometry": self._junction_forward_geometry,
                "forward_used": self._junction_forward_used,
                "white_geometry": self._junction_white_geometry,
                "white_guard_used": self._junction_white_guard_used,
                "white_guard_reason": self._junction_white_guard_reason,
                "blocker": self.junction_reacquisition_blocker(),
            }, separators=(",", ":")))
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
    cv2.setNumThreads(1)
    lane_follower_node = LaneFollowerNode(node_name="lane_follower_node")
    rospy.spin()
