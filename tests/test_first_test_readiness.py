"""First-test readiness checks with synthetic images; never connects to a robot."""
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

import cv2
import numpy as np
from test_lane_follower import LaneTests


class ReadinessTests(LaneTests):
    def test_camera_guided_curve_setting_turns_then_straightens(self):
        self.node.drive_enabled = True
        self.node.base_speed = .09
        self.node.max_speed = .20
        self.node.max_steering = .11
        self.node.min_active_wheel_speed = .03
        self.node.taper_inner_wheel_floor = False
        self.node.k_p = .75
        self.node.near_center_k_p = .35
        self.node.full_gain_error = .09
        self.node.alpha = .20
        self.node.deadband = .05
        self.node.steering_bias = .0075
        self.node.smooth_steering_deadband = True
        self.node.speed_scale = 1.0
        # With the provisional target, a recorded curve starts near -0.14
        # lane error. This does not establish camera alignment. Preflight camera
        # callbacks settle the filter before emergency stop is released.
        for _ in range(120):
            self.now += .1
            left, right, steering = self.node.compute_wheel_speeds(-.14)
        self.assertAlmostEqual(left, .03, places=6)
        self.assertAlmostEqual(right, .1875, places=6)
        self.assertAlmostEqual(steering, .0975, places=6)
        for _ in range(60):
            self.now += .1
            left, right, steering = self.node.compute_wheel_speeds(-.23)
        # Outside the latched corner state, the slower wheel continues rolling
        # at the verified floor so a saturated correction cannot become a
        # loaded stationary-wheel pivot and stall before corner recognition.
        self.assertAlmostEqual(left, .03, places=6)
        self.assertAlmostEqual(right, .20, places=6)
        self.assertAlmostEqual(steering, .11, places=6)
        for _ in range(60):
            self.now += .1
            left, right, steering = self.node.compute_wheel_speeds(0.0)
        # The bounded test preset retains its small right-turn trim on a
        # straight to compensate the observed loaded left drift.
        self.assertAlmostEqual(left, .0975, places=6)
        self.assertAlmostEqual(right, .0825, places=6)
        self.assertAlmostEqual(steering, -.0075, places=6)

    def test_live_launchers_keep_unverified_features_out_of_first_tests(self):
        root = Path(__file__).resolve().parents[1]
        camera = (root / "launchers/lane-camera-debug.sh").read_text()
        stand = (root / "launchers/lane-follow-stand.sh").read_text()
        road = (root / "launchers/lane-follow.sh").read_text()
        self.assertIn("_drive_enabled:=false", camera)
        self.assertIn("_wheels_topic:=/${VEHICLE_NAME}/lane_follower/diagnostic_wheels_cmd", camera)
        for launcher in (camera, stand, road):
            self.assertIn("_obstacle_enabled:=false", launcher)
            self.assertIn("_avoidance_enabled:=false", launcher)
            self.assertNotIn("_sharp_corner_enabled:=true", launcher)
        self.assertIn("_base_speed:=0.05", stand)
        self.assertIn("_max_speed:=0.07", stand)
        self.assertIn("_max_steering:=0.02", stand)

    def test_bounded_turn_floor_matches_fixed_calibration_and_never_overrides_stop(self):
        self.node.drive_enabled = True
        self.node.base_speed = .09
        self.node.max_speed = .20
        self.node.max_steering = .11
        self.node.min_active_wheel_speed = .03
        self.node.taper_inner_wheel_floor = False
        self.node.k_p = self.node.near_center_k_p = .75
        self.node.max_steering_change = 1.0
        self.node.filtered_error = -.5
        self.now += .1
        left, right, _ = self.node.compute_wheel_speeds(-.5)
        self.assertAlmostEqual(left, .03)
        self.assertEqual(right, .20)
        self.node.filtered_error = .5
        self.now += .1
        left, right, _ = self.node.compute_wheel_speeds(.5)
        self.assertEqual(left, .20)
        self.assertAlmostEqual(right, .03)
        for lost in (None, float("nan"), float("inf")):
            self.assertEqual(self.node.compute_wheel_speeds(lost)[:2], (0., 0.))
        self.node.drive_enabled = False
        self.assertEqual(self.node.compute_wheel_speeds(.5)[:2], (0., 0.))

    def test_inner_wheel_floor_tapers_only_near_maximum_steering(self):
        self.node.drive_enabled = True
        self.node.base_speed = .09
        self.node.max_speed = .20
        self.node.max_steering = .11
        self.node.min_active_wheel_speed = .03
        self.node.taper_inner_wheel_floor = True
        self.node.k_p = self.node.near_center_k_p = 1.0
        self.node.alpha = 1.0
        self.node.deadband = 0.0
        self.node.steering_bias = 0.0
        self.node.max_steering_change = 1.0

        self.node.filtered_error = -.095
        self.now += .1
        left, right, steering = self.node.compute_wheel_speeds(-.095)
        self.assertAlmostEqual(abs(steering), .095)
        self.assertAlmostEqual(min(left, right), .03)

        self.node.filtered_error = -.11
        self.now += .1
        left, right, steering = self.node.compute_wheel_speeds(-.11)
        self.assertAlmostEqual(abs(steering), .11)
        self.assertAlmostEqual(min(left, right), 0.0)
        self.assertAlmostEqual(max(left, right), .20)

    def configure_sharp_corner(self):
        self.node.drive_enabled = True
        self.node.base_speed = .09
        self.node.max_speed = .20
        self.node.sharp_corner_enabled = True
        self.node.sharp_corner_confirm_seconds = .20
        self.node.sharp_corner_recent_lane_seconds = 1.0
        self.node.sharp_corner_approach_seconds = .15
        self.node.sharp_corner_turn_speed = .20
        self.node.sharp_corner_min_turn_seconds = 1.0
        self.node.sharp_corner_white_confirm_seconds = .10
        self.node.sharp_corner_pivot_seconds = .45
        self.node.sharp_corner_relief_seconds = .25
        self.node.sharp_corner_relief_inner_speed = .09
        self.node.sharp_corner_max_turn_seconds = 3.5
        self.node.sharp_corner_reacquire_seconds = .30
        self.node.sharp_corner_trigger_error = .10
        self.node.sharp_corner_exit_error = .13

    def test_sharp_corner_requires_confirmation_and_stable_outgoing_lane(self):
        self.configure_sharp_corner()
        self.node._lane_both_visible = True
        self.node._yellow_boundary_visible = True
        self.node._white_boundary_visible = True
        self.assertIsNone(self.node.sharp_corner_wheels(.05, False))

        self.node._lane_both_visible = False
        self.node._white_boundary_visible = False
        self.now += .01
        self.assertIsNone(self.node.sharp_corner_wheels(.12, False))
        self.now += .21
        self.assertEqual(self.node.sharp_corner_wheels(.14, False), (.09, .09, 0.0))
        self.assertEqual(self.node._sharp_corner_state, "approach")

        self.now += .16
        self.assertEqual(self.node.sharp_corner_wheels(.20, False), (.20, 0.0, -.10))
        self.assertEqual(self.node._sharp_corner_state, "turning")

        # The latched maneuver briefly rolls the inside wheel before a loaded
        # zero-inner-wheel pivot can remain mechanically stalled for 0.75 s.
        self.now += .46
        left, right, steering = self.node.sharp_corner_wheels(.20, False)
        self.assertAlmostEqual(left, .20)
        self.assertAlmostEqual(right, .09)
        self.assertAlmostEqual(steering, -.055)
        self.assertEqual(self.node._sharp_corner_phase, "rolling_relief")
        self.now += .25
        self.assertEqual(
            self.node.sharp_corner_wheels(.20, False),
            (.20, 0.0, -.10),
        )
        self.assertEqual(self.node._sharp_corner_phase, "pivot")

        # The first visible white segment does not finish the maneuver while
        # the lane is still too far from the configured outgoing alignment.
        self.node._lane_both_visible = True
        self.node._white_boundary_visible = True
        self.now += .10
        self.assertEqual(self.node.sharp_corner_wheels(.17, False), (.20, 0.0, -.10))
        for advance in (.10, .10, .10, .11):
            self.now += advance
            result = self.node.sharp_corner_wheels(.12, False)
        self.assertEqual(self.node._sharp_corner_state, "reacquiring")
        self.now += .31
        result = self.node.sharp_corner_wheels(.12, False)
        self.assertEqual(self.node._sharp_corner_state, "cooldown")
        self.assertNotEqual(result[:2], (.20, 0.0))

    def test_continuous_corner_approaches_then_holds_until_stable_lane(self):
        self.configure_sharp_corner()
        self.node.sharp_corner_approach_seconds = 1.0
        self.node.sharp_corner_max_turn_seconds = 10.0
        self.node.sharp_corner_relief_inner_speed = 0.0
        self.node._yellow_boundary_visible = True
        self.node._sharp_corner_state = "approach"
        self.node._sharp_corner_state_since = self.now
        self.now += .99
        self.assertEqual(self.node.sharp_corner_wheels(.2, False), (.09, .09, 0.0))
        self.now += .02
        started = self.now
        for elapsed in (0, .46, .69, 1.0, 3.6, 5.1):
            self.now = started + elapsed
            self.assertEqual(self.node.sharp_corner_wheels(.2, False), (.20, 0, -.10))
            self.assertEqual(self.node._sharp_corner_phase, "pivot")
        self.node._lane_both_visible = True
        self.node._white_boundary_visible = True
        self.node.sharp_corner_wheels(.05, False)
        self.now += .11
        self.node._lane_both_visible = False
        self.node.sharp_corner_wheels(.2, False)
        self.node._lane_both_visible = True
        self.node.sharp_corner_wheels(.05, False)
        self.now += .31
        self.node.sharp_corner_wheels(.05, False)
        self.assertEqual(self.node._sharp_corner_state, "cooldown")

    def test_continuous_corner_deadline_does_not_restart_with_each_pulse_period(self):
        self.configure_sharp_corner()
        self.node.sharp_corner_max_turn_seconds = 10.0
        self.node.sharp_corner_relief_inner_speed = 0.0
        self.node._yellow_boundary_visible = True
        self.node._sharp_corner_state = "turning"
        self.node._sharp_corner_state_since = self.now
        self.now += 10.01
        self.assertEqual(self.node.sharp_corner_wheels(.2, False), (0, 0, 0))
        self.assertEqual(self.node._sharp_corner_state, "fault")
        self.assertTrue(self.node.manual_stop)

    def test_reviewed_corner_pair_approach_and_prompt_white_handoff(self):
        from test_bounded_ground_supervisor import MODULE as supervisor
        args = supervisor.parse_args(["--camera-guided-curve"])
        supervisor.apply_camera_guided_preset(args)
        self.configure_sharp_corner()
        for name, value in vars(args).items():
            if hasattr(self.node, name):
                setattr(self.node, name, value)
        self.node.drive_enabled = True
        self.node._lane_both_visible = True
        self.node._yellow_boundary_visible = True
        self.node._white_boundary_visible = True
        self.assertIsNone(self.node.sharp_corner_wheels(.14, False))
        # Ordinary high-error steering retains both rolling wheels.
        for _ in range(10):
            self.now += .05
            self.assertGreaterEqual(min(self.node.compute_wheel_speeds(.14)[:2]), .03)
            self.node.sharp_corner_wheels(.14, False)
        self.node._lane_both_visible = False
        self.node._white_boundary_visible = False
        self.assertIsNone(self.node.sharp_corner_wheels(.14, False))
        self.now += .21
        self.assertEqual(self.node.sharp_corner_wheels(.14, False), (.09, .09, 0.0))
        self.now += .99
        self.assertEqual(self.node.sharp_corner_wheels(.14, False), (.09, .09, 0.0))
        self.now += .02
        self.assertEqual(self.node.sharp_corner_wheels(.14, False), (.20, 0.0, -.10))
        # No additional minimum pivot duration: stable returning white ends it.
        self.node._white_boundary_visible = True
        self.node._lane_both_visible = True
        self.now += .05
        self.node.sharp_corner_wheels(-.16, False)
        self.now += .11
        pair = self.node.sharp_corner_wheels(-.16, False)
        self.assertEqual(self.node._sharp_corner_state, "reacquiring")
        self.assertGreaterEqual(min(pair[:2]), .03)
        for _ in range(8):
            self.now += .05
            pair = self.node.sharp_corner_wheels(-.16, False)
        self.assertGreater(pair[1], pair[0])
        # Handoff cannot reset the overall ten-second deadline.
        self.now = self.node._sharp_corner_state_since + 10.01
        self.assertEqual(self.node.sharp_corner_wheels(-.16, False), (0., 0., 0.))
        self.assertTrue(self.node.manual_stop)

    def test_pivot_keeps_current_yellow_after_width_expires_then_reacquires(self):
        from test_bounded_ground_supervisor import MODULE as supervisor
        self.configure_sharp_corner()
        self.node.sharp_corner_turn_speed = .15
        self.node.sharp_corner_relief_inner_speed = 0.
        self.node.sharp_corner_min_turn_seconds = 0.
        self.node.sharp_corner_max_turn_seconds = 10.
        self.node.temporal_lane_width_fallback = True
        self.node.temporal_yellow_only_timeout = 5.
        self.node.detect_lane_bgr(self.image())
        self.node._sharp_corner_state = "turning"
        self.node._sharp_corner_state_since = self.now
        self.now += 5.1
        yellow_only = self.image()
        yellow_only[:, 400:] = 0
        error, _, _ = self.node.detect_lane_bgr(yellow_only)
        self.assertIsNone(error)
        self.assertTrue(self.node._yellow_boundary_visible)
        self.assertEqual(self.node._lane_diagnostic, "Lane lost: boundary gap exceeded")
        self.assertEqual(self.node.sharp_corner_wheels(error, False), (.15, 0., -.075))
        status = dict(self.node.status(), camera_valid=True)
        self.assertIsNone(supervisor.motion_fault(
            self.now, True, (self.now, status), True, ["test"], {"test"},
            [(self.now, .15, 0.)], True))
        # Both fresh boundaries rebuild the width estimate and end the pivot.
        self.now += .1
        error, _, _ = self.node.detect_lane_bgr(self.image())
        self.node.sharp_corner_wheels(error, False)
        self.now += .11
        error, _, _ = self.node.detect_lane_bgr(self.image())
        self.node.sharp_corner_wheels(error, False)
        self.assertEqual(self.node._sharp_corner_state, "reacquiring")

    def test_reappearing_white_hands_off_before_yellow_leaves_view(self):
        self.configure_sharp_corner()
        self.node.sharp_corner_relief_inner_speed = 0.0
        self.node.alpha = .20
        self.node.deadband = .05
        self.node.smooth_steering_deadband = True
        self.node.k_p = .75
        self.node.near_center_k_p = .35
        self.node.full_gain_error = .09
        self.node.max_steering = .11
        self.node.steering_bias = .0075
        self.node._sharp_corner_state = "turning"
        self.node._sharp_corner_state_since = self.now - 1.1
        self.node._lane_both_visible = True
        self.node._yellow_boundary_visible = True
        self.node._white_boundary_visible = True
        self.assertEqual(self.node.sharp_corner_wheels(-.13, False), (.20, 0, -.10))
        self.now += .11
        self.node._lane_both_visible = False
        self.node._yellow_boundary_visible = False
        self.node._white_boundary_visible = True
        left, right, steering = self.node.sharp_corner_wheels(-.09, False)
        self.assertEqual(self.node._sharp_corner_state, "reacquiring")
        self.assertEqual(self.node._sharp_corner_phase, "lane_reacquire")
        self.assertFalse(self.node.manual_stop)
        # The fixed 0.20/0.00 pivot has ended. Resetting the filter makes the
        # first normal-controller correction deliberately tiny; the retained
        # steering trim means it need not be exactly zero.
        self.assertAlmostEqual(left, .09, delta=.002)
        self.assertAlmostEqual(right, .09, delta=.002)
        self.assertLess(abs(steering), .002)
        self.now += .1
        left, right, steering = self.node.sharp_corner_wheels(-.09, False)
        self.assertGreater(right, left)
        self.assertGreater(steering, 0)

    def test_sharp_corner_rejects_unconfirmed_or_unsafe_geometry(self):
        self.configure_sharp_corner()
        self.node._lane_both_visible = False
        self.node._yellow_boundary_visible = True
        self.node._white_boundary_visible = False
        # No recently complete lane: yellow-only does not trigger a maneuver.
        self.now += .3
        self.assertIsNone(self.node.sharp_corner_wheels(.4, False))

        self.node._lane_both_visible = True
        self.node._white_boundary_visible = True
        self.assertIsNone(self.node.sharp_corner_wheels(.05, False))
        self.node._lane_both_visible = False
        self.node._white_boundary_visible = False
        self.now += .21
        self.assertIsNone(self.node.sharp_corner_wheels(.05, False))
        self.assertEqual(self.node._sharp_corner_state, "idle")
        self.now += .01
        self.assertIsNone(self.node.sharp_corner_wheels(.2, False))
        self.now += .21
        self.node.sharp_corner_wheels(.2, False)
        self.now += .16
        self.node._yellow_boundary_visible = False
        self.assertEqual(self.node.sharp_corner_wheels(None, False), (0.0, 0.0, 0.0))
        self.assertEqual(self.node._sharp_corner_state, "fault")
        self.assertTrue(self.node.manual_stop)

    def test_red_line_stops_an_active_sharp_corner(self):
        self.configure_sharp_corner()
        self.node._sharp_corner_state = "turning"
        self.node._sharp_corner_state_since = self.now
        self.node._yellow_boundary_visible = True
        self.assertEqual(self.node.sharp_corner_wheels(.2, True), (0.0, 0.0, 0.0))
        self.assertTrue(self.node.red_stop_latched)
        self.assertEqual(self.node.navigation_state, "red_stop")

    def test_sharp_corner_timeout_and_camera_loss_latch_faults(self):
        self.configure_sharp_corner()
        self.node._sharp_corner_state = "turning"
        self.node._sharp_corner_state_since = self.now - 3.51
        self.node._yellow_boundary_visible = True
        self.assertEqual(self.node.sharp_corner_wheels(.2, False), (0.0, 0.0, 0.0))
        self.assertEqual(self.node.navigation_state, "fault")

        self.setUp()
        self.configure_sharp_corner()
        self.node._camera_valid = True
        self.node._sharp_corner_state = "turning"
        self.node._sharp_corner_state_since = self.now
        self.node._yellow_boundary_visible = True
        self.node._last_frame_time = self.now - .6
        self.node.check_camera_timeout(None)
        self.assertEqual(self.node._sharp_corner_state, "fault")
        self.assertEqual(self.speeds(), (0.0, 0.0))

    def test_invalid_parameters_fail_before_subscribing(self):
        bad = {
            "drive_enabled": ["false", 1], "show_debug": ["true"],
            "flip_steering": [0], "route_enabled": ["false"],
            "smooth_steering_deadband": ["true", 1, None],
            "temporal_lane_width_fallback": ["true", 1, None],
            "boundary_risk_stop": ["true", 1, None],
            "taper_inner_wheel_floor": ["true", 1, None],
            "sharp_corner_enabled": ["true", 1, None],
            "auto_continue": [1], "junctions_calibrated": ["true"],
            "obstacle_enabled": [1], "require_client_heartbeat": ["false"],
            "k_p": [float("nan"), float("inf"), -1, True, "0.2"],
            "near_center_k_p": [-1, .3],
            "full_gain_error": [-1, .03, 1],
            "roi_y0_fraction": [.96, -1], "roi_y1_fraction": [.4, 2],
            "red_stop_y_fraction": [.95, 1], "lost_speed": [.08, -.1],
            "red_stop_trigger_bottom_fraction": [.64, .95, float("nan")],
            "base_speed": [2], "max_speed": [1.1, .01],
            "min_active_wheel_speed": [-.01, .09, float("nan")],
            "max_steering": [-.1, .5], "deadband": [-1, 1],
            "steering_bias": [float("nan"), .5, -.5],
            "lane_target_fraction": [0, 1, -0.1],
            "max_steering_change": [0, -1], "white_left_cutoff": [0, 1],
            "fallback_lane_width_fraction": [float("nan"), 1],
            "temporal_lane_width_timeout": [0, -.1, .51, float("nan")],
            "temporal_yellow_only_timeout": [0, -.1, 5.01, float("nan")],
            "white_boundary_risk_fraction": [.35, 1, float("nan")],
            "sharp_corner_confirm_seconds": [.09, .51, float("nan")],
            "sharp_corner_recent_lane_seconds": [.49, 1.51, float("nan")],
            "sharp_corner_approach_seconds": [-.01, 1.01, float("nan")],
            "sharp_corner_turn_speed": [0, .19, float("nan")],
            "sharp_corner_min_turn_seconds": [-.01, 3.01, float("nan")],
            "sharp_corner_white_confirm_seconds": [.04, .31, float("nan")],
            "sharp_corner_pivot_seconds": [.19, .76, float("nan")],
            "sharp_corner_relief_seconds": [.09, .51, float("nan")],
            "sharp_corner_relief_inner_speed": [-.01, .09, float("nan")],
            "sharp_corner_max_turn_seconds": [.49, 10.01, float("nan")],
            "sharp_corner_reacquire_seconds": [.19, 1.01, float("nan")],
            "sharp_corner_trigger_error": [0, 1, float("nan")],
            "sharp_corner_exit_error": [0, .35, float("nan")],
            "junction_left_seconds": [0, float("nan")],
            "junction_right_seconds": [0, float("nan")],
            "junction_straight_speed": [0, .19, float("nan")],
            "junction_left_speed": [0, .19, float("nan")],
            "junction_right_speed": [0, .19, float("nan")],
            "junction_left_bias": [-.01, .06, float("nan")],
            "junction_right_bias": [-.01, .06, float("nan")],
            "yellow_lower": [[24, 140], [181, 140, 120], [True, 140, 120], [37, 140, 120]],
            "red_high_upper": [[180, 256, 255]],
        }
        for name, values in bad.items():
            for value in values:
                with self.subTest(name=name, value=value):
                    self.params = {"~" + name: value}
                    with patch.object(self.mod.rospy, "Subscriber") as subscribe:
                        with patch.dict(self.mod.os.environ, VEHICLE_NAME="duck2"):
                            with self.assertRaises(ValueError):
                                self.mod.LaneFollowerNode("invalid")
                        subscribe.assert_not_called()

    def test_colour_defaults_and_overrides(self):
        self.assertEqual(self.node.yellow_lower.tolist(), [24, 140, 120])
        self.assertEqual(self.node.white_upper.tolist(), [180, 55, 255])
        self.assertEqual(self.node.red_low_lower.tolist(), [0, 110, 90])
        self.assertEqual(self.node.red_high_upper.tolist(), [180, 255, 255])
        self.params = {"~yellow_lower": [25, 145, 125]}
        with patch.dict(self.mod.os.environ, VEHICLE_NAME="duck2"):
            other = self.mod.LaneFollowerNode("custom")
        self.assertEqual(other.yellow_lower.tolist(), [25, 145, 125])

    def test_gain_schedule_is_gentle_near_center_and_full_on_curve(self):
        self.node.drive_enabled = True
        self.node.base_speed = .09
        self.node.max_speed = .15
        self.node.max_steering = .09
        self.node.k_p = .75
        self.node.near_center_k_p = .35
        self.node.full_gain_error = .09
        self.node.deadband = .05
        self.node.smooth_steering_deadband = True
        self.node.steering_bias = .0075
        self.node.max_steering_change = 1.0

        self.node.filtered_error = .05
        left, right, steering = self.node.compute_wheel_speeds(.05)
        self.assertAlmostEqual(left, .115, places=6)
        self.assertAlmostEqual(right, .065, places=6)
        self.assertAlmostEqual(steering, -.025, places=6)

        self.now += .1
        self.node.filtered_error = -.14
        left, right, steering = self.node.compute_wheel_speeds(-.14)
        self.assertAlmostEqual(left, 0.0, places=6)
        self.assertAlmostEqual(right, .15, places=6)
        self.assertAlmostEqual(steering, .09, places=6)

    def test_disabled_obstacle_handling_does_not_mask_lane_dividers(self):
        self.node.obstacle_enabled = False
        self.node.drive_enabled = False
        with patch.object(self.node, "detect_ducks_bgr") as detect_ducks:
            self.now += .1
            self.deliver(self.image())
        detect_ducks.assert_not_called()
        self.assertEqual(self.node._duck_boxes, [])

    def test_invalid_stamps_cannot_refresh_or_advance_state(self):
        self.node.drive_enabled = True
        self.now += .1
        self.deliver(self.image())
        accepted = self.node._last_camera_stamp
        for stamp in [0, -1, float("nan"), float("inf"), accepted, accepted-.01,
                      self.now-1, self.now+.2]:
            with self.subTest(stamp=stamp):
                index = self.node.route_index
                self.node.callback(self.message(self.image(), stamp))
                self.assertEqual(self.speeds(), (0, 0))
                self.assertFalse(self.node._camera_valid)
                self.assertEqual(self.node._last_camera_stamp, accepted)
                self.assertEqual(self.node.route_index, index)
                self.assertIsNone(self.node._obstacle_clear_since)
        self.node.callback(NS(image=self.image()))
        self.assertIn("missing", self.node._camera_error)
        self.now += .1
        self.deliver(self.image())
        self.assertTrue(self.node._camera_valid)
        self.assertGreater(max(self.speeds()), 0)
        self.assertLessEqual(max(self.speeds()), .0150001)

    def test_source_age_consumes_watchdog_budget(self):
        self.node.drive_enabled = True
        self.now += 1
        self.node.callback(self.message(self.image(), self.now-.4))
        self.assertTrue(self.node._camera_valid)
        self.now += .11
        self.node.check_camera_timeout(None)
        self.assertEqual(self.speeds(), (0, 0))
        self.assertFalse(self.node._camera_valid)

    def test_rejected_image_faults_junction_without_advancing(self):
        self.node.drive_enabled = True
        self.node.navigation_state = "crossing"
        index = self.node.route_index
        self.node.callback(self.message(self.image(), 0))
        self.assertEqual(self.node.navigation_state, "fault")
        self.assertEqual(self.node.route_index, index)
        self.assertEqual(self.speeds(), (0, 0))
        self.assertIsNone(self.node._obstacle_clear_since)

    def test_watchdog_can_stop_while_image_processing_is_blocked(self):
        entered, release = threading.Event(), threading.Event()
        self.node.drive_enabled = True
        self.now += .1
        self.deliver(self.image())
        self.assertGreater(max(self.speeds()), 0)
        def decode(msg, **kwargs):
            entered.set()
            if not release.wait(2):
                raise ValueError("Test release timed out")
            return msg.image
        self.node.bridge.compressed_imgmsg_to_cv2 = decode
        self.now += .1
        worker = threading.Thread(target=self.node.callback, args=(self.message(self.image()),))
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            self.now += .6
            self.node.check_camera_timeout(None)
            self.assertEqual(self.speeds(), (0, 0))
        finally:
            release.set()
            worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertFalse(self.node._camera_valid)
        self.assertEqual(self.speeds(), (0, 0))

    def test_slow_processing_discards_all_perception_results(self):
        def slow_decode(msg, **kwargs):
            self.now += .6
            return msg.image
        self.node.drive_enabled = True
        self.now += .1
        self.node.bridge.compressed_imgmsg_to_cv2 = slow_decode
        self.node.callback(self.message(self.image(((260, 370), (580, 390)))))
        self.assertFalse(self.node.red_stop_latched)
        self.assertFalse(self.node._lane_both_visible)
        self.assertIsNone(self.node._lane_limits)
        self.assertIsNone(self.node._obstacle_clear_since)
        self.assertEqual(self.speeds(), (0, 0))

    def test_shutdown_during_processing_does_not_wait_or_restart(self):
        entered, release = threading.Event(), threading.Event()
        errors = []
        def blocked_decode(msg, **kwargs):
            entered.set()
            if not release.wait(2):
                raise ValueError("Test release timed out")
            return msg.image
        def deliver():
            try:
                self.deliver(self.image())
            except Exception as error:
                errors.append(error)
        self.node.bridge.compressed_imgmsg_to_cv2 = blocked_decode
        self.node.drive_enabled = True
        self.now += .1
        worker = threading.Thread(target=deliver)
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            self.node.on_shutdown()
            self.assertEqual(self.speeds(), (0, 0))
        finally:
            release.set()
            worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertFalse(errors)
        self.assertEqual(self.speeds(), (0, 0))
        self.assertEqual((self.node.filtered_error, self.node.prev_steering), (0, 0))

    def test_overlapping_callbacks_are_serialized(self):
        entered, release = threading.Event(), threading.Event()
        calls = []
        first = self.message(self.image(), self.now+.01)
        second = self.message(self.image(), self.now+.02)
        def decode(msg, **kwargs):
            calls.append(msg)
            if msg is first:
                entered.set()
                if not release.wait(2):
                    raise ValueError("Test release timed out")
            return msg.image
        self.node.bridge.compressed_imgmsg_to_cv2 = decode
        one = threading.Thread(target=self.node.callback, args=(first,))
        two = threading.Thread(target=self.node.callback, args=(second,))
        one.start()
        try:
            self.assertTrue(entered.wait(1))
            two.start()
            self.assertEqual(len(calls), 1)
        finally:
            release.set()
            one.join(2)
            if two.ident is not None:
                two.join(2)
        self.assertEqual(len(calls), 2)
        self.assertEqual(self.node._last_camera_stamp, self.now+.02)

    def test_image_validation_and_outside_lane_centre(self):
        for bad in [np.zeros((1, 1, 3), np.uint8), np.zeros((480, 640, 4), np.uint8),
                    np.zeros((480, 640, 3), float), "not an image"]:
            self.now += .1
            self.node.callback(self.message(bad))
            self.assertFalse(self.node._camera_valid)
            self.assertEqual(self.speeds(), (0, 0))
        self.node.fallback_lane_width_fraction = .9
        image = self.image()
        image[:, 400:] = 0
        error, _, _ = self.node.detect_lane_bgr(image)
        self.assertIsNone(error)
        self.assertIn("outside", self.node._lane_diagnostic)

    def test_lane_target_fraction_shifts_error_without_changing_detection(self):
        image = self.image()
        default_error, _, _ = self.node.detect_lane_bgr(image)
        self.node.lane_target_fraction = .456
        calibrated_error, _, _ = self.node.detect_lane_bgr(image)
        self.assertAlmostEqual(calibrated_error - default_error, .088, places=6)
        self.assertEqual(self.node._lane_diagnostic, "Both boundaries")

    def test_temporal_lane_width_bridges_dash_gap_without_center_jump(self):
        wide_lane = np.zeros((480, 640, 3), np.uint8)
        cv2.rectangle(wide_lane, (80, 240), (100, 455), (0, 255, 255), -1)
        cv2.rectangle(wide_lane, (520, 240), (540, 455), (255, 255, 255), -1)
        white_only = wide_lane.copy()
        white_only[:, :300] = 0

        # The ordinary configuration retains its existing fixed-width fallback.
        both_error = self.node.detect_lane_bgr(wide_lane)[0]
        fixed_error = self.node.detect_lane_bgr(white_only)[0]
        self.assertEqual(self.node._lane_diagnostic, "White-only fallback")
        self.assertGreater(abs(fixed_error - both_error), .1)
        self.assertFalse(self.node.status()["temporal_lane_width_fallback"])

        # The bounded test preset opts in. Exercise the real callback commit so
        # the remembered width cannot be lost in the private perception copy.
        self.node.temporal_lane_width_fallback = True
        self.now += .1
        self.deliver(wide_lane)
        both_error = self.node._last_lane_error
        measured_half_width = self.node._lane_half_width_px
        self.assertAlmostEqual(measured_half_width, 220.0, delta=2.0)
        self.now += .1
        self.deliver(white_only)
        self.assertEqual(self.node._lane_diagnostic, "White-only temporal fallback")
        self.assertAlmostEqual(self.node._last_lane_error, both_error, places=6)
        self.assertEqual(self.node.status()["lane_half_width_px"], measured_half_width)
        self.assertTrue(self.node.status()["lane_both_visible"] is False)

        # The longest single-boundary gap in the accepted recording was 0.201
        # seconds. A longer loss must stop instead of reusing old geometry.
        self.now += .1
        self.deliver(wide_lane)
        self.now += self.node.temporal_lane_width_timeout + .01
        self.deliver(white_only)
        self.assertIsNone(self.node._last_lane_error)
        self.assertEqual(self.node._lane_diagnostic,
                         "Lane lost: boundary gap exceeded")
        self.assertEqual(self.speeds(), (0, 0))
        self.assertIsNone(self.node._lane_half_width_px)

        # A real lane loss clears the memory rather than applying it to a later,
        # unrelated scene.
        self.now += .1
        self.deliver(np.zeros_like(wide_lane))
        self.assertIsNone(self.node._lane_half_width_px)
        self.assertIsNone(self.node._last_lane_error)

    def test_yellow_only_can_use_a_separate_bounded_corner_timeout(self):
        lane = np.zeros((480, 640, 3), np.uint8)
        cv2.rectangle(lane, (80, 240), (100, 455), (0, 255, 255), -1)
        cv2.rectangle(lane, (520, 240), (540, 455), (255, 255, 255), -1)
        yellow_only = lane.copy()
        yellow_only[:, 300:] = 0
        self.node.temporal_lane_width_fallback = True
        self.node.temporal_yellow_only_timeout = 5.0

        self.now += .1
        self.deliver(lane)
        both_error = self.node._last_lane_error
        self.now += 4.8  # Beyond the white-only timeout, within corner timeout.
        self.deliver(yellow_only)
        self.assertEqual(self.node._lane_diagnostic, "Yellow-only temporal fallback")
        self.assertAlmostEqual(self.node._last_lane_error, both_error, places=6)

        self.now += .21
        self.deliver(yellow_only)
        self.assertIsNone(self.node._last_lane_error)
        self.assertEqual(self.node._lane_diagnostic,
                         "Lane lost: boundary gap exceeded")
        self.assertEqual(self.speeds(), (0, 0))
        self.assertIsNone(self.node._lane_half_width_px)

    def test_test_only_white_boundary_guard_stops_before_cutoff(self):
        lane = np.zeros((480, 640, 3), np.uint8)
        cv2.rectangle(lane, (80, 240), (100, 455), (0, 255, 255), -1)
        cv2.rectangle(lane, (250, 240), (270, 455), (255, 255, 255), -1)
        white_only = lane.copy()
        white_only[:, :200] = 0
        self.node.temporal_lane_width_fallback = True
        self.node.boundary_risk_stop = True
        self.now += .1
        self.deliver(lane)
        self.now += .1
        self.deliver(white_only)
        self.assertIsNone(self.node._last_lane_error)
        self.assertEqual(self.node._lane_diagnostic,
                         "Lane lost: white boundary risk")
        self.assertEqual(self.speeds(), (0, 0))

    def test_dim_yellow_requires_bounded_test_override(self):
        hsv = np.zeros((480, 640, 3), np.uint8)
        hsv[:, :, 2] = 20
        # Median HSV from duck2's own stationary curve recording.
        cv2.rectangle(hsv, (80, 240), (100, 455), (26, 108, 169), -1)
        image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.rectangle(image, (520, 240), (540, 455), (255, 255, 255), -1)
        self.node.detect_lane_bgr(image)
        self.assertEqual(self.node._lane_diagnostic, "White-only fallback")
        self.node.yellow_lower = np.array([20, 70, 80], dtype=np.uint8)
        self.node.yellow_upper = np.array([35, 255, 255], dtype=np.uint8)
        error, _, _ = self.node.detect_lane_bgr(image)
        self.assertIsNotNone(error)
        self.assertEqual(self.node._lane_diagnostic, "Both boundaries")

    def test_synthetic_lane_variations_and_shadow_loss(self):
        images = [self.image()]
        dashed = self.image()
        for y in range(240, 455, 60):
            dashed[y:y+25, :300] = 0
        images.append(dashed)
        for direction in [-1, 1]:
            curved = np.zeros_like(self.image())
            for x, colour in [(170, (0, 255, 255)), (480, (255, 255, 255))]:
                points = np.array([(x+int(direction*60*((455-y)/215)**2), y)
                                   for y in range(240, 456)], np.int32)
                cv2.polylines(curved, [points], False, colour, 20)
            images.append(curved)
        for image in images:
            error, _, _ = self.node.detect_lane_bgr(image)
            self.assertIsNotNone(error)
            self.assertLess(abs(error), .3)
        for side, label in [(slice(0, 300), "White-only"), (slice(400, 640), "Yellow-only")]:
            image = self.image()
            image[:, side] = 0
            error, _, _ = self.node.detect_lane_bgr(image)
            self.assertIsNotNone(error)
            self.assertIn(label, self.node._lane_diagnostic)
        # An isolated small distractor should not materially displace the estimate.
        image = self.image()
        cv2.circle(image, (330, 350), 2, (0, 255, 255), -1)
        error, _, _ = self.node.detect_lane_bgr(image)
        self.assertLess(abs(error), .04)
        shadow = (self.image().astype(float)*.1).astype(np.uint8)
        self.node.drive_enabled = True
        self.now += .1
        self.deliver(shadow)
        self.assertEqual(self.speeds(), (0, 0))
        self.assertIn("no boundaries", self.node._lane_diagnostic)
        reversed_pair = np.zeros_like(shadow)
        cv2.rectangle(reversed_pair, (330, 240), (350, 455), (0, 255, 255), -1)
        cv2.rectangle(reversed_pair, (250, 240), (270, 455), (255, 255, 255), -1)
        self.assertIsNone(self.node.detect_lane_bgr(reversed_pair)[0])

    def test_stop_and_loss_reset_steering_and_recovery_ramps(self):
        self.node.drive_enabled = True
        for reason in ["manual_stop", "red_stop_latched", "obstacle_stop_latched"]:
            self.node.filtered_error, self.node.prev_steering = .7, .05
            setattr(self.node, reason, True)
            self.node.publish_wheels(.1, .1)
            self.assertEqual((self.node.filtered_error, self.node.prev_steering), (0, 0))
            setattr(self.node, reason, False)
        for error in [None, float("nan"), float("inf")]:
            self.node.filtered_error, self.node.prev_steering = .7, .05
            self.assertEqual(self.node.compute_wheel_speeds(error)[:2], (0, 0))
            self.assertEqual((self.node.filtered_error, self.node.prev_steering), (0, 0))
        self.now += .1
        self.deliver(self.image())
        self.assertLessEqual(max(self.speeds()), .0150001)

    def test_steering_sign_for_both_flip_settings(self):
        self.node.drive_enabled = True
        for flip in [False, True]:
            for error in [-.5, .5]:
                self.node.flip_steering = flip
                self.node.reset_steering()
                for _ in range(30):
                    self.now += 1/30
                    left, right, _ = self.node.compute_wheel_speeds(error)
                self.assertGreater((right-left)*error*(-1 if flip else 1), 0)
                self.assertTrue(0 <= left <= self.node.max_speed)
                self.assertTrue(0 <= right <= self.node.max_speed)

    def test_debug_shows_diagnostics_with_no_lane(self):
        self.node.show_debug = True
        with patch.object(cv2, "imshow") as show, patch.object(cv2, "waitKey"):
            self.now += .1
            self.deliver(np.zeros_like(self.image()))
        self.assertEqual(show.call_count, 2)
        preview = show.call_args_list[0][0][1]
        self.assertGreater(np.count_nonzero(preview), 0)
        self.assertIn("no boundaries", self.node.status()["lane_diagnostic"])
        self.assertEqual(self.node.status()["stop_reason"], "Driving disabled")
        self.assertEqual(self.speeds(), (0, 0))


def load_tests(loader, tests, pattern):
    return unittest.TestSuite(ReadinessTests(name) for name in ReadinessTests.__dict__
                              if name.startswith("test_"))


if __name__ == "__main__":
    unittest.main()
