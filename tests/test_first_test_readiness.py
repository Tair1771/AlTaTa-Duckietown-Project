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
        self.assertIn("_base_speed:=0.05", stand)
        self.assertIn("_max_speed:=0.07", stand)
        self.assertIn("_max_steering:=0.02", stand)

    def test_invalid_parameters_fail_before_subscribing(self):
        bad = {
            "drive_enabled": ["false", 1], "show_debug": ["true"],
            "flip_steering": [0], "route_enabled": ["false"],
            "auto_continue": [1], "junctions_calibrated": ["true"],
            "obstacle_enabled": [1], "require_client_heartbeat": ["false"],
            "k_p": [float("nan"), float("inf"), -1, True, "0.2"],
            "roi_y0_fraction": [.96, -1], "roi_y1_fraction": [.4, 2],
            "red_stop_y_fraction": [.95, 1], "lost_speed": [.08, -.1],
            "base_speed": [2], "max_speed": [1.1, .01],
            "max_steering": [-.1, .5], "deadband": [-1, 1],
            "max_steering_change": [0, -1], "white_left_cutoff": [0, 1],
            "fallback_lane_width_fraction": [float("nan"), 1],
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
