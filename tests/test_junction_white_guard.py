"""Right-border protection during straight junctions, with ROS mocked offline."""
import unittest
from unittest.mock import patch

import cv2
import numpy as np

import test_lane_follower as fixtures


class JunctionWhiteGuardTests(unittest.TestCase):
    setUp = fixtures.LaneTests.setUp
    subscribe = fixtures.LaneTests.subscribe
    message = fixtures.LaneTests.message

    @staticmethod
    def white_mask(far=340, near=410, start=0.05, end=0.96, thickness=12):
        mask = np.zeros((216, 640), np.uint8)
        for row in range(int(start * 215), int(end * 215) + 1):
            x = int(round(far + (near - far) * row / 215.0))
            cv2.line(mask, (x - thickness // 2, row),
                     (x + thickness // 2, row), 255, 1)
        return mask

    def configure(self, state="crossing", turn="straight"):
        n = self.node
        n.junction_white_boundary_guard = True
        n.junctions_calibrated = True
        n.junction_straight_visual_approach = True
        n.junction_straight_approach_max_steering = .03
        n.junction_straight_lane_target_fraction = .49
        n.junction_straight_lateral_gain = .25
        n.junction_straight_heading_gain = .30
        n.max_speed = .20
        n.min_active_wheel_speed = .03
        n.acceleration_limit = 1.0
        n.drive_enabled = True
        n.manual_stop = False
        n.red_stop_latched = False
        n._camera_valid = True
        n._last_frame_time = self.now
        n.navigation_state = state
        n._active_turn = turn
        n._junction_phase = "entry" if state == "crossing" else "searching"
        n._crossing_progress = 0.0
        n._junction_approach_active = state == "following"
        n._sharp_corner_state = "idle"
        n._junction_white_width_reference = None
        n._junction_white_geometry = n.right_white_boundary_geometry(
            self.white_mask(), 640)
        return n

    def apply(self, wheels=(.15, .15, 0.0), frames=4):
        result = wheels
        for _ in range(frames):
            self.now += .1
            self.node._last_frame_time = self.now
            result = self.node.junction_white_guard_wheels(*wheels)
        return result

    def assert_left_correction(self, result, original=(.15, .15, 0.0)):
        left, right, steering = result
        self.assertGreater(right, left)
        self.assertGreater(steering, 0)
        self.assertLessEqual(steering, .03 + 1e-9)
        self.assertAlmostEqual((left + right) / 2, sum(original[:2]) / 2)
        self.assertGreaterEqual(left, self.node.min_active_wheel_speed)
        self.assertLessEqual(right, self.node.max_speed)
        self.assertTrue(self.node._junction_white_guard_used)

    def test_coherent_near_white_works_without_any_yellow(self):
        n = self.configure()
        self.assertTrue(n._junction_white_geometry["usable"])
        self.assertGreaterEqual(len(n._junction_white_geometry["pairs"]), 3)
        self.assert_left_correction(self.apply())

    def test_feature_defaults_off_and_requires_a_real_boolean(self):
        self.assertFalse(self.node.junction_white_boundary_guard)
        self.params["~junction_white_boundary_guard"] = "true"
        with patch.dict(self.mod.os.environ, VEHICLE_NAME="duck2"):
            with self.assertRaisesRegex(ValueError, "junction_white_boundary_guard must be a boolean"):
                self.mod.LaneFollowerNode("invalid-white-guard")

    def test_straight_entry_and_search_both_use_protection(self):
        for state in ("crossing", "reacquiring"):
            with self.subTest(state=state):
                self.configure(state=state)
                self.assert_left_correction(self.apply())

    def test_latched_approach_uses_protection_without_a_committed_turn(self):
        self.configure(state="following", turn=None)
        self.assert_left_correction(self.apply())

    def test_inward_white_can_override_existing_right_request(self):
        self.configure()
        original = (.17, .13, -.02)
        self.assert_left_correction(self.apply(original), original)

    def test_already_stronger_left_correction_is_preserved(self):
        self.configure()
        original = (.07, .17, .05)
        self.assertEqual(self.apply(original), original)

    def test_clear_right_side_white_does_not_apply_permanent_trim(self):
        n = self.configure()
        n._junction_white_geometry = n.right_white_boundary_geometry(
            self.white_mask(far=455, near=565), 640)
        self.assertTrue(n._junction_white_geometry["usable"])
        original = (.15, .15, 0.0)
        self.assertEqual(self.apply(original), original)
        self.assertFalse(n._junction_white_guard_used)

    def test_trustworthy_centered_aligned_pair_prevents_false_single_edge_trim(self):
        n = self.configure()
        n._junction_lane_geometry = dict(valid=True, steering_valid=True,
            near_support=True, lateral_error=0.0, heading_error=0.0,
            image_width=640, pairs=[])
        self.assertEqual(self.apply(), (.15, .15, 0.0))
        self.assertFalse(n._junction_white_guard_used)

    def test_paired_lane_with_rightward_heading_still_allows_edge_protection(self):
        n = self.configure()
        n._junction_lane_geometry = dict(valid=True, steering_valid=True,
            near_support=True, lateral_error=0.0, heading_error=.20,
            image_width=640, pairs=[])
        self.assert_left_correction(self.apply())

    def test_protection_releases_when_white_moves_back_to_the_side(self):
        n = self.configure()
        self.assert_left_correction(self.apply())
        n._junction_white_geometry = n.right_white_boundary_geometry(
            self.white_mask(far=455, near=565), 640)
        self.assertEqual(self.apply(), (.15, .15, 0.0))
        self.assertFalse(n._junction_white_guard_used)
        self.assertIsNone(n._junction_white_guard_reason)

    def test_missing_white_clears_diagnostics_and_keeps_bounded_crossing_profile(self):
        n = self.configure()
        self.assert_left_correction(self.apply())
        n._junction_white_geometry = n.right_white_boundary_geometry(
            np.zeros((216, 640), np.uint8), 640)
        self.assertEqual(self.apply(), (.15, .15, 0.0))
        self.assertFalse(n._junction_white_guard_used)
        self.assertIsNone(n._junction_white_guard_reason)

    def test_horizontal_far_broad_and_jumping_fragments_are_rejected(self):
        horizontal = np.zeros((216, 640), np.uint8)
        cv2.rectangle(horizontal, (230, 110), (630, 126), 255, -1)
        jumping = self.white_mask()
        jumping[80:135] = np.roll(jumping[80:135], 150, axis=1)
        masks = [horizontal, self.white_mask(end=.40),
                 self.white_mask(start=.75),
                 self.white_mask(thickness=200), jumping,
                 np.zeros((216, 640), np.uint8)]
        n = self.configure()
        for mask in masks:
            with self.subTest(nonzero=int(np.count_nonzero(mask))):
                n._junction_white_geometry = n.right_white_boundary_geometry(mask, 640)
                self.assertFalse(n._junction_white_geometry["usable"])
                self.assertEqual(self.apply(), (.15, .15, 0.0))

    def test_ordinary_roads_and_intended_turns_are_unchanged(self):
        cases = [("following", None, False), ("crossing", "left", False),
                 ("crossing", "right", False), ("reacquiring", "left", False),
                 ("reacquiring", "right", False), ("red_stop", None, True),
                 ("route_complete", None, False)]
        for state, turn, approach in cases:
            with self.subTest(state=state, turn=turn):
                n = self.configure(state, turn)
                n._junction_approach_active = approach
                self.assertEqual(self.apply(), (.15, .15, 0.0))

    def test_disabled_invalid_camera_and_stale_geometry_are_unchanged(self):
        for field, value in (("junction_white_boundary_guard", False),
                             ("junctions_calibrated", False),
                             ("junction_straight_visual_approach", False),
                             ("_camera_valid", False), ("drive_enabled", False),
                             ("manual_stop", True), ("red_stop_latched", True)):
            with self.subTest(field=field):
                n = self.configure()
                setattr(n, field, value)
                self.assertEqual(self.apply(), (.15, .15, 0.0))
        n = self.configure()
        self.now += .6
        self.assertEqual(n.junction_white_guard_wheels(.15, .15, 0), (.15, .15, 0))

    def test_zero_and_paused_output_are_never_revived(self):
        n = self.configure()
        self.assertEqual(self.apply((0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
        n.live.enabled = True
        n.live.start("white-guard-pause")
        n.live.request_pause(7, self.now)
        self.assertEqual(self.apply(), (.15, .15, 0.0))
        n.publish_wheels(*self.apply()[:2])
        self.assertEqual(n._last_wheel_speeds, (0.0, 0.0))

    def test_guard_clears_encoder_reference_without_advancing_route(self):
        n = self.configure()
        n._junction_encoder_start = (123, 456)
        n._junction_encoder_adjustment = -.01
        before = (n.navigation_state, n._active_turn, n.route_index)
        self.assert_left_correction(self.apply())
        self.assertIsNone(n._junction_encoder_start)
        self.assertEqual(n._junction_encoder_adjustment, 0)
        self.assertEqual((n.navigation_state, n._active_turn, n.route_index), before)

    def test_recent_paired_width_protects_before_absolute_intrusion(self):
        n = self.configure()
        n._junction_white_geometry = n.right_white_boundary_geometry(
            self.white_mask(far=420, near=500), 640)
        # Previously aligned row pairs provide actual perspective widths;
        # their centers are exactly the dedicated junction target.
        pairs = []
        for fraction, width in ((.18, 300), (.36, 340), (.54, 380),
                                (.72, 420), (.88, 440)):
            pairs.append(dict(row_fraction=fraction, width=width,
                              yellow_x=.49 * 640 - width / 2,
                              white_x=.49 * 640 + width / 2,
                              center_x=.49 * 640))
        n._junction_white_width_reference = dict(
            time=self.now, image_width=640, pairs=pairs)
        self.assert_left_correction(self.apply())

    def test_expired_paired_width_cannot_become_a_permanent_left_trim(self):
        n = self.configure()
        n._junction_white_geometry = n.right_white_boundary_geometry(
            self.white_mask(far=420, near=500), 640)
        n._junction_white_width_reference = dict(time=self.now - 1.0, image_width=640,
            pairs=[dict(row_fraction=f, width=440, yellow_x=93.6,
                        white_x=533.6, center_x=313.6)
                   for f in (.18, .36, .54, .72, .88)])
        self.assertEqual(self.apply(), (.15, .15, 0.0))

    def test_actual_callback_commits_white_geometry_and_guards_output(self):
        n = self.configure()
        n._junction_white_geometry = None
        image = np.zeros((480, 640, 3), np.uint8)
        y0, y1 = int(n.roi_y0_fraction * 480), int(n.roi_y1_fraction * 480)
        mask = cv2.resize(self.white_mask(), (640, y1 - y0), interpolation=cv2.INTER_NEAREST)
        image[y0:y1][mask != 0] = (255, 255, 255)
        with patch.object(n, "navigation_wheels", return_value=(.15, .15, 0.0)), \
                patch.object(n, "sharp_corner_wheels", return_value=None):
            for _ in range(5):
                self.now += .1
                n.callback(self.message(image))
        self.assertTrue(n._junction_white_geometry["usable"])
        self.assertTrue(n._junction_white_guard_used)
        self.assertGreater(n._last_wheel_speeds[1], n._last_wheel_speeds[0])


if __name__ == "__main__":
    unittest.main()
