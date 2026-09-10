"""Actual camera callbacks through yellow gaps; no physical ROS transport."""
import socket
import unittest
from unittest.mock import patch
import test_left_road_response as scenes


class CurveYellowGapTests(unittest.TestCase):
    subscribe = scenes.LeftRoadResponseTests.subscribe
    message = scenes.LeftRoadResponseTests.message
    def road(self, **kwargs):
        kwargs.setdefault('offset', -30)
        return scenes.LeftRoadResponseTests.road(self, **kwargs)

    def setUp(self):
        scenes.LeftRoadResponseTests.setUp(self)
        self.block = patch.object(socket, 'socket', side_effect=AssertionError('Robot network blocked'))
        self.block.start()
        self.addCleanup(self.block.stop)
        n = self.node
        n.temporal_lane_width_fallback = True
        n.temporal_lane_width_timeout = .30
        n.temporal_yellow_only_timeout = 5.
        n.road_left_lookahead = False
        n.road_white_reference_value = 170
        n.white_lower[2] = 150
        n.lane_target_fraction = .441

    def frame(self, image, dt=.1):
        self.now += dt
        self.node.callback(self.message(image))
        return self.node._last_wheel_speeds

    def confirm_curve(self):
        for _ in range(12):
            self.frame(self.road(dashed=True))
        self.assertTrue(self.node._road_left_boost_active)

    def test_confirmed_curve_keeps_turning_through_short_yellow_gap(self):
        self.confirm_curve()
        for _ in range(6):
            left, right = self.frame(self.road(missing='yellow'))
            self.assertGreater(right, left)
            self.assertGreater(right, .15)
        self.assertIsNotNone(self.node._last_lane_error)

    def test_moving_white_edge_does_not_reverse_a_confirmed_curve(self):
        self.confirm_curve()
        n = self.node
        confirmed_at = n._road_left_boost_last_seen
        # The same curved white edge moves just 20 px in a 640 px image.
        # Its old half-width makes the centroid error positive, although the
        # matched-depth white trace still confirms the recent left bend.
        for _ in range(7):
            left, right = self.frame(
                self.road(missing='yellow', offset=-10), dt=.05)
            self.assertGreater(n._last_lane_error, .03)
            self.assertTrue(n._road_left_boost_gap_active)
            self.assertGreater(right, left)
            self.assertEqual(n._road_left_boost_last_seen, confirmed_at)
        self.assertAlmostEqual(left, .03)
        self.assertAlmostEqual(right, .20)

    def test_intermittent_yellow_does_not_command_right_on_a_left_curve(self):
        self.confirm_curve()
        n = self.node
        # Sparse dashes return every 0.2 s. The mixed-depth centroid remains
        # above the former .03 cutoff; near/far boundary pairs still describe
        # a centered-enough left curve. Previously the renewing lane width
        # kept driving alive while steering eventually reversed to the right.
        for index in range(20):
            missing = None if index % 4 == 3 else 'yellow'
            left, right = self.frame(
                self.road(offset=-10, missing=missing, dashed=True), dt=.05)
            self.assertGreater(n._last_lane_error, .03)
            self.assertTrue(n._road_left_boost_active)
            self.assertGreater(right, left)
            if missing is None:
                self.assertTrue(n._road_left_shape['candidate'])
                self.assertTrue(n._lane_both_visible)
                self.assertFalse(n._road_left_boost_gap_active)
            else:
                self.assertTrue(n._road_left_boost_gap_active)
        self.assertAlmostEqual(left, .03)
        self.assertAlmostEqual(right, .20)

    def test_full_boundaries_retain_right_correction_for_large_left_offset(self):
        self.confirm_curve()
        n = self.node
        # A real large left-of-lane offset must retain recovery authority;
        # curvature alone must never force the calibrated left-wheel pair.
        for _ in range(15):
            left, right = self.frame(self.road(offset=100, dashed=True))
            self.assertTrue(n._road_left_shape['candidate'])
            self.assertGreater(n._last_lane_error, .30)
            self.assertFalse(n._road_left_boost_active)
            self.assertIsNone(n._road_left_boost_last_seen)
        self.assertGreater(left, right)

    def test_yellow_gap_keeps_bright_white_reference_and_lane_error(self):
        import cv2
        import numpy as np
        image = self.road(dashed=True)
        # A dim, broad fragment passes the normal V=150 mask but must not
        # replace the V=170 white reference just because yellow disappears.
        cv2.rectangle(image, (565, 280), (595, 435), (160, 160, 160), -1)
        for _ in range(12):
            self.frame(image)
        n = self.node
        self.assertTrue(n._road_left_boost_active)
        error = n._last_lane_error
        missing = image.copy()
        hsv = cv2.cvtColor(missing, cv2.COLOR_BGR2HSV)
        missing[cv2.inRange(hsv, n.yellow_lower, n.yellow_upper) > 0] = 0
        for _ in range(5):
            self.frame(missing)
            self.assertTrue(n._road_white_reference_used)
            self.assertAlmostEqual(n._last_lane_error, error, places=5)
            self.assertTrue(n._road_left_boost_gap_active)

    def test_white_alone_cannot_extend_confirmation_indefinitely(self):
        self.confirm_curve()
        for _ in range(10):
            wheels = self.frame(self.road(missing='yellow'))
        self.assertEqual(wheels, (0., 0.))

    def test_yellow_return_then_straight_releases_curve_without_stopping(self):
        self.confirm_curve()
        for _ in range(5):
            self.frame(self.road(missing='yellow'))
        self.frame(self.road(dashed=True))
        self.assertTrue(self.node._road_left_boost_active)
        self.assertFalse(self.node._road_left_boost_gap_active)
        self.frame(self.road(bend=0))
        self.assertFalse(self.node._road_left_boost_active)
        self.assertGreater(sum(self.node._last_wheel_speeds), 0)

    def test_straight_white_jumping_white_and_complete_loss_fail_closed(self):
        import numpy as np
        for image in (self.road(bend=0, missing='yellow'),
                      self.road(offset=80, missing='yellow'),
                      np.zeros((480, 640, 3), np.uint8)):
            self.confirm_curve()
            self.frame(image)
            self.assertFalse(self.node._road_left_boost_gap_active)
            for _ in range(5):
                wheels = self.frame(image)
            self.assertEqual(wheels, (0., 0.))

    def test_red_approach_and_junction_never_inherit_curve_gap(self):
        for field, value in (('_red_line_visible', True),
                             ('_junction_approach_active', True),
                             ('navigation_state', 'reacquiring'),
                             ('_active_turn', 'left'),
                             ('_sharp_corner_state', 'turning')):
            self.confirm_curve()
            self.frame(self.road(missing='yellow'))
            n = self.node
            before = getattr(n, field)
            setattr(n, field, value)
            self.assertFalse(n.left_road_yellow_gap_allowed(self.now), field)
            setattr(n, field, before)

    def test_stale_image_cannot_keep_the_old_turn(self):
        self.confirm_curve()
        self.now += .6
        self.node.publish_wheels(.03, .20)
        self.assertEqual(self.node._last_wheel_speeds, (0., 0.))
        self.assertIsNone(self.node._road_left_boost_last_seen)

    def test_disabled_option_retains_the_original_short_fallback(self):
        self.confirm_curve()
        self.node.road_left_curve_boost = False
        for _ in range(6):
            wheels = self.frame(self.road(missing='yellow'))
        self.assertEqual(wheels, (0., 0.))

    def test_straight_white_and_new_unconfirmed_curve_do_not_authorize_turn(self):
        for _ in range(12):
            self.frame(self.road(bend=0))
        for _ in range(6):
            wheels = self.frame(self.road(missing='yellow'))
        self.assertEqual(wheels, (0., 0.))
        self.assertFalse(self.node._road_left_boost_active)

    def test_stop_and_pause_clear_old_curve_before_yellow_gap(self):
        for gate in ('manual_stop', 'pause'):
            self.confirm_curve()
            n = self.node
            if gate == 'manual_stop':
                n.manual_stop = True
            else:
                n.live.enabled = n.live.active = True
                n.live.paused_at = self.now
            n.publish_wheels(.03, .20)
            self.assertEqual(n._last_wheel_speeds, (0., 0.))
            n.manual_stop = False
            n.live.enabled = False
            n.live.paused_at = None
            for _ in range(6):
                wheels = self.frame(self.road(missing='yellow'))
            self.assertEqual(wheels, (0., 0.))
            self.assertFalse(n._road_left_boost_active)


if __name__ == '__main__':
    unittest.main()
