"""Steering continuity and stopping with ROS transport replaced by test doubles."""
import unittest

import numpy as np

from test_lane_follower import LaneTests


class SteeringTransitionTests(unittest.TestCase):
    subscribe = LaneTests.subscribe
    message = LaneTests.message
    deliver = LaneTests.deliver
    speeds = LaneTests.speeds
    image = LaneTests.image

    def setUp(self):
        LaneTests.setUp(self)
        self.node.drive_enabled = True
        self.node.base_speed = .09
        self.node.max_speed = .15
        self.node.max_steering = .09
        self.node.k_p = .59
        # This suite verifies the legacy single-gain transition behavior.
        # Explicitly keep the opt-in gain schedule disabled after changing k_p.
        self.node.near_center_k_p = self.node.k_p
        self.node.deadband = .05
        self.node.steering_bias = .015
        self.node.smooth_steering_deadband = True

    def settled(self, error, smooth=True, flip=True):
        self.node.smooth_steering_deadband = smooth
        self.node.flip_steering = flip
        self.node.filtered_error = error
        self.node.prev_steering = 0.0
        for _ in range(12):
            self.now += .1
            result = self.node.compute_wheel_speeds(error)
        return result

    def test_threshold_does_not_reverse_steering_for_tiny_error_change(self):
        for flip in (False, True):
            for boundary in (-.05, .05):
                below = self.settled(boundary - 1e-6, flip=flip)[2]
                above = self.settled(boundary + 1e-6, flip=flip)[2]
                self.assertLess(abs(above - below), 2e-6)
                self.assertGreater(above * below, 0)
        # Prove this probe detects the old discontinuity, including the
        # right/left reversal responsible for the latest observed oscillation.
        below = self.settled(-.050001, smooth=False)[2]
        above = self.settled(-.049999, smooth=False)[2]
        self.assertLess(below * above, 0)
        self.assertGreater(abs(above - below), .029)

    def test_response_is_bounded_monotonic_and_continuous_through_centre(self):
        errors = np.linspace(-.15, .15, 601)
        for flip in (False, True):
            results = [self.settled(float(error), flip=flip) for error in errors]
            steering = np.array([r[2] for r in results])
            changes = np.diff(steering) * (-1 if flip else 1)
            self.assertTrue(np.all(changes >= -1e-10))
            self.assertLess(np.max(np.abs(changes)), .0005)
            self.assertLessEqual(np.max(np.abs(steering)), .09)
            self.assertTrue(all(0 <= wheel <= .15 for r in results for wheel in r[:2]))

    def test_curve_authority_is_unchanged_outside_transition_region(self):
        for flip in (False, True):
            for error in (-.5, -.23, -.14, -.05, .05, .14, .23, .5):
                self.assertEqual(self.settled(error, flip=flip),
                                 self.settled(error, smooth=False, flip=flip))

    def test_small_errors_can_counter_trim_before_old_threshold(self):
        self.assertLess(self.settled(0)[2], 0)  # trim is still provisional
        self.assertGreater(self.settled(-.04)[2], 0)
        self.node.steering_bias = 0
        for error in (.001, .01, .03):
            left = self.settled(-error)[2]
            right = self.settled(error)[2]
            self.assertAlmostEqual(left, -right)
            self.assertLess(abs(right), self.node.k_p * error)
        self.node.deadband = 0
        self.assertAlmostEqual(self.settled(.01)[2], -.0059)

    def test_real_filter_and_slew_bound_jitter_and_curve_exit(self):
        self.node.alpha = .20
        previous = 0.0
        # Alternating detections straddling the old switching threshold.
        for error in [-.049, -.051] * 100 + [0.] * 100:
            self.now += 1 / 30
            left, right, steering = self.node.compute_wheel_speeds(error)
            self.assertLessEqual(abs(steering - previous), .00400001)
            self.assertTrue(0 <= left <= .15 and 0 <= right <= .15)
            previous = steering
        self.assertAlmostEqual(steering, -.015, places=6)
        self.assertEqual(self.node.compute_wheel_speeds(None), (0., 0., 0.))
        self.assertEqual((self.node.filtered_error, self.node.prev_steering), (0., 0.))

    def test_smooth_mode_obeys_red_manual_stale_shutdown_and_restart_gates(self):
        self.node.obstacle_enabled = False
        for gate in ('manual_stop', 'red_stop_latched'):
            setattr(self.node, gate, True)
            self.now += .1
            self.deliver(self.image())
            self.assertEqual(self.speeds(), (0., 0.))
            self.assertEqual(self.node.prev_steering, 0.)
            setattr(self.node, gate, False)
        self.now += .1
        self.deliver(self.image())
        self.assertGreater(max(self.speeds()), 0.)
        self.assertLessEqual(max(self.speeds()), .0150001)
        self.now += 1.
        self.node.check_camera_timeout(None)
        self.assertEqual(self.speeds(), (0., 0.))
        self.node.on_shutdown()
        self.now += .1
        self.deliver(self.image())
        self.assertEqual(self.speeds(), (0., 0.))


if __name__ == '__main__':
    unittest.main()
