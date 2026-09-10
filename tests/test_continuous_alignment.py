"""Continuous-route drift and left-exit regressions, without robot access."""
import unittest
import test_navigation as fixtures


class ContinuousAlignmentTests(unittest.TestCase):
    setUp = fixtures.NavigationTests.setUp
    subscribe = fixtures.NavigationTests.subscribe
    message = fixtures.NavigationTests.message
    command = fixtures.NavigationTests.command
    configure_route = fixtures.NavigationTests.configure_route
    step = fixtures.NavigationTests.step
    reach_stop = fixtures.NavigationTests.reach_stop
    enter_crossing = fixtures.NavigationTests.enter_crossing
    prepare_guarded_left_search = fixtures.NavigationTests.prepare_guarded_left_search

    @staticmethod
    def corridor(lateral=-.22648, heading=-.02578, near=False):
        return dict(valid=near, steering_valid=True, near_support=near,
                    pair_count=3, row_span=.36,
                    pairs=[{'row_fraction':.18}, {'row_fraction':.36},
                           {'row_fraction':.54}],
                    lateral_error=lateral, heading_error=heading)

    def left_search(self):
        self.prepare_guarded_left_search()
        n = self.node
        n.junction_left_visual_latch = True
        n.max_speed = .20
        n.junction_left_speed = .105
        n.junction_left_bias = .075
        n.junction_straight_lateral_gain = .25
        n.junction_straight_heading_gain = .30
        return n

    def latch_left(self):
        n = self.left_search()
        n._junction_lane_geometry = self.corridor()
        self.step(error=-.07)
        wheels = self.step(error=-.07, seconds=.11)
        self.assertTrue(n._junction_left_visual_entry)
        self.assertEqual(n.route_index, 1)
        self.assertGreater(wheels[0], .03)
        self.assertAlmostEqual(wheels[1], .18)
        return n

    def test_left_latches_midfield_before_near_dashes_then_corrects_heading(self):
        n = self.latch_left()
        # The failed recording returned to its full left arc here instead.
        n._junction_lane_geometry = self.corridor(.15, -.14625, near=True)
        left, right, _ = self.step(error=.19)
        self.assertGreater(left, right)
        self.assertEqual(n._junction_phase, 'aligning')
        self.assertEqual(n.route_index, 1)
        self.assertGreaterEqual(right, .03)
        self.assertLessEqual(left, .18)

    def test_left_partial_rows_after_latch_do_not_restart_arc(self):
        n = self.latch_left()
        n._junction_lane_geometry = dict(self.corridor(.02, .02), pair_count=2, row_span=.18)
        left, right, _ = self.step()
        self.assertLess(abs(left - right), .01)
        self.assertTrue(n._junction_left_visual_entry)

    def test_left_missing_corridor_after_latch_stops(self):
        n = self.latch_left()
        n._junction_lane_geometry = None
        self.assertEqual(self.step(error=None, both=False), (0, 0, 0))
        self.assertEqual(n.navigation_state, 'fault')

    def test_left_false_fragments_and_diagonal_lane_never_latch(self):
        n = self.left_search()
        for g in (None, dict(self.corridor(), pair_count=1),
                  self.corridor(heading=.4), self.corridor(lateral=.5),
                  dict(self.corridor(), row_span=.18)):
            n._junction_lane_geometry = g
            for _ in range(3):
                self.assertAlmostEqual(self.step()[0], .03)
                self.assertFalse(n._junction_left_visual_entry)

    def test_left_flicker_and_departure_gate(self):
        n = self.left_search()
        n._departed_red = False
        n._junction_lane_geometry = self.corridor()
        for t in (self.now, self.now+.2):
            self.assertIsNone(n.left_junction_visual_entry(t))
        n._departed_red = True
        for _ in range(3):
            n._junction_lane_geometry = self.corridor()
            self.step()
            n._junction_lane_geometry = None
            self.step(error=None, both=False)
        self.assertFalse(n._junction_left_visual_entry)

    def test_left_completion_still_requires_near_aligned_stability(self):
        n = self.latch_left()
        n._junction_lane_geometry = self.corridor(0,0,near=False)
        for _ in range(6):
            self.step()
            self.assertEqual(n.route_index, 1)
        n._junction_lane_geometry = self.corridor(0,0,near=True)
        for _ in range(7):
            self.step()
        self.assertEqual(n.route_index, 2)
        self.assertEqual(n.navigation_state, 'following')

    def test_left_deadline_and_manual_stop_remain_active(self):
        n = self.latch_left()
        n._reacquire_started = self.now - n.junction_reacquire_timeout
        self.assertEqual(self.step(), (0,0,0))
        self.assertEqual(n.navigation_state, 'fault')

    def test_left_manual_stop_remains_zero(self):
        n = self.latch_left()
        self.command('stop')
        n.publish_wheels(.18, .03)
        self.assertEqual((self.messages[-1].vel_left, self.messages[-1].vel_right), (0,0))
        self.assertEqual(n.navigation_state, 'fault')
        self.assertTrue(n.manual_stop)

    def test_left_alignment_acceleration_preserves_requested_ratio(self):
        n = self.latch_left()
        n._camera_valid = True
        n.red_stop_latched = n.manual_stop = False
        n._last_frame_time = self.now
        n._last_wheel_speeds = (.03, .18)
        n._last_publish_time = self.now - .05
        n.publish_wheels(.18, .15)
        message = self.messages[-1]
        self.assertAlmostEqual(message.vel_left / message.vel_right, .18 / .15)
        self.assertLessEqual(message.vel_left, .03 + .05 * n.acceleration_limit)

    def road(self):
        n = self.node
        n.road_heading_guard = True
        n.junction_straight_lateral_gain = .25
        n.junction_straight_heading_gain = .30
        n.navigation_state = 'following'
        n._junction_phase = 'idle'
        n._lane_both_visible = True
        n._junction_lane_geometry = self.corridor(.01230,-.03093,near=True)
        return n

    def test_recorded_road_geometry_limits_excessive_right_request(self):
        n = self.road()
        self.assertAlmostEqual(n.guard_road_heading(.027), .012354)
        self.assertTrue(n._road_heading_guard_used)

    def test_guard_is_symmetric_and_does_not_weaken_supported_curves(self):
        n = self.road()
        n._junction_lane_geometry = self.corridor(-.01230,.03093,near=True)
        self.assertAlmostEqual(n.guard_road_heading(-.027), -.012354)
        for lateral, heading in ((-.2, .05), (.2,-.05), (0,.2)):
            n._junction_lane_geometry = self.corridor(lateral,heading,near=True)
            self.assertEqual(n.guard_road_heading(-.1), -.1)

    def test_guard_preserves_agreement_and_requires_ordinary_supported_road(self):
        n = self.road()
        self.assertEqual(n.guard_road_heading(.005), .005)
        for field,value in (('road_heading_guard',False),('_lane_both_visible',False),
                            ('_red_line_visible',True),('_junction_approach_active',True),
                            ('navigation_state','reacquiring'),('_sharp_corner_state','turning')):
            old=getattr(n,field);setattr(n,field,value)
            self.assertEqual(n.guard_road_heading(.027), .027)
            setattr(n,field,old)


if __name__ == '__main__':
    unittest.main()
