"""Straight intersection exits must release the ordinary road follower."""
import unittest
import cv2
import numpy as np
import test_navigation as fixtures


class StraightExitHandoffTests(unittest.TestCase):
    setUp = fixtures.NavigationTests.setUp
    subscribe = fixtures.NavigationTests.subscribe
    message = fixtures.NavigationTests.message
    command = fixtures.NavigationTests.command
    configure_route = fixtures.NavigationTests.configure_route
    step = fixtures.NavigationTests.step
    reach_stop = fixtures.NavigationTests.reach_stop
    enter_crossing = fixtures.NavigationTests.enter_crossing

    @staticmethod
    def masks(end=.685, start=.04, reverse=False):
        yellow = np.zeros((216, 640), dtype=np.uint8)
        white = np.zeros_like(yellow)
        a, b = (470, 160) if reverse else (160, 470)
        cv2.rectangle(yellow, (a, int(start*215)), (a+9, int(end*215)), 255, -1)
        cv2.rectangle(white, (b, int(start*215)), (b+9, int(end*215)), 255, -1)
        return yellow, white

    def geometry(self, **kwargs):
        y, w = self.masks(**kwargs)
        n = self.node
        original = n.junction_lane_geometry(y, w, 640)
        return original, n.straight_exit_geometry(y, w, 640, original)

    def search(self, live=True):
        self.configure_route(["A", "E", "C"])
        n = self.node
        n.junction_straight_visual_approach = True
        n.junction_straight_lane_target_fraction = .49
        n.junction_straight_reacquire_seconds = .5
        self.enter_crossing()
        while n.navigation_state != "reacquiring":
            self.step(error=None, both=False)
        if live:
            n.live.start("exit-test")
        return n

    def test_fixed_bands_miss_but_dense_real_near_rows_establish_corridor(self):
        original, dense = self.geometry()
        self.assertFalse(original["valid"])
        self.assertTrue(dense["valid"])
        self.assertTrue(dense["near_support"])
        self.assertGreaterEqual(dense["pair_count"], 6)
        self.assertGreaterEqual(dense["row_span"], .54)
        self.assertGreaterEqual(dense["pairs"][-1]["row_fraction"], 2/3)

    def test_distant_fragments_narrow_depth_and_wrong_order_are_rejected(self):
        for kwargs in (dict(end=.40), dict(start=.60, end=.90), dict(reverse=True)):
            original, dense = self.geometry(**kwargs)
            self.assertIs(original, dense)
            self.assertFalse(dense["valid"])

    def test_dense_partial_corridor_is_retained_without_inventing_near_support(self):
        original, dense = self.geometry(end=.58)
        self.assertFalse(original["valid"])
        self.assertFalse(dense["valid"])
        self.assertFalse(dense["near_support"])
        self.assertGreaterEqual(dense["pair_count"], 6)
        n = self.search()
        n._junction_exit_geometry = dense
        for _ in range(5):
            self.step(error=-.10)
        self.assertEqual(n.navigation_state, "following")
        self.assertIsNone(n._reacquire_started)
        for _ in range(120):
            self.step(error=-.20)
        self.assertEqual(n.route_index, 2)
        self.assertEqual(n.navigation_state, "following")

    def test_original_valid_geometry_is_unchanged(self):
        original, dense = self.geometry(end=.95)
        self.assertTrue(original["valid"])
        self.assertIs(original, dense)

    def test_live_exit_handoff_is_stable_once_and_restores_curve_steering(self):
        n = self.search()
        n._junction_lane_geometry = self.geometry()[1]
        for _ in range(3):
            self.step()
            self.assertEqual(n.route_index, 1)
        for _ in range(4):
            self.step()
        self.assertEqual(n.route_index, 2)
        self.assertEqual(n.navigation_state, "following")
        self.assertEqual(n._junction_phase, "complete")
        self.assertEqual(n._last_junction_result["alignment"], "lane_following")
        self.assertIsNone(n._junction_deadline_at)
        self.assertFalse(n._junction_approach_active)
        # The dense straight fit must no longer own steering on the next curve.
        n.reset_steering()
        self.now += .1
        expected = n.compute_wheel_speeds(-.30)
        n.reset_steering()
        self.assertEqual(self.step(error=-.30), expected)
        self.assertGreater(expected[1], expected[0])
        for _ in range(100):
            self.step(error=-.30)
        self.assertEqual(n.navigation_state, "following")
        self.assertEqual(n.route_index, 2)
        self.assertEqual(self.step(error=None, both=False), (0, 0, 0))
        self.step(red=True)
        self.assertEqual(n.navigation_state, "red_stop")

    def test_map_app_handoff_matches_live_chat_handoff(self):
        n = self.search(live=False)
        n.require_client_heartbeat = True
        n._active_client_id = "map-app"
        n._heartbeat_clients["map-app"] = (1, self.now)
        n._junction_lane_geometry = self.geometry()[1]
        for _ in range(8):
            n._heartbeat_clients["map-app"] = (1, self.now)
            self.step()
        self.assertEqual(n.route_index, 2)
        self.assertEqual(n._junction_phase, "complete")

    def test_misalignment_or_flicker_does_not_bypass_acceptance(self):
        n = self.search()
        good = self.geometry()[1]
        for _ in range(6):
            n._junction_lane_geometry = dict(good, heading_error=.4)
            self.step()
        self.assertEqual(n.route_index, 1)
        for _ in range(4):
            n._junction_lane_geometry = good
            self.step()
            n._junction_lane_geometry = self.geometry(end=.40)[1]
            self.step()
        self.assertEqual(n.route_index, 1)
        n._reacquire_started = self.now - n.junction_reacquire_timeout
        self.assertEqual(self.step(), (0, 0, 0))
        self.assertEqual(n.navigation_state, "fault")

    def test_dense_sampling_only_runs_for_app_straight_exit(self):
        n = self.node
        image = np.zeros((480, 640, 3), np.uint8)
        original = n.straight_exit_geometry
        calls = []
        def observe(*args):
            calls.append(True)
            return original(*args)
        n.straight_exit_geometry = observe
        n.live.start("scope-test")
        for state, turn in (("following", None), ("reacquiring", "left"),
                            ("reacquiring", "right")):
            n.navigation_state, n._active_turn = state, turn
            n.detect_lane_bgr(image)
        self.assertEqual(calls, [])
        n.navigation_state, n._active_turn = "reacquiring", "straight"
        n.detect_lane_bgr(image)
        self.assertEqual(len(calls), 1)
        n.live.enabled = False
        n.detect_lane_bgr(image)
        self.assertEqual(len(calls), 1)


    def test_dense_exit_evidence_never_changes_visual_steering(self):
        n = self.search()
        n._junction_lane_geometry = dict(valid=False, steering_valid=True,
                                        lateral_error=.06, heading_error=-.03)
        before = n.straight_visual_wheels(.15)
        n._junction_exit_geometry = dict(valid=True, near_support=True,
                                        lateral_error=-.08, heading_error=.07)
        self.assertEqual(n.straight_visual_wheels(.15), before)
        self.assertIs(n.junction_exit_geometry(), n._junction_exit_geometry)

    def test_road_centered_exit_does_not_demand_junction_target_first(self):
        n = self.search()
        n.lane_target_fraction = .441
        n._junction_lane_geometry = dict(valid=True, steering_valid=True,
                                        near_support=True, lateral_error=-.13,
                                        heading_error=.01)
        for _ in range(8):
            self.step(error=-.02)
        self.assertEqual(n.navigation_state, "following")
        self.assertEqual(n.route_index, 2)
        self.assertEqual(n._junction_phase, "complete")

    def test_road_handoff_still_needs_heading_near_support_and_containment(self):
        n = self.search()
        n.lane_target_fraction = .441
        base = dict(valid=True, near_support=True, lateral_error=-.13, heading_error=.01)
        for geometry, error in ((dict(base, heading_error=.20), 0.),
                                (dict(base, near_support=False), 0.),
                                (base, .30), (dict(base, lateral_error=-.4), 0.)):
            n._junction_lane_geometry = geometry
            n._last_lane_error = error
            self.assertFalse(n.straight_reacquisition_good())
        n._junction_lane_geometry = base
        n._last_lane_error = 0.
        n._lane_both_visible = True
        self.assertTrue(n.straight_reacquisition_good())
        n.live.enabled = False
        self.assertFalse(n.straight_reacquisition_good())

    def test_invalid_camera_clears_dense_exit_evidence(self):
        n = self.search()
        n._junction_exit_geometry = self.geometry()[1]
        n.reject_camera("test missing frame")
        self.assertIsNone(n._junction_exit_geometry)
        self.assertEqual(n.navigation_state, "fault")


    @staticmethod
    def curve_corridor(near=False, heading=-.075, lateral=-.278):
        return dict(valid=near, steering_valid=True, near_support=near,
                    pair_count=3, row_span=.54 if near else .36,
                    lateral_error=lateral, heading_error=heading,
                    pairs=[dict(row_fraction=.18), dict(row_fraction=.36),
                           dict(row_fraction=.72 if near else .54)])

    def test_straight_instruction_releases_steering_before_curve_is_aligned(self):
        n = self.search()
        n.junction_reacquire_timeout = 9.
        n._junction_lane_geometry = self.curve_corridor()
        self.step(error=-.151)
        self.assertFalse(n._junction_straight_visual_entry)
        self.step(error=-.151, seconds=.31)
        self.assertTrue(n._junction_straight_visual_entry)
        self.assertEqual(n._junction_phase, "complete")
        self.assertEqual(n.route_index, 2)  # steering and route handoff commit together
        self.assertIsNone(n._reacquire_started)
        self.assertIsNone(n._junction_deadline_at)
        # The failed run kept the straight-fit controller until its deadline.
        # During road tracking navigation must call the ordinary wheel mixer.
        calls = []
        def road(error):
            calls.append(error)
            return (.03, .15, .06)
        n.compute_wheel_speeds = road
        n.planned_junction_wheels = lambda *a, **k: self.fail("Straight profile resumed")
        n.straight_visual_wheels = lambda *a: self.fail("Straight alignment resumed")
        self.assertEqual(self.step(error=-.15), (.03, .15, .06))
        self.assertEqual(calls, [-.15])
        # The next curve slope exceeds the old straight-heading limit.
        n._junction_lane_geometry = self.curve_corridor(True, heading=.216, lateral=-.136)
        for _ in range(7):
            self.step(error=-.084)
        self.assertEqual(n.navigation_state, "following")
        self.assertEqual(n.route_index, 2)
        self.assertIsNone(n._active_turn)
        self.assertIsNone(n._junction_deadline_at)
        self.assertEqual(n._junction_phase, "complete")

    def test_partial_dash_after_road_latch_does_not_resume_straight_profile(self):
        n = self.search()
        n._junction_lane_geometry = self.curve_corridor()
        self.step(error=-.15)
        self.step(error=-.15, seconds=.31)
        n._junction_lane_geometry = None
        n.planned_junction_wheels = lambda *a, **k: self.fail("Straight profile resumed")
        self.step(error=-.12, both=False)  # ordinary one-border tracking
        self.assertTrue(n._junction_straight_visual_entry)
        self.assertEqual(n.route_index, 2)
        self.assertEqual(self.step(error=None, both=False), (0,0,0))

    def test_false_or_flickering_corridor_never_latches_road_tracking(self):
        n = self.search()
        n.junction_reacquire_timeout = 9.
        for g in (dict(self.curve_corridor(), pair_count=1),
                  self.curve_corridor(heading=.6), self.curve_corridor(lateral=.7)):
            n._junction_lane_geometry = g
            for _ in range(3):
                self.step(error=-.1)
                self.assertFalse(n._junction_straight_visual_entry)
        n._junction_lane_geometry = self.curve_corridor()
        self.step(error=-.1)
        n._junction_lane_geometry = None
        self.step(error=None, both=False)
        n._junction_lane_geometry = self.curve_corridor()
        self.step(error=-.1)
        self.assertFalse(n._junction_straight_visual_entry)

    def test_road_tracking_does_not_latch_for_left_right_or_unmanaged_test(self):
        n = self.search(live=False)
        n._last_lane_error = -.15
        n._lane_both_visible = True
        n._junction_lane_geometry = self.curve_corridor()
        self.assertFalse(n.update_straight_road_tracking(self.now))
        n.live.start("test-scope")
        for turn in ("left", "right"):
            n._active_turn = turn
            self.assertFalse(n.update_straight_road_tracking(self.now))
            self.assertFalse(n.update_straight_road_tracking(self.now+.3))


if __name__ == "__main__":
    unittest.main()
