"""Map, command and junction-state tests; no physical motion."""
import json
import unittest
from types import SimpleNamespace as NS
from test_lane_follower import LaneTests

class NavigationTests(LaneTests):
    def test_continuous_approach_does_not_override_ordinary_curve_before_red_seen(self):
        n = self.node
        self.configure_route(["A", "B", "C"])
        n.junctions_calibrated = True
        n.junction_straight_visual_approach = True
        n._red_line_visible = False
        n._junction_lane_geometry = {"valid": False, "steering_valid": False}
        ordinary = n.compute_wheel_speeds(-0.25)
        self.assertEqual(n.straight_approach_wheels(*ordinary), ordinary)
        n._red_line_visible = True
        self.assertNotEqual(n.straight_approach_wheels(*ordinary), ordinary)
        n._red_line_visible = False
        self.assertTrue(n._junction_approach_active)

    def test_new_confirmed_route_clears_sharp_corner_fault(self):
        n = self.node
        n.route_enabled = True
        n.drive_enabled = True
        n.navigation_state = "fault"
        n.manual_stop = True
        n._sharp_corner_state = "fault"
        result = self.command("set_route", route=["A", "B", "C"],
                              position_confirmed=True)
        self.assertTrue(result["accepted"], result)
        self.assertEqual(n._sharp_corner_state, "idle")
        self.assertEqual(n.navigation_state, "following")

    def test_route_mode_allows_sharp_bends_only_while_following(self):
        n = self.node
        n.route_enabled = True
        n.sharp_corner_enabled = True
        n.avoidance_enabled = False
        n.validate_settings()
        n.navigation_state = "red_stop"
        n._sharp_corner_state = "idle"
        n._lane_both_visible = False
        n._yellow_boundary_visible = True
        n._white_boundary_visible = False
        n._sharp_corner_last_both_time = self.now
        self.assertIsNone(n.sharp_corner_wheels(.5, False))
        self.assertEqual(n._sharp_corner_state, "idle")

    def command(self, action, **kw):
        msg = dict(id=str(len(self.node._seen_commands)+1), issued_at=self.now, action=action)
        msg.update(kw)
        self.node.command_callback(NS(data=json.dumps(msg)))
        return self.node._last_command

    def configure_route(self, route=None):
        self.node.route_enabled = True
        self.node.drive_enabled = True
        self.node.navigation_state = "awaiting_route"
        result = self.command("set_route", route=route or ["A","D","C","E","A"],
                              position_confirmed=True)
        self.assertTrue(result["accepted"], result)
        self.assertTrue(self.node.manual_stop)
        self.node._last_frame_time = self.now
        self.assertTrue(self.command("continue")["accepted"])

    def step(self, error=0.0, red=False, seconds=.1, both=True):
        self.now += seconds
        self.node._last_frame_time = self.now
        self.node._last_lane_error = error
        self.node._lane_both_visible = both
        return self.node.navigation_wheels(error, red)

    def reach_stop(self):
        self.step(red=True)
        self.assertEqual(self.node.navigation_state, "red_stop")

    def enter_crossing(self):
        self.node.junctions_calibrated = True
        self.reach_stop()
        for _ in range(22):
            self.step(red=True)
        self.assertEqual(self.node.navigation_state, "crossing")

    def test_delayed_commands_cannot_override_stop_or_target_new_junction(self):
        self.configure_route()
        old_epoch=self.node._control_epoch
        self.command("stop")
        result=self.command("continue",expected_control_epoch=old_epoch)
        self.assertFalse(result["accepted"])
        self.assertTrue(self.node.manual_stop)
        result=self.command("turn",value="right",expected_route_index=2,
                            expected_next_junction="C")
        self.assertFalse(result["accepted"])
        self.assertEqual(self.node.route,["A","D","C","E","A"])

    def test_map_turns_and_invalid_paths(self):
        m = self.mod
        self.assertEqual(m.junction_turn("A","D","C"), "straight")
        self.assertEqual(m.junction_turn("A","D","B"), "right")
        self.assertEqual(m.junction_turn("D","B","C"), "left")
        self.assertEqual(len(m.MAP_PORTS["C"]), 3)
        for bad in [[], ["A"], ["A","C"], ["A","B","A"], ["Q","A"]]:
            with self.assertRaises(ValueError):
                m.validate_route(bad)

    def test_next_turn_reconnects_route(self):
        self.configure_route()
        result = self.command("turn", value="right")
        self.assertTrue(result["accepted"], result)
        self.assertEqual(self.node.route, ["A","D","B","C","E","A"])
        original = self.node.route.copy()
        result = self.command("turn", value="left")
        self.assertFalse(result["accepted"])
        self.assertEqual(self.node.route, original)

    def test_start_position_confirmation_required(self):
        self.node.route_enabled = True
        self.node.navigation_state = "awaiting_route"
        self.assertFalse(self.command("set_route", route=["A","D"])["accepted"])
        self.assertEqual(self.node.navigation_state, "awaiting_route")
        self.node.drive_enabled = True
        self.now += .1
        self.node.publish_wheels(.1,.1)
        self.assertEqual(self.speeds(), (0,0))

    def test_route_metadata_must_match_shared_map_and_exact_approaches(self):
        self.node.route_enabled = True
        self.node.navigation_state = "awaiting_route"
        route = ["A", "B", "C"]
        valid = self.command("set_route", route=route, position_confirmed=True,
                             map_id="altata-five-junction-v1",
                             start_approach="A->B", destination_approach="B->C")
        self.assertTrue(valid["accepted"], valid)
        status = self.node.status()
        self.assertEqual(status["start_approach"], "A->B")
        self.assertEqual(status["destination_approach"], "B->C")
        self.node.navigation_state = "awaiting_route"
        for field, value in (("map_id", "wrong-map"),
                             ("start_approach", "B->A"),
                             ("destination_approach", "C->B")):
            command = dict(route=route, position_confirmed=True,
                           map_id="altata-five-junction-v1",
                           start_approach="A->B", destination_approach="B->C")
            command[field] = value
            self.assertFalse(self.command("set_route", **command)["accepted"])

    def test_stop_hold_and_calibration_gate(self):
        self.configure_route()
        self.reach_stop()
        self.node.junctions_calibrated = True
        self.assertFalse(self.command("continue")["accepted"])
        self.node.junctions_calibrated = False
        self.step(red=True, seconds=3)
        self.assertFalse(self.command("continue")["accepted"])
        self.assertEqual(self.node.navigation_state, "red_stop")

    def test_manual_stop_prevents_automatic_release(self):
        self.configure_route()
        self.node.junctions_calibrated = True
        self.reach_stop()
        self.assertTrue(self.command("stop")["accepted"])
        self.step(red=True, seconds=3)
        self.assertEqual(self.node.navigation_state, "red_stop")
        self.assertTrue(self.command("continue")["accepted"])
        self.assertEqual(self.node.navigation_state, "crossing")

    def test_route_progress_requires_outgoing_lane(self):
        self.configure_route()
        self.enter_crossing()
        for _ in range(20):
            self.step(error=None, both=False)
        self.assertEqual(self.node.route_index, 1)
        self.assertEqual(self.node.navigation_state, "reacquiring")
        for _ in range(6):
            self.step()
        self.assertEqual(self.node.navigation_state, "following")
        self.assertEqual(self.node.route_index, 2)

    def test_unmarked_reacquisition_continues_selected_profile(self):
        cases = (
            (["A", "B", "C"], "straight", "equal"),
            (["A", "B", "D"], "left", "left"),
            (["A", "B", "E"], "right", "right"),
        )
        for route, turn, expected in cases:
            with self.subTest(turn=turn):
                self.setUp()
                self.configure_route(route)
                self.enter_crossing()
                command = None
                for _ in range(30):
                    command = self.step(error=None, both=False)
                    if self.node.navigation_state == "reacquiring":
                        break
                self.assertEqual(self.node.navigation_state, "reacquiring")
                left, right, _ = command
                self.assertGreater(max(left, right), 0)
                if expected == "equal":
                    self.assertAlmostEqual(left, right)
                elif expected == "left":
                    self.assertLess(left, right)
                else:
                    self.assertGreater(left, right)
                status = self.node.status()
                self.assertEqual(status["junction_phase"], "searching")
                self.assertIsNotNone(status["junction_deadline_remaining_seconds"])

    def test_straight_approach_uses_row_geometry_after_red_becomes_visible(self):
        self.configure_route(["A", "B", "C"])
        self.node.junctions_calibrated = True
        self.node.junction_straight_visual_approach = True
        self.node.junction_straight_approach_max_steering = .01
        self.node.junction_straight_lateral_gain = .25
        self.node.junction_straight_heading_gain = .30
        self.node._junction_lane_geometry = {
            "valid": True, "lateral_error": 0.0, "heading_error": 0.0}
        self.node._red_line_visible = True
        left, right, steering = self.node.straight_approach_wheels(
            .112, .068, -.022)
        self.assertGreater(steering, -.022)
        self.assertLess(steering, 0.)
        for _ in range(3):
            self.now += .1
            left, right, steering = self.node.straight_approach_wheels(left, right, steering)
        self.assertAlmostEqual(steering, 0.0)
        self.assertAlmostEqual(left, self.node.base_speed)
        self.assertAlmostEqual(right, self.node.base_speed)
        self.node._red_line_visible = False
        self.node._junction_lane_geometry["heading_error"] = .20
        self.now += .1
        left, right, steering = self.node.straight_approach_wheels(
            .09, .09, 0.0)
        self.assertGreater(steering, 0.0)
        self.assertLess(left, right)

    def test_straight_approach_uses_two_row_control_evidence(self):
        self.configure_route(["A", "B", "C"])
        self.node.junctions_calibrated = True
        self.node.junction_straight_visual_approach = True
        self.node.junction_straight_approach_max_steering = .01
        self.node.junction_straight_lateral_gain = .25
        self.node.junction_straight_heading_gain = .30
        self.node._junction_lane_geometry = {
            "valid": False, "steering_valid": True,
            "near_support": False, "pair_count": 2,
            "lateral_error": .20, "heading_error": 0.0,
        }
        self.node._red_line_visible = True
        left, right, steering = self.node.straight_approach_wheels(
            .09, .09, 0.0)
        # With the normal flipped steering convention, a lane lying to the
        # image right requests a bounded physical right correction.
        self.assertLess(steering, 0.0)
        self.assertGreater(left, right)
        self.assertLessEqual(abs(steering), .01)

    def test_left_junction_uses_row_geometry_during_approach(self):
        self.configure_route(["A", "B", "D"])
        self.node.junctions_calibrated = True
        self.node.junction_straight_visual_approach = True
        self.node.junction_straight_approach_max_steering = .03
        self.node.junction_straight_lateral_gain = .25
        self.node.junction_straight_heading_gain = .30
        self.node._junction_lane_geometry = {
            "valid": True, "lateral_error": 0.0, "heading_error": 0.0}
        self.node._red_line_visible = True
        left, right, steering = self.node.straight_approach_wheels(
            .112, .068, -.022)
        self.assertGreater(steering, -.022)
        self.assertLess(steering, 0.)
        for _ in range(3):
            self.now += .1
            left, right, steering = self.node.straight_approach_wheels(left, right, steering)
        self.assertAlmostEqual(steering, 0.0)
        self.assertAlmostEqual(left, self.node.base_speed)
        self.assertAlmostEqual(right, self.node.base_speed)

    def test_straight_departure_holds_for_large_recorded_heading_error(self):
        self.configure_route(["A", "B", "C"])
        self.node.junctions_calibrated = True
        self.node.junction_straight_visual_approach = True
        self.node.junction_straight_departure_max_heading = .12
        self.node._junction_approach_geometry = {
            "valid": True, "lateral_error": 0.0, "heading_error": .20}
        self.node._junction_approach_geometry_time = self.now
        self.reach_stop()
        self.assertEqual(self.node._junction_stop_geometry["heading_error"], .20)
        for _ in range(25):
            wheels = self.step(red=True)
        self.assertEqual(self.node.navigation_state, "red_stop")
        self.assertEqual(wheels, (0.0, 0.0, 0.0))
        self.assertIn("heading", self.node.status()["junction_departure_blocker"])

    def test_straight_crossing_balances_cumulative_encoder_progress(self):
        self.configure_route(["A", "B", "C"])
        self.node.junction_straight_speed = .15
        self.node.junction_straight_encoder_balance = True
        self.node.junction_straight_encoder_balance_gain = .12
        self.node.junction_straight_encoder_balance_max = .015
        self.node.junction_straight_encoder_balance_min_ticks = 12
        self.node.left_encoder_callback(NS(data=100))
        self.node.right_encoder_callback(NS(data=200))
        self.enter_crossing()
        self.node.left_encoder_callback(NS(data=140))
        self.node.right_encoder_callback(NS(data=230))
        left, right, correction = self.node.planned_junction_wheels()
        self.assertAlmostEqual(correction, .015)
        self.assertAlmostEqual(left, .135)
        self.assertAlmostEqual(right, .165)
        self.assertAlmostEqual(self.node.status()["junction_encoder_adjustment"], .015)
        self.node.left_encoder_callback(NS(data=150))
        self.node.right_encoder_callback(NS(data=260))
        left, right, correction = self.node.planned_junction_wheels()
        self.assertLess(correction, 0)
        self.assertGreater(left, right)
        self.now += .31
        self.assertEqual(self.node.straight_encoder_adjustment(), 0.0)

    def test_visual_alignment_clears_and_unmarked_phase_restarts_balance(self):
        self.configure_route(["A", "B", "C"])
        self.node.junction_straight_encoder_balance = True
        self.node.junction_straight_encoder_balance_gain = .12
        self.node.junction_straight_encoder_balance_max = .015
        self.node.junction_straight_lateral_gain = .25
        self.node.junction_straight_heading_gain = .30
        self.node._active_turn = "straight"
        self.node._junction_encoder_start = (10, 20)
        self.node._junction_lane_geometry = {
            "valid": True, "lateral_error": 0.0, "heading_error": 0.0}
        self.assertIsNotNone(self.node.straight_visual_wheels(.15))
        self.assertIsNone(self.node._junction_encoder_start)
        self.node.left_encoder_callback(NS(data=100))
        self.node.right_encoder_callback(NS(data=200))
        left, right, adjustment = self.node.planned_junction_wheels()
        self.assertEqual(adjustment, 0.0)
        self.assertEqual(self.node._junction_encoder_start, (100, 200))
        self.assertAlmostEqual(left, right)

    def test_outgoing_search_steers_before_near_field_reacquisition(self):
        self.configure_route(["A", "B", "C"])
        self.node.junctions_calibrated = True
        self.node.junction_straight_visual_approach = True
        self.node.junction_straight_speed = .15
        self.node.junction_straight_lateral_gain = .25
        self.node.junction_straight_heading_gain = .30
        self.enter_crossing()
        self.node.navigation_state = "reacquiring"
        self.node._crossing_progress = (
            self.node.junction_entry_seconds + self.node.junction_straight_seconds)
        self.node._reacquire_started = self.now
        self.node._departed_red = True
        self.node._junction_lane_geometry = {
            "valid": False, "steering_valid": True,
            "near_support": False, "pair_count": 2,
            "lateral_error": .45, "heading_error": .08,
        }
        original_index = self.node.route_index
        left, right, steering = self.node.navigation_wheels(None, False)
        self.assertLess(steering, 0.0)
        self.assertGreater(left, right)
        self.assertEqual(self.node.route_index, original_index)
        self.assertEqual(self.node._junction_phase, "searching")

    def test_straight_departure_has_no_fixed_bias_before_encoder_evidence(self):
        self.configure_route(["A", "B", "C"])
        self.node.junction_straight_speed = .15
        self.node.junction_straight_encoder_balance = False
        self.node._active_turn = "straight"
        left, right, correction = self.node.planned_junction_wheels()
        self.assertAlmostEqual(correction, 0.0)
        self.assertAlmostEqual(left, .15)
        self.assertAlmostEqual(right, .15)

        self.node.junction_straight_lateral_gain = .25
        self.node.junction_straight_heading_gain = .30
        self.node._junction_lane_geometry = {
            "valid": True, "lateral_error": 0.0, "heading_error": 0.0}
        left, right, steering = self.node.straight_visual_wheels(.15)
        self.assertAlmostEqual(steering, 0.0)
        self.assertAlmostEqual(left, .15)
        self.assertAlmostEqual(right, .15)

    def test_outgoing_alignment_preserves_active_wheel_floor(self):
        self.configure_route(["A", "B", "C"])
        self.node.min_active_wheel_speed = .03
        self.node.junction_straight_speed = .09
        self.enter_crossing()
        self.node._crossing_progress = (
            self.node.junction_entry_seconds + self.node.junction_straight_seconds)
        left, right, _ = self.step(error=.5, both=True)
        self.assertGreaterEqual(left, .03)
        self.assertGreaterEqual(right, .03)
        self.assertAlmostEqual(max(left, right), .09)

    def prepare_guarded_left_search(self):
        self.configure_route(["A", "B", "D"])
        self.node.junction_straight_visual_approach = True
        self.node.junction_left_speed = .09
        self.node.junction_left_bias = .06
        self.node.min_active_wheel_speed = .03
        self.node.junction_reacquire_timeout = 5.0
        self.enter_crossing()
        while self.node.navigation_state != "reacquiring":
            self.step(error=None, both=False)

    def test_left_recorded_false_corridor_cannot_reverse_turn_or_complete(self):
        self.prepare_guarded_left_search()
        # Failed run: centroid error .20 and both colours visible, but the
        # matched-row detector found no corridor (occasionally one far pair).
        for count in (0, 1, 0, 1, 0, 0, 1, 0):
            self.node._junction_lane_geometry = {
                "valid": False, "steering_valid": False,
                "near_support": False, "pair_count": count}
            left, right, _ = self.step(error=.20, both=True)
            self.assertAlmostEqual(left, .03)
            self.assertAlmostEqual(right, .15)
            self.assertEqual(self.node.route_index, 1)
            self.assertEqual(self.node._junction_phase, "searching")
        self.node._reacquire_started = self.now - 5.0
        self.assertEqual(self.step(error=.20), (0.0, 0.0, 0.0))
        self.assertEqual(self.node.navigation_state, "fault")

    def test_stronger_left_arc_preserves_entry_and_deadline(self):
        self.prepare_guarded_left_search()
        self.node.base_speed = .09
        self.node.max_speed = .20
        self.node.junction_left_speed = .105
        self.node.junction_left_bias = .075
        left, right, _ = self.node.planned_junction_wheels(apply_turn=False)
        self.assertAlmostEqual(left, .09)
        self.assertAlmostEqual(right, .09)
        for _ in range(10):
            left, right, steering = self.step(error=None, both=False)
            self.assertAlmostEqual(left, .03)
            self.assertAlmostEqual(right, .18)
            self.assertAlmostEqual(steering, .075)
            self.assertEqual(self.node.route_index, 1)
        self.node._junction_lane_geometry = {
            "valid": True, "near_support": True, "steering_valid": True,
            "lateral_error": .3, "heading_error": .0}
        left, right, _ = self.step(error=.2)
        self.assertAlmostEqual(max(left, right), .18)
        self.assertGreaterEqual(min(left, right), .03)
        self.node._reacquire_started = self.now - 5.0
        self.assertEqual(self.step(error=None, both=False), (0.0, 0.0, 0.0))
        self.assertEqual(self.node.navigation_state, "fault")

    def test_left_rejects_distant_or_diagonal_corridor(self):
        self.prepare_guarded_left_search()
        for near, heading in ((False, 0.0), (True, .4), (True, -.4)):
            self.node._junction_lane_geometry = {
                "valid": True, "near_support": near,
                "lateral_error": 0.0, "heading_error": heading}
            left, right, _ = self.step(error=.0)
            self.assertAlmostEqual(left, .03)
            self.assertAlmostEqual(right, .15)
            self.assertEqual(self.node.route_index, 1)

    def test_left_near_corridor_alignment_and_interrupted_stability(self):
        self.prepare_guarded_left_search()
        good = {"valid": True, "steering_valid": True, "near_support": True,
                "lateral_error": 0.0, "heading_error": 0.0}
        self.node._junction_lane_geometry = dict(good, lateral_error=.3)
        left, right, _ = self.step(error=.20)
        self.assertAlmostEqual(max(left, right), .15)
        self.assertGreaterEqual(min(left, right), .03)
        self.assertEqual(self.node.route_index, 1)
        self.node._junction_lane_geometry = good.copy()
        self.step()
        self.step()
        self.node._junction_lane_geometry = None
        left, right, _ = self.step(error=None, both=False)
        self.assertAlmostEqual(left, .03)
        self.assertAlmostEqual(right, .15)
        self.assertIsNone(self.node._lane_good_since)
        self.node._junction_lane_geometry = good.copy()
        for _ in range(8):
            if self.node.navigation_state == "following":
                break
            self.step()
        self.assertEqual(self.node.navigation_state, "following")
        self.assertEqual(self.node.route_index, 2)
        self.assertEqual(self.step(error=None, both=False), (0.0, 0.0, 0.0))

    def prepare_guarded_right_search(self):
        self.configure_route(["A", "B", "E"])
        self.node.junction_straight_visual_approach = True
        self.node.max_speed = .20
        self.node.junction_right_speed = .10
        self.node.junction_right_bias = .10
        self.node.junction_reacquire_timeout = 5.0
        self.node._white_boundary_visible = True
        self.enter_crossing()
        self.node._white_boundary_visible = False
        while self.node.navigation_state != "reacquiring":
            self.step(error=None, both=False)

    def test_right_entry_waits_for_white_end_then_full_clearance(self):
        self.configure_route(["A", "B", "E"])
        self.node.junction_straight_visual_approach = True
        self.node.base_speed = .09
        self.node.max_speed = .20
        self.node.junction_right_speed = self.node.junction_right_bias = .10
        self.node._white_boundary_visible = True
        self.enter_crossing()
        for _ in range(10):
            left, right, _ = self.step()
            self.assertAlmostEqual(left, .09)
            self.assertAlmostEqual(right, .09)
        self.node._white_boundary_visible = False
        self.step()
        self.node._white_boundary_visible = True
        self.step()
        self.assertIsNone(self.node._junction_right_absent_since)
        self.node._white_boundary_visible = False
        while self.node._junction_right_advance_started is None:
            self.step(error=None, both=False)
        started = self.node._junction_right_advance_started
        while self.now - started < .15:
            left, right, _ = self.step(error=None, both=False)
            self.assertAlmostEqual(left, right)
            self.assertEqual(self.node._junction_phase, "entry")
        while self.node._junction_right_turn_origin is None:
            left, right, _ = self.step(error=None, both=False)
        self.assertGreaterEqual(self.now - started, .25)
        self.assertLess(self.now - started, .40)
        self.assertAlmostEqual(left, .20)
        self.assertAlmostEqual(right, 0.0)
        self.assertEqual(self.node._junction_phase, "turning")
        self.assertIsNone(self.node._reacquire_started)

    def test_right_entry_without_observed_white_stops_on_deadline(self):
        self.configure_route(["A", "B", "E"])
        self.node.junction_straight_visual_approach = True
        self.node._white_boundary_visible = False
        self.enter_crossing()
        for _ in range(15):
            left, right, _ = self.step(error=None, both=False)
            self.assertAlmostEqual(left, right)
            self.assertIsNone(self.node._junction_right_turn_origin)
        self.node._junction_deadline_at = self.now
        self.assertEqual(self.step(error=None, both=False), (0.0, 0.0, 0.0))
        self.assertEqual(self.node.navigation_state, "fault")

    def test_right_rejects_false_corridor_and_retains_bounded_pivot(self):
        self.prepare_guarded_right_search()
        for geometry in (
                {"valid": False, "steering_valid": False, "near_support": False},
                {"valid": True, "steering_valid": True, "near_support": False,
                 "lateral_error": 0.0, "heading_error": 0.0},
                {"valid": True, "steering_valid": True, "near_support": True,
                 "lateral_error": 0.0, "heading_error": .4}):
            self.node._junction_lane_geometry = geometry
            left, right, _ = self.step(error=.0, both=True)
            self.assertAlmostEqual(left, .20)
            self.assertAlmostEqual(right, 0.0)
            self.assertEqual(self.node.route_index, 1)
            self.assertEqual(self.node._junction_phase, "searching")
        self.node._reacquire_started = self.now - 5.0
        self.assertEqual(self.step(error=None, both=False), (0.0, 0.0, 0.0))
        self.assertEqual(self.node.navigation_state, "fault")

    def test_right_requires_stable_near_outgoing_lane_before_completion(self):
        self.prepare_guarded_right_search()
        good = {"valid": True, "steering_valid": True, "near_support": True,
                "lateral_error": 0.0, "heading_error": 0.0,
                "pair_count": 3, "row_span": .54,
                "pairs": [{"row_fraction": .18}, {"row_fraction": .54},
                          {"row_fraction": .72}]}
        self.node._junction_lane_geometry = good.copy()
        for i in range(3):
            left, right, _ = self.step(error=.0)
            if i > 0:
                self.assertAlmostEqual(left, right)
            self.assertEqual(self.node.route_index, 1)
        self.node._junction_lane_geometry = dict(good, valid=False, near_support=False)
        left, right, _ = self.step(error=.0, both=True)
        self.assertGreater(right, 0.0)
        self.assertIsNone(self.node._lane_good_since)
        self.node._junction_lane_geometry = good.copy()
        for _ in range(8):
            if self.node.navigation_state == "following":
                break
            self.step(error=.0)
        self.assertEqual(self.node.navigation_state, "following")
        self.assertEqual(self.node.route_index, 2)

    def test_right_visible_midfield_corridor_ends_pivot_without_completing_route(self):
        self.prepare_guarded_right_search()
        self.node.junction_straight_lateral_gain = .25
        self.node.junction_straight_heading_gain = .30
        # Recorded centered view: three paired rows, no bottom-row support.
        self.node._junction_lane_geometry = {
            "valid": False, "steering_valid": True, "near_support": False,
            "pair_count": 3, "row_span": .36,
            "pairs": [{"row_fraction": .18}, {"row_fraction": .36},
                      {"row_fraction": .54}],
            "lateral_error": .016354, "heading_error": -.098438}
        self.step(error=.02)
        left, right, steering = self.step(error=.02)
        self.assertTrue(self.node._junction_right_visual_entry)
        self.assertGreater(left, 0)
        self.assertGreater(right, 0)
        self.assertLessEqual(max(left, right), .20)
        self.assertAlmostEqual(steering, (right - left) / 2)
        self.assertEqual(self.node.route_index, 1)
        self.assertEqual(self.node._junction_phase, "aligning")
        # Losing a near row must not restart the pivot; complete loss stops.
        self.node._junction_lane_geometry = None
        self.assertEqual(self.step(error=None, both=False), (0, 0, 0))
        self.assertEqual(self.node.navigation_state, "fault")

    def test_right_corridor_is_checked_before_timed_pivot_finishes(self):
        self.prepare_guarded_right_search()
        self.node.navigation_state = "crossing"
        self.node.junction_right_seconds = 1.6
        self.node._crossing_progress = self.node._junction_right_turn_origin + 1.0
        # Two successive recorded views at 7.71/7.81 s in the failed run.
        for lateral, heading in ((.110104, -.125), (.070781, -.110938)):
            self.node._junction_lane_geometry = {
                "valid": False, "steering_valid": True, "near_support": False,
                "pair_count": 3, "row_span": .36,
                "pairs": [{"row_fraction": .18}, {"row_fraction": .36},
                          {"row_fraction": .54}],
                "lateral_error": lateral, "heading_error": heading}
            left, right, _ = self.step(error=.0)
        self.assertTrue(self.node._junction_right_visual_entry)
        self.assertEqual(self.node.navigation_state, "reacquiring")
        self.assertGreater(right, 0.0)
        self.assertLessEqual(max(left, right), .20)
        self.assertLess(self.node._crossing_progress - self.node._junction_right_turn_origin, 1.6)
        self.assertEqual(self.node.route_index, 1)
        # The next callback must stay in visual control despite the timer.
        self.assertGreater(self.step(error=.0)[1], 0.0)
        self.node._junction_deadline_at = self.now
        self.assertEqual(self.step(), (0.0, 0.0, 0.0))

    def test_right_tracking_trim_is_small_and_excludes_pivot_and_stops(self):
        self.prepare_guarded_right_search()
        self.node.min_active_wheel_speed = .03
        self.node.junction_right_tracking_trim = .003
        pair = self.node.right_junction_tracking_wheels(.100, .080, -.010)
        self.assertAlmostEqual(pair[0], .097)
        self.assertAlmostEqual(pair[1], .083)
        self.assertAlmostEqual(pair[2], -.007)
        self.assertEqual(self.node.right_junction_tracking_wheels(.20, 0, -.10),
                         (.20, 0, -.10))
        self.assertEqual(self.node.right_junction_tracking_wheels(0, 0, 0), (0, 0, 0))
        left, right, _ = self.node.right_junction_tracking_wheels(.03, .20, .085)
        self.assertEqual((left, right), (.03, .20))
        for turn in ("straight", "left"):
            self.node._active_turn = turn
            self.assertEqual(self.node.right_junction_tracking_wheels(.1, .08, -.01),
                             (.1, .08, -.01))

    def test_right_tracking_trim_applies_to_forward_entry(self):
        self.prepare_guarded_right_search()
        self.node.navigation_state = "crossing"
        self.node._junction_right_turn_origin = None
        self.node._white_boundary_visible = True
        self.node.junction_right_tracking_trim = .003
        self.node.base_speed = .09
        left, right, _ = self.step()
        self.assertAlmostEqual(left, .087)
        self.assertAlmostEqual(right, .093)

    def test_right_visual_alignment_preserves_full_requested_differential(self):
        self.prepare_guarded_right_search()
        n = self.node
        n._junction_right_visual_entry = True
        n.junction_right_encoder_assist = False
        n.junction_right_tracking_trim = 0.0
        n.min_active_wheel_speed = .03
        n.junction_straight_lateral_gain = .25
        n.junction_straight_heading_gain = .30
        n.flip_steering = True
        # Recorded alignment geometry: correct right steering was weakened by
        # treating the outer-wheel cap as centre speed before clipping.
        for lateral, heading in ((.077, -.112), (.068, -.218), (.399, -.034),
                                 (-.077, .112), (0.0, 0.0)):
            n._junction_lane_geometry = {
                "valid": True, "steering_valid": True,
                "lateral_error": lateral, "heading_error": heading}
            requested = n.straight_visual_steering(n._junction_lane_geometry)
            expected = max(-.085, min(.085, requested))
            left, right, actual = n.right_junction_visual_entry(self.now)
            self.assertAlmostEqual(max(left, right), .20)
            self.assertGreaterEqual(min(left, right), .03 - 1e-9)
            self.assertAlmostEqual(actual, expected)
            self.assertAlmostEqual(right - left, 2 * expected)
        # No global sign override: the other existing flip setting mirrors it.
        n.flip_steering = False
        n._junction_lane_geometry.update(lateral_error=.077, heading_error=-.112)
        left, right, actual = n.right_junction_visual_entry(self.now)
        self.assertGreater(right, left)
        self.assertAlmostEqual(actual, .25 * .077 + .30 * .112)

    def test_right_visual_ramp_preserves_ratio_and_stops_immediately(self):
        self.prepare_guarded_right_search()
        self.node._junction_right_visual_entry = True
        self.node._camera_valid = True
        self.node.red_stop_latched = self.node.manual_stop = False
        self.node._last_wheel_speeds = (.20, 0.0)
        self.node._last_publish_time = self.now
        for _ in range(12):
            previous = self.node._last_wheel_speeds
            self.now += .1
            self.node._last_frame_time = self.now
            self.node.publish_wheels(.16, .20)
            left, right = self.speeds()
            self.assertAlmostEqual(left / right, .8)
            self.assertLessEqual(left - previous[0], .015 + 1e-8)
            self.assertLessEqual(right - previous[1], .015 + 1e-8)
        self.node.manual_stop = True
        self.node.publish_wheels(.16, .20)
        self.assertEqual(self.speeds(), (0.0, 0.0))

    def test_right_encoder_assist_is_bounded_directional_and_resets(self):
        self.prepare_guarded_right_search()
        n = self.node
        n.junction_right_encoder_assist = True
        n._junction_right_visual_entry = True
        n.min_active_wheel_speed = .03
        n._junction_lane_geometry = {"steering_valid": True}
        def sample(left_ticks, right_ticks, left=.20, right=.12):
            n._left_encoder_tick, n._right_encoder_tick = left_ticks, right_ticks
            n._left_encoder_time = n._right_encoder_time = self.now
            return n.right_alignment_encoder_wheels(left, right)
        self.assertEqual(sample(100, 100), (.20, .12))
        self.now += .3
        left, right = sample(133, 131)  # near-equal measured rotation
        self.assertEqual(left, .20)
        self.assertAlmostEqual(right, .105)
        self.assertEqual(sample(133, 131, .12, .20), (.12, .20))
        self.now += .3
        left, right = sample(164, 164, .12, .20)
        self.assertAlmostEqual(left, .105)
        self.assertEqual(right, .20)
        self.assertEqual(sample(1, 1, .12, .20), (.12, .20))  # counter reset
        self.now += .31
        self.assertEqual(n.right_alignment_encoder_wheels(.12, .20), (.12, .20))
        self.assertIsNone(n._right_alignment_reference)
        sample(10, 10)
        self.now += .3
        self.assertEqual(sample(60, 40), (.20, .12))  # correction already sufficient
        self.assertEqual(sample(60, 40, .15, .15), (.15, .15))
        self.assertIsNone(n._right_alignment_reference)
        sample(60, 40)
        n.reset_steering()
        self.assertIsNone(n._right_alignment_reference)

    def test_right_encoder_assist_excludes_pivot_other_turns_and_stalls(self):
        self.prepare_guarded_right_search()
        n = self.node
        n.junction_right_encoder_assist = True
        n._junction_right_visual_entry = True
        n._junction_lane_geometry = {"steering_valid": True}
        n._left_encoder_tick = n._right_encoder_tick = 100
        n._left_encoder_time = n._right_encoder_time = self.now
        self.assertEqual(n.right_alignment_encoder_wheels(.20, 0), (.20, 0))
        n._active_turn = "left"
        self.assertEqual(n.right_alignment_encoder_wheels(.20, .12), (.20, .12))
        n._active_turn = "right"
        n.right_alignment_encoder_wheels(.20, .12)
        self.now += .3
        n._left_encoder_time = n._right_encoder_time = self.now
        self.assertEqual(n.right_alignment_encoder_wheels(.20, .12), (.20, .12))
        self.assertEqual(n._right_alignment_adjustment, 0)

    def test_next_red_completes_stable_right_outgoing_corridor(self):
        self.prepare_guarded_right_search()
        self.node._junction_right_visual_entry = True
        self.node._junction_lane_geometry = {
            "valid": True, "steering_valid": True, "near_support": True,
            "lateral_error": .15, "heading_error": -.1}
        for _ in range(5):
            self.step(error=.15, red=False)
        self.assertTrue(self.node._junction_right_red_rearmed)
        self.assertEqual(self.step(error=.15, red=True), (0, 0, 0))
        self.assertTrue(self.node.red_stop_latched)
        self.assertEqual(self.node.navigation_state, "route_complete")
        self.assertEqual(self.node.route_index, 2)
        self.assertEqual(self.node._last_junction_result["outcome"],
                         "reacquired_at_next_red")

    def test_next_red_advances_once_and_waits_at_intermediate_junction(self):
        self.prepare_guarded_right_search()
        self.node.route = ["A", "B", "E", "C"]
        self.node._junction_right_visual_entry = True
        self.node._junction_lane_geometry = {
            "valid": True, "steering_valid": True, "near_support": True,
            "lateral_error": .12, "heading_error": -.07}
        for _ in range(5):
            self.step(error=.12, red=False)
        self.assertTrue(self.node._junction_right_red_rearmed)
        self.assertEqual(self.step(error=.12, red=True), (0, 0, 0))
        self.assertEqual(self.node.navigation_state, "red_stop")
        self.assertEqual(self.node.route_index, 2)
        self.assertIsNone(self.node._active_turn)
        # Holding the same image cannot advance a second edge.
        self.step(error=.12, red=True)
        self.assertEqual(self.node.route_index, 2)

    def test_right_red_rearm_requires_clear_stable_outgoing_view(self):
        self.prepare_guarded_right_search()
        self.node._junction_right_visual_entry = True
        geometry = {"valid": True, "steering_valid": True, "near_support": True,
                    "lateral_error": .15, "heading_error": -.1}
        self.node._junction_lane_geometry = geometry
        for _ in range(5):
            self.step(error=.15, red=True)
        self.assertFalse(self.node._junction_right_red_rearmed)
        for _ in range(3):
            self.node._junction_lane_geometry = geometry
            self.step(error=.15, red=False)
            self.node._junction_lane_geometry = dict(geometry, valid=False)
            self.step(error=.15, red=False)
        self.assertFalse(self.node._junction_right_red_rearmed)

    def test_right_early_pivot_does_not_accept_departure_lane(self):
        self.prepare_guarded_right_search()
        self.node.navigation_state = "crossing"
        self.node._crossing_progress = self.node._junction_right_turn_origin
        self.node._departed_red = False
        self.node._junction_lane_geometry = {
            "valid": True, "steering_valid": True, "near_support": True,
            "pair_count": 3, "row_span": .54,
            "pairs": [{"row_fraction": .18}, {"row_fraction": .54},
                      {"row_fraction": .72}],
            "lateral_error": 0.0, "heading_error": 0.0}
        for _ in range(5):
            self.assertEqual(self.step(red=True)[1], 0.0)
            self.assertFalse(self.node._junction_right_visual_entry)

    def test_right_corridor_flicker_does_not_latch_visual_entry(self):
        self.prepare_guarded_right_search()
        candidate = {"valid": False, "steering_valid": True, "near_support": False,
                     "pair_count": 3, "row_span": .36,
                     "pairs": [{"row_fraction": .18}, {"row_fraction": .36},
                               {"row_fraction": .54}],
                     "lateral_error": 0.0, "heading_error": 0.0}
        for _ in range(3):
            self.node._junction_lane_geometry = candidate
            self.step(error=0)
            self.node._junction_lane_geometry = None
            self.step(error=None, both=False)
        self.assertFalse(self.node._junction_right_visual_entry)
        self.assertEqual(self.node.route_index, 1)

    def test_left_outgoing_alignment_keeps_proven_outer_wheel_cap(self):
        self.configure_route(["A", "B", "D"])
        self.node.junction_left_speed = .09
        self.node.junction_left_bias = .06
        self.node.min_active_wheel_speed = .03
        self.enter_crossing()
        self.node._crossing_progress = (
            self.node.junction_entry_seconds + self.node.junction_left_seconds)
        left, right, _ = self.step(error=.5, both=True)
        self.assertGreaterEqual(left, .03)
        self.assertGreaterEqual(right, .03)
        self.assertAlmostEqual(max(left, right), .15)

    def test_left_crossing_can_reacquire_after_previous_three_second_limit(self):
        self.configure_route(["A", "B", "D"])
        self.node.junction_reacquire_timeout = 5.0
        self.enter_crossing()
        while self.node.navigation_state != "reacquiring":
            self.step(error=None, both=False)
        for _ in range(35):
            self.step(error=None, both=False)
        self.assertEqual(self.node.navigation_state, "reacquiring")
        for _ in range(5):
            self.step(error=0.0, both=True)
        self.assertEqual(self.node.navigation_state, "following")
        self.assertEqual(self.node.route_index, 2)

    def test_outgoing_lane_must_be_stable_before_route_advances(self):
        self.configure_route(["A", "B", "C"])
        self.enter_crossing()
        while self.node.navigation_state != "reacquiring":
            self.step(error=None, both=False)
        original_index = self.node.route_index
        for _ in range(2):
            self.step(error=.1, both=True)
        self.assertEqual(self.node.route_index, original_index)
        self.step(error=None, both=False)
        for _ in range(5):
            self.step(error=.1, both=True)
        self.assertEqual(self.node.navigation_state, "following")
        self.assertEqual(self.node.route_index, original_index + 1)
        self.assertEqual(self.node._last_junction_result["outcome"], "reacquired")

    def test_straight_outgoing_requires_multirow_alignment_and_settling(self):
        self.configure_route(["A", "B", "C"])
        self.node.junctions_calibrated = True
        self.node.junction_straight_visual_approach = True
        self.node.junction_straight_lateral_gain = .25
        self.node.junction_straight_heading_gain = .30
        self.node.junction_straight_reacquire_seconds = .5
        self.node.junction_straight_settle_seconds = .6
        self.enter_crossing()
        while self.node.navigation_state != "reacquiring":
            self.node._junction_lane_geometry = {
                "valid": False, "near_support": False, "pair_count": 1}
            self.step(error=None, both=False)
        original_index = self.node.route_index
        for _ in range(8):
            self.node._junction_lane_geometry = {
                "valid": True, "near_support": True, "pair_count": 4,
                "lateral_error": .02, "heading_error": .14}
            self.step(error=.02, both=True)
        self.assertEqual(self.node.route_index, original_index)
        for _ in range(7):
            self.node._junction_lane_geometry = {
                "valid": True, "near_support": True, "pair_count": 4,
                "lateral_error": .05, "heading_error": .04}
            self.step(error=.05, both=True)
        self.assertEqual(self.node.navigation_state, "following")
        self.assertEqual(self.node._junction_phase, "settling")
        self.assertFalse(self.node._junction_settled)
        for _ in range(8):
            self.node._junction_lane_geometry = {
                "valid": True, "near_support": True, "pair_count": 5,
                "lateral_error": .01, "heading_error": .01}
            self.step(error=.01, both=True)
        self.assertTrue(self.node._junction_settled)
        self.assertEqual(self.node._junction_phase, "complete")
        self.assertEqual(self.node._last_junction_result["alignment"], "settled")

    def test_normal_lane_loss_still_requests_zero(self):
        self.configure_route(["A", "B", "C"])
        self.assertEqual(self.node.navigation_state, "following")
        self.assertEqual(self.step(error=None, both=False), (0.0, 0.0, 0.0))

    def test_extended_straight_search_reacquires_after_old_timeout(self):
        self.configure_route(["A", "B", "C"])
        self.node.junction_reacquire_timeout = 7.0
        self.node.junction_reacquire_max_error = .10
        self.enter_crossing()
        # Entirely unmarked through entry and over four seconds of search.
        for _ in range(60):
            left, right, _ = self.step(error=None, both=False)
            self.assertGreater(left, 0)
            self.assertAlmostEqual(left, right)
            self.assertEqual(self.node.route_index, 1)
        self.assertEqual(self.node.navigation_state, "reacquiring")
        status = self.node.status()
        self.assertTrue(status["junction_departed_red"])
        self.assertIn("ordered", status["junction_reacquisition_blocker"])
        self.assertGreater(status["junction_search_remaining_seconds"], 0)
        # One boundary or a poorly aligned pair must not advance the route.
        for error, both in ((.1, False), (.58, True), (.17, True)):
            for _ in range(4):
                self.node._last_lane_error = error
                self.step(error=error, both=both)
                self.assertEqual(self.node.route_index, 1)
        self.assertEqual(self.node.status()["junction_reacquisition_blocker"],
                         "Outgoing lane needs alignment")
        for _ in range(5):
            self.node._last_lane_error = .08
            self.step(error=.08, both=True)
        self.assertEqual(self.node.navigation_state, "following")
        self.assertEqual(self.node.route_index, 2)
        self.assertIsNone(self.node.status()["junction_search_remaining_seconds"])
        self.assertEqual(self.step(error=None, both=False), (0.0, 0.0, 0.0))

    def test_extended_straight_search_still_expires(self):
        self.configure_route(["A", "B", "C"])
        self.node.junction_reacquire_timeout = 7.0
        self.enter_crossing()
        for _ in range(95):
            command = self.step(error=None, both=False)
            if self.node.navigation_state == "fault":
                break
        self.assertEqual(self.node.navigation_state, "fault")
        self.assertEqual(command, (0.0, 0.0, 0.0))
        self.assertEqual(self.node.route_index, 1)
        self.assertIn("not reacquired", self.node._fault_reason)

    def test_invalid_t_junction_exit_is_rejected(self):
        self.configure_route(["A", "D", "C"])
        original = list(self.node.route)
        result = self.command("turn", value="left")
        self.assertFalse(result["accepted"])
        self.assertEqual(self.node.route, original)

    def test_junction_deadline_stops_unmarked_crossing(self):
        self.configure_route(["A", "B", "C"])
        self.enter_crossing()
        self.now = self.node._junction_deadline_at
        self.node._last_frame_time = self.now
        self.node._lane_both_visible = False
        command = self.node.navigation_wheels(None, False)
        self.assertEqual(command, (0.0, 0.0, 0.0))
        self.assertEqual(self.node.navigation_state, "fault")
        self.assertIn("time limit", self.node._fault_reason)

    def test_red_never_clears_cannot_finish_junction(self):
        self.configure_route()
        self.enter_crossing()
        for _ in range(55):
            self.step(red=True)
        self.assertEqual(self.node.navigation_state, "fault")
        self.assertEqual(self.node.route_index, 1)

    def test_outgoing_lane_failure_faults(self):
        self.configure_route()
        self.enter_crossing()
        for _ in range(55):
            self.step(error=None, both=False)
        self.assertEqual(self.node.navigation_state, "fault")
        self.assertEqual(self.speeds(), (0,0))
        self.assertFalse(self.command("continue")["accepted"])

    def test_camera_loss_mid_turn_requires_reset(self):
        self.configure_route()
        self.enter_crossing()
        self.now += .6
        self.node.check_camera_timeout(None)
        self.assertEqual(self.node.navigation_state, "fault")
        self.assertTrue(self.node.manual_stop)
        self.assertEqual(self.speeds(), (0,0))

    def test_stop_mid_turn_requires_reset(self):
        self.configure_route()
        self.enter_crossing()
        self.assertTrue(self.command("stop")["accepted"])
        self.assertEqual(self.node.navigation_state, "fault")
        self.assertFalse(self.command("continue")["accepted"])

    def test_reappearing_cross_traffic_red_does_not_cancel_crossing(self):
        self.configure_route()
        self.enter_crossing()
        for _ in range(4):
            self.step()
        self.step(red=True)
        self.assertEqual(self.node.navigation_state, "crossing")
        self.assertTrue(self.node._junction_red_reappeared)
        self.assertEqual(self.node.route_index, 1)
        self.assertFalse(self.node.red_stop_latched)

    def test_destination_stops_without_crossing(self):
        self.configure_route(["A","D"])
        self.step(red=True)
        self.assertEqual(self.node.navigation_state, "route_complete")
        self.assertFalse(self.command("continue")["accepted"])

    def test_steering_signs_and_slowing_during_crossing(self):
        self.configure_route()
        self.command("turn", value="right")
        self.enter_crossing()
        for _ in range(7):
            left,right,_ = self.step()
        self.assertGreater(left,right)
        progress = self.node._crossing_progress
        self.assertTrue(self.command("speed_scale",value=.5)["accepted"])
        left2,right2,_ = self.step()
        self.assertAlmostEqual(left2,left/2)
        self.assertAlmostEqual(right2,right/2)
        self.assertAlmostEqual(self.node._crossing_progress-progress,.05)

    def test_invalid_commands_and_deduplication(self):
        initial = self.node.speed_scale
        msg = NS(data=json.dumps(dict(id="same",action="slow_down",issued_at=self.now)))
        self.node.command_callback(msg)
        self.node.command_callback(msg)
        self.assertAlmostEqual(self.node.speed_scale, initial*.8)
        for value in [float("nan"),float("inf"),True,-1,2,"fast"]:
            self.assertFalse(self.command("speed_scale", value=value)["accepted"])
        self.assertFalse(self.command("continue", issued_at=self.now-10)["accepted"])
        self.assertTrue(self.command("stop", issued_at=self.now-100)["accepted"])
        self.assertEqual(self.speeds(), (0,0))

    def test_commands_cannot_enable_debug_or_exceed_wheel_cap(self):
        self.assertFalse(self.command("continue")["accepted"])
        self.assertTrue(self.command("speed_up")["accepted"])
        self.node.publish_wheels(100,100)
        self.assertEqual(self.speeds(), (0,0))
        self.node.drive_enabled = True
        self.node.acceleration_limit = 100
        self.now += .1
        self.node.publish_wheels(100,100)
        self.assertEqual(self.speeds(), (self.node.max_speed,self.node.max_speed))
        self.node.publish_wheels(float("nan"), .1)
        self.assertEqual(self.speeds(), (0,0))

def load_tests(loader, tests, pattern):
    # Run only the new cases here; the shared lane suite is discovered separately.
    names = [name for name in NavigationTests.__dict__ if name.startswith("test_")]
    return unittest.TestSuite(NavigationTests(name) for name in names)

if __name__ == "__main__":
    unittest.main()
