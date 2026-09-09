"""Map, command and junction-state tests; no physical motion."""
import json
import unittest
from types import SimpleNamespace as NS
from test_lane_follower import LaneTests

class NavigationTests(LaneTests):
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

    def test_normal_lane_loss_still_requests_zero(self):
        self.configure_route(["A", "B", "C"])
        self.assertEqual(self.node.navigation_state, "following")
        self.assertEqual(self.step(error=None, both=False), (0.0, 0.0, 0.0))

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

    def test_new_red_during_crossing_faults(self):
        self.configure_route()
        self.enter_crossing()
        for _ in range(4):
            self.step()
        self.step(red=True)
        self.assertEqual(self.node.navigation_state, "fault")

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
