"""Actual controller live-session integration with OpenCV and mocked ROS."""
import unittest
import json
import cv2
import numpy as np
from copy import deepcopy
from types import SimpleNamespace as NS
from test_lane_follower import LaneTests
from live_chat import LiveChatSession, Intent, interpret_live
from route_planner import plan_route


class LiveNavigationTests(unittest.TestCase):
    setUp = LaneTests.setUp
    subscribe = LaneTests.subscribe
    message = LaneTests.message
    deliver = LaneTests.deliver
    image = LaneTests.image
    speeds = LaneTests.speeds

    def command(self, action, **kwargs):
        n = self.node
        value = dict(action=action, id=str(len(n._seen_commands)+1), issued_at=self.now,
                     run_id="test", expected_control_epoch=n._control_epoch)
        value.update(kwargs)
        n.command_callback(NS(data=json.dumps(value)))
        return n._last_command

    def managed(self, route=None):
        n = self.node
        n.route_enabled = n.drive_enabled = n.junctions_calibrated = True
        self.assertTrue(self.command("set_route", route=route or ["A", "D"], position_confirmed=True,
                                     managed_session=True)["accepted"])
        n._camera_valid = True
        n._last_frame_time = self.now
        self.assertTrue(self.command("continue")["accepted"])
        return n

    def straight(self):
        n = self.node
        n._junction_lane_geometry = {"valid": True, "near_support": True,
            "image_width": 640, "row_span": .7, "lateral_error": 0., "heading_error": 0.,
            "pairs": [{"center_x": 300.} for _ in range(4)]}
        n._lane_both_visible = True
        n._last_lane_error = 0.
        n._red_line_visible = False
        n.prev_steering = 0.
        n.live_tick(observe=True)
        self.now += .6
        n._last_frame_time = self.now
        n.live_tick(observe=True)
        self.assertTrue(n.live.straight)

    def test_managed_red_wait_never_auto_departs_and_deadline_ends(self):
        n = self.managed()
        n.auto_continue = True
        n.navigation_wheels(0., True)
        self.assertEqual(n.navigation_state, "red_stop")
        start = self.now
        for seconds in (2.1, 30.0, 59.9):
            self.now = start + seconds
            n._last_frame_time = self.now
            n.navigation_wheels(0., True)
            self.assertEqual(n.navigation_state, "red_stop")
            self.assertTrue(n.live.active)
        self.now = start + 60
        n._last_frame_time = self.now
        n.check_camera_timeout(None)
        self.assertFalse(n.live.active)
        self.assertEqual(self.speeds(), (0, 0))
        self.assertFalse(self.command("resume")["accepted"])

    def test_a_e_route_override_straight_then_delayed_left_at_c(self):
        n = self.managed(["A", "E"])
        plan = plan_route("A->E", "B->C")
        self.assertEqual(plan.route, ("A", "E", "B", "C"))
        laptop = LiveChatSession("A->E", plan.turns, run_id="test")
        self.assertEqual(laptop.queue, ["left", "right"])
        test = self
        class Link:
            def send(self, action, command_id=None, **kwargs):
                if command_id is not None:
                    kwargs["id"] = command_id
                return test.command(action, **kwargs)
        link = Link()
        laptop.execute(interpret_live("I want it to go straight on the next junction"),
                       link, n.status())
        self.assertEqual(laptop.queue, ["straight"])
        self.assertEqual(n.route, ["A", "E"])
        n.navigation_wheels(0., True)
        self.now += 2.1
        n._last_frame_time = self.now
        laptop.service(link, n.status())
        self.assertEqual(n._active_turn, "straight")
        self.assertEqual(n.route, ["A", "E", "C"])
        for _ in range(60):
            self.now += .1
            n._last_frame_time = self.now
            n._last_lane_error = 0.
            n._lane_both_visible = True
            n.navigation_wheels(0., False)
            laptop.service(link, n.status())
            if n.route_index == 2:
                break
        self.assertEqual(laptop.approach, "E->C")
        self.assertEqual(laptop.queue, [])
        self.assertIsNone(laptop.inflight)
        n.navigation_wheels(0., True)
        arrival = self.now
        for elapsed in (2.1, 30.1, 55.):
            self.now = arrival + elapsed
            n._last_frame_time = self.now
            wheels = n.navigation_wheels(0., True)
            laptop.service(link, n.status())
            n.publish_wheels(*wheels[:2])
            self.assertEqual(n.navigation_state, "red_stop")
            self.assertTrue(n.live.active)
            self.assertEqual(self.speeds(), (0., 0.))
        self.assertTrue(any("60 seconds" in message for message in laptop.take_notices()))
        self.assertAlmostEqual(n.live.snapshot(self.now)["wait_remaining"], 5.)
        laptop.execute(interpret_live("go left"), link, n.status())
        laptop.service(link, n.status())
        self.assertEqual(n._active_turn, "left")
        self.assertEqual(n.navigation_state, "crossing")
        self.assertEqual(n.route, ["A", "E", "C", "B"])

    def test_direction_after_sixty_seconds_cannot_restart_ended_run(self):
        n = self.managed(["E", "C"])
        n.navigation_wheels(0., True)
        self.now += 60
        n._last_frame_time = self.now
        n.check_camera_timeout(None)
        self.assertFalse(n.live.active)
        ack = self.command("junction_instruction", value="left",
                           approach="E->C", expected_route_index=1)
        self.assertFalse(ack["accepted"])
        self.assertEqual(self.speeds(), (0., 0.))

    def test_pause_check_ends_at_first_red_and_rejects_departure(self):
        n = self.managed()
        n.live.stop_at_next_red = True
        n.navigation_wheels(0., True)
        n.check_camera_timeout(None)
        self.assertEqual(n.navigation_state, "route_complete")
        self.assertFalse(n.live.active)
        self.assertEqual(self.speeds(), (0, 0))
        self.assertEqual(n.live.end_reason, "Pause check complete at red line")
        self.now += 3
        n._last_frame_time = self.now
        self.assertFalse(self.command("junction_instruction", value="right",
                                      approach="A->D", expected_route_index=1)["accepted"])
        self.assertFalse(self.command("resume")["accepted"])

    def test_single_valid_instruction_reports_same_id_and_rejects_unavailable_exit(self):
        n = self.managed()
        n.navigation_wheels(0., True)
        self.now += 2.1
        n._last_frame_time = self.now
        kw = dict(approach="A->D", expected_route_index=1)
        self.assertFalse(self.command("junction_instruction", value="left", **kw)["accepted"])
        self.assertEqual(n.route, ["A", "D"])
        ack = self.command("junction_instruction", value="right", **kw)
        self.assertTrue(ack["accepted"], ack)
        self.assertEqual(n.route, ["A", "D", "B"])
        self.assertEqual(n.navigation_state, "crossing")
        self.assertEqual(n.status()["live_session"]["instruction_id"], ack["id"])
        self.assertFalse(self.command("junction_instruction", value="right", **kw)["accepted"])

    def test_pause_immediately_zero_without_straight_and_timed_resume(self):
        n = self.managed()
        n.publish_wheels(.1, .1)
        start = self.now
        self.assertFalse(n.live.straight)
        self.assertTrue(self.command("pause", seconds=3)["accepted"])
        self.assertEqual(self.speeds(), (0, 0))
        self.assertEqual(n.live.paused_at, start)
        for i in range(20):
            self.now += .1
            n._last_frame_time = self.now
            n.live_tick(observe=True)
        self.assertIsNotNone(n.live.paused_at)
        n.publish_wheels(.15, .15)
        self.assertEqual(self.speeds(), (0, 0))
        self.now = start + 3
        n._last_frame_time = self.now
        n.check_camera_timeout(None)
        self.assertIsNone(n.live.paused_at)
        n.publish_wheels(.1, .1)
        self.assertGreater(sum(self.speeds()), 0)

    def test_pause_in_crossing_freezes_motion_clock_and_preserves_profile(self):
        n = self.managed()
        n.navigation_wheels(0., True)
        self.now += 2.1
        n._last_frame_time = self.now
        self.command("junction_instruction", value="right", approach="A->D", expected_route_index=1)
        original = n.planned_junction_wheels()
        deadline = n._junction_deadline_at
        self.assertTrue(self.command("pause")["accepted"])
        self.assertEqual(n.planned_junction_wheels(), original)
        self.assertEqual(n._junction_deadline_at, deadline)
        self.assertIsNotNone(n.live.paused_at)
        self.assertEqual(n.navigation_wheels(None, False), (0, 0, 0))
        self.assertEqual(self.speeds(), (0, 0))
        crossing_started = n._crossing_started
        self.now += 5
        n._last_frame_time = self.now
        n.live_tick()
        self.assertEqual(n._junction_deadline_at, deadline + 5)
        self.assertEqual(n._crossing_started, crossing_started + 5)
        self.assertTrue(self.command("resume")["accepted"])
        self.assertEqual(n.planned_junction_wheels(), original)

    def test_camera_failure_during_timed_pause_never_restarts_wheels(self):
        n = self.managed()
        self.command("pause", seconds=2)
        self.now += 2
        n.check_camera_timeout(None)
        self.assertFalse(n.live.active)
        self.assertEqual(self.speeds(), (0, 0))
        self.assertFalse(self.command("resume")["accepted"])

    def test_pause_in_sharp_corner_keeps_remaining_maneuver_time(self):
        n = self.managed()
        n.sharp_corner_enabled = True
        n._sharp_corner_state = "turning"
        n._sharp_corner_state_since = self.now - .4
        started = n._sharp_corner_state_since
        self.command("pause", seconds=3)
        self.assertEqual(n.sharp_corner_wheels(None, False), (0, 0, 0))
        self.now += 3
        n._last_frame_time = self.now
        n.live_tick()
        self.assertIsNone(n.live.paused_at)
        self.assertEqual(n._sharp_corner_state, "turning")
        self.assertAlmostEqual(n._sharp_corner_state_since, started + 3)

    def test_stop_during_pause_cancels_automatic_restart(self):
        n = self.managed()
        self.command("pause", seconds=2)
        self.command("stop")
        self.now += 3
        n._last_frame_time = self.now
        n.check_camera_timeout(None)
        n.publish_wheels(.15, .15)
        self.assertFalse(n.live.active)
        self.assertEqual(self.speeds(), (0, 0))

    def test_profile_affects_confirmed_straight_only(self):
        n = self.managed()
        n.base_speed = .09
        n.max_speed = .20
        n.steering_bias = 0.
        self.assertFalse(self.command("straight_profile", value="fast")["accepted"])
        self.straight()
        for name, speed in (("fast", .11), ("slow", .09), ("normal", .10)):
            self.assertTrue(self.command("straight_profile", value=name)["accepted"])
            n.prev_steering = 0.
            left, right, _ = n.compute_wheel_speeds(0.)
            self.assertAlmostEqual((left+right)/2, speed)
            self.assertLessEqual(max(left, right), .12)
        # Newly seen curvature invalidates cached classification immediately.
        n._junction_lane_geometry["heading_error"] = .2
        self.assertFalse(self.command("straight_profile", value="fast")["accepted"])
        n.prev_steering = 0.
        enabled = n.compute_wheel_speeds(.3)
        n.live.enabled = False
        n.prev_steering = 0.
        ordinary = n.compute_wheel_speeds(.3)
        self.assertEqual(enabled, ordinary)

    def test_all_junction_profiles_unchanged_for_all_speed_selections(self):
        n = self.managed()
        for turn in ("straight", "left", "right"):
            n._active_turn = turn
            n.navigation_state = "crossing"
            for profile in ("slow", "normal", "fast"):
                n.live.profile = profile
                n.live.enabled = False
                original = n.planned_junction_wheels()
                n.live.enabled = True
                self.assertEqual(n.planned_junction_wheels(), original)

    def test_stop_overrides_resume_and_stale_commands(self):
        n = self.managed()
        epoch = n._control_epoch
        self.assertTrue(self.command("stop")["accepted"])
        self.assertFalse(self.command("resume", expected_control_epoch=epoch)["accepted"])
        n.publish_wheels(.2, .2)
        self.assertEqual(self.speeds(), (0, 0))

    def test_legacy_global_speed_controls_rejected_in_live_mode(self):
        n = self.managed()
        for action in ("speed_up", "slow_down", "speed_scale"):
            self.assertFalse(self.command(action, value=1.5)["accepted"])
        self.assertEqual(n.speed_scale, 1.)

    def test_managed_approach_uses_same_red_geometry_as_planned_route(self):
        n = self.managed()
        n.junction_straight_visual_approach = True
        n._red_line_visible = True
        n._last_lane_error = 0.
        n._junction_lane_geometry = {"valid": True, "lateral_error": 0., "heading_error": 0.}
        n.straight_approach_wheels(.09, .09, 0.)
        self.assertTrue(n._junction_approach_active)
        n.navigation_wheels(0., True)
        self.assertEqual(n._junction_stop_geometry, n._junction_lane_geometry)

    def test_laptop_robot_complete_turn_queue_pause_and_next_red(self):
        n = self.managed()
        laptop = LiveChatSession("A->D", ["right", "left"], run_id="test")
        test = self
        class Link:
            def send(self, action, command_id=None, **kwargs):
                if command_id is not None:
                    kwargs["id"] = command_id
                return test.command(action, **kwargs)
        link = Link()
        laptop.observe(n.status())
        n.navigation_wheels(0., True)
        self.now += 2.1
        n._last_frame_time = self.now
        laptop.service(link, n.status())
        self.assertEqual(n._active_turn, "right")
        self.assertEqual(laptop.queue, ["left"])
        for _ in range(60):
            self.now += .1
            n._last_frame_time = self.now
            n._last_lane_error = 0.
            n._lane_both_visible = True
            n.navigation_wheels(0., False)
            laptop.service(link, n.status())
            if n.route_index == 2:
                break
        self.assertEqual(n.route_index, 2)
        self.assertEqual(laptop.approach, "D->B")
        self.assertIsNone(laptop.inflight)
        self.straight()
        laptop.execute(Intent("pause", 1), link, n.status())
        n.publish_wheels(.1, .1)
        self.assertEqual(self.speeds(), (0, 0))
        self.assertEqual(laptop.queue, ["left"])
        self.now += 1.1
        n._last_frame_time = self.now
        n.check_camera_timeout(None)
        self.assertIsNone(n.live.paused_at)
        n.navigation_wheels(0., True)
        self.now += 2.1
        n._last_frame_time = self.now
        laptop.service(link, n.status())
        self.assertEqual(n._active_turn, "left")
        self.assertEqual(n.route, ["A", "D", "B", "C"])

    def test_interleaved_stop_blocks_late_pause_resume_profile_and_junction(self):
        n = self.managed()
        epoch = n._control_epoch
        self.command("stop")
        for action, kwargs in (("pause", {}), ("resume", {}),
                               ("straight_profile", {"value": "fast"}),
                               ("junction_instruction", {"value": "right", "expected_route_index": 1,
                                                          "approach": "A->D"})):
            ack = self.command(action, expected_control_epoch=epoch, **kwargs)
            self.assertFalse(ack["accepted"])
        self.assertFalse(n.live.active)
        self.assertTrue(n.manual_stop)

    def test_real_detector_classifies_straight_but_not_curved_or_missing_markings(self):
        n = self.managed()
        n.junction_straight_lane_target_fraction = .5
        for _ in range(9):
            self.now += .1
            self.deliver(self.image())
        self.assertTrue(n.live.straight, n._junction_lane_geometry)
        curved = np.zeros((480, 640, 3), dtype=np.uint8)
        for y in range(240, 480):
            shift = int(100 * ((480-y)/240.) ** 2)
            cv2.line(curved, (160+shift, y), (180+shift, y), (0,255,255), 1)
            cv2.line(curved, (470+shift, y), (490+shift, y), (255,255,255), 1)
        self.now += .1
        self.deliver(curved)
        self.assertFalse(n.live.straight)
        self.now += .1
        self.deliver(np.zeros_like(curved))
        self.assertFalse(n.live.straight)

    def test_new_start_cannot_reset_position_at_an_active_red_stop(self):
        n = self.managed()
        n.navigation_wheels(0., True)
        n.publish_wheels(0., 0.)
        ack = self.command("set_route", route=["D", "B"], managed_session=True,
                           position_confirmed=True, run_id="replacement")
        self.assertFalse(ack["accepted"])
        self.assertEqual(n.route, ["A", "D"])
        self.assertEqual(n.live.run_id, "test")


if __name__ == "__main__":
    unittest.main()
