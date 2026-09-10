"""Live chat policy and transactional map queue; no robot connection."""
import sys
import unittest
from pathlib import Path
from copy import deepcopy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "laptop"))
sys.path.insert(0, str(ROOT / "tests"))
from live_chat import Intent, LiveChatSession, follow_turn, interpret_live
from duckie_lane_follower.live_session import LiveSession, PROFILES
from route_planner import RED_LINE_APPROACHES, MAP_PORTS, parse_approach, junction_turn
from route_control import start_route
from companion_core import OfflineCompanionSession


class Transport:
    def __init__(self, status):
        self.current = deepcopy(status)
        self.calls = []
        self.fail_once = False

    def send(self, action, **kwargs):
        self.calls.append((action, kwargs))
        if self.fail_once:
            self.fail_once = False
            raise RuntimeError("Lost acknowledgment")
        if action == "set_route":
            self.current["live_session"].update(run_id=kwargs.get("run_id"), active=True)
        return {"accepted": True}

    def poll_status(self):
        return deepcopy(self.current)


def status(session):
    return {"state": "red_stop", "route_index": 1, "control_epoch": 0,
            "current_approach": session.approach, "junction_instruction_ready": True,
            "wheel_publishers": ["/duck2/lane_follower_node"],
            "live_session": {"version": 1, "active": True, "run_id": session.run_id,
                             "pause_mode": "immediate",
                             "profile": "normal", "instruction_id": None}}


class LiveLanguageTests(unittest.TestCase):
    def test_supported_phrases(self):
        samples = {
            "The next turn should be right": Intent("turns", ("right",)),
            "Left after that one": Intent("turns", ("left",), True),
            "Next left then right": Intent("turns", ("left", "right")),
            "Can you go straight at the next intersection?": Intent("turns", ("straight",)),
            "I want it to go straight on the next junction": Intent("turns", ("straight",)),
            "I want the bot to go left": Intent("turns", ("left",)),
            "stop for 7s": Intent("pause", 7),
            "stop for 7 seconds": Intent("pause", 7),
            "wait 1.5s": Intent("pause", 1.5),
            "stop": Intent("pause"), "pause for five seconds": Intent("pause", 5),
            "please wait 3.5 seconds": Intent("pause", 3.5),
            "stop the bot": Intent("pause"), "stop the run": Intent("end"),
            "quit": Intent("end"), "pause the run": Intent("pause"),
            "continue": Intent("resume"), "cancel pause": Intent("resume"),
            "set speed to normal": Intent("profile", "normal"),
            "speed up": Intent("profile", "up"), "slow down": Intent("profile", "down"),
        }
        for text, expected in samples.items():
            with self.subTest(text=text):
                self.assertEqual(interpret_live(text), expected)

    def test_ambiguous_unsupported_or_negated_cannot_move(self):
        for text in ("don't stop", "do not quit", "reverse", "turn right if possible",
                     "what if we turn left", "speed 0.8", "right or left", "drive to the duck",
                     "stop for 7s extra", "stop for 7ms", "stop for -7s",
                     "pause for 0 seconds", "pause for 9999 seconds", "resume then right"):
            with self.subTest(text=text):
                self.assertEqual(interpret_live(text).action, "clarify")


class LiveQueueTests(unittest.TestCase):
    def test_terminal_status_preserves_only_verified_last_turn(self):
        for reported_turn in ("right", "left"):
            with self.subTest(reported_turn=reported_turn):
                queue = LiveChatSession("A->D", ["right"])
                value = status(queue)
                transport = Transport(value)
                queue.service(transport, value)
                value.update(state="route_complete", route_index=2, current_approach="D->B",
                             junction_last_result={"outcome": "reacquired_at_next_red",
                                                   "turn": reported_turn, "route_index": 2})
                value["live_session"].update(active=False, instruction_id=queue.inflight["id"],
                                             end_reason="No junction instruction within 60 seconds of the red stop")
                queue.observe(value)
                self.assertFalse(queue.active)
                self.assertEqual(queue.approach, "D->B" if reported_turn == "right" else "A->D")
                self.assertEqual(queue.queue, [])

    def test_map_validation_all_directed_approaches(self):
        for approach in RED_LINE_APPROACHES:
            previous, junction = parse_approach(approach)
            allowed = {junction_turn(previous, junction, node): "%s->%s" % (junction, node)
                       for node in MAP_PORTS[junction].values() if node != previous}
            for turn in ("left", "right", "straight"):
                with self.subTest(approach=approach, turn=turn):
                    if turn in allowed:
                        self.assertEqual(follow_turn(approach, turn), allowed[turn])
                    else:
                        with self.assertRaises(ValueError):
                            follow_turn(approach, turn)

    def test_replace_append_and_reject_atomically(self):
        queue = LiveChatSession("A->D", ["straight"])
        queue.replace_turns(["right"])
        queue.replace_turns(["left"], append=True)
        self.assertEqual(queue.queue, ["right", "left"])
        with self.assertRaises(ValueError):
            queue.replace_turns(["left"])
        self.assertEqual(queue.queue, ["right", "left"])

    def test_once_only_send_then_reported_completion(self):
        queue = LiveChatSession("A->D", ["right", "left"])
        value = status(queue)
        transport = Transport(value)
        queue.service(transport, value)
        self.assertEqual(queue.queue, ["left"])
        for _ in range(4):
            queue.service(transport, value)
        self.assertEqual(len(transport.calls), 1)
        value.update(state="following", route_index=2, current_approach="D->B",
                     junction_instruction_ready=False,
                     junction_last_result={"outcome": "reacquired", "turn": "right", "route_index": 2})
        value["live_session"]["instruction_id"] = queue.inflight["id"]
        queue.service(transport, value)
        self.assertEqual(queue.approach, "D->B")
        self.assertIsNone(queue.inflight)
        self.assertEqual(queue.queue, ["left"])

    def test_pending_current_turn_cannot_be_replaced(self):
        queue = LiveChatSession("A->D", ["right"])
        value = status(queue)
        transport = Transport(value)
        queue.service(transport, value)
        # Next turn is evaluated from D->B, even while still inside D.
        queue.replace_turns(["left"])
        self.assertEqual(queue.inflight["turn"], "right")
        self.assertEqual(queue.queue, ["left"])

    def test_lost_ack_retries_same_id_and_preserves_queue(self):
        queue = LiveChatSession("A->D", ["right"])
        value = status(queue)
        transport = Transport(value)
        transport.fail_once = True
        with self.assertRaises(RuntimeError):
            queue.service(transport, value)
        self.assertEqual(queue.queue, ["right"])
        with self.assertRaisesRegex(ValueError, "unknown outcome"):
            queue.replace_turns(["straight"])
        queue.service(transport, value)
        self.assertEqual(transport.calls[0][1]["command_id"], transport.calls[1][1]["command_id"])
        self.assertEqual(queue.queue, [])

    def test_status_recovers_lost_ack_without_second_send(self):
        queue = LiveChatSession("A->D", ["right"])
        value = status(queue)
        transport = Transport(value)
        transport.fail_once = True
        with self.assertRaises(RuntimeError):
            queue.service(transport, value)
        value["live_session"]["instruction_id"] = queue.inflight["id"]
        value.update(state="crossing", junction_instruction_ready=False)
        queue.service(transport, value)
        self.assertEqual(len(transport.calls), 1)
        self.assertEqual(queue.queue, [])

    def test_wrong_run_and_unreported_progress_prevent_delivery(self):
        queue = LiveChatSession("A->D", ["right"])
        value = status(queue)
        transport = Transport(value)
        value["live_session"]["run_id"] = "other"
        with self.assertRaises(ValueError):
            queue.service(transport, value)
        value = status(queue)
        value["route_index"] = 3
        with self.assertRaises(ValueError):
            queue.service(transport, value)
        self.assertFalse(transport.calls)

    def test_no_queue_asks_once_never_auto_continues(self):
        queue = LiveChatSession("A->D")
        value = status(queue)
        transport = Transport(value)
        for _ in range(3):
            queue.service(transport, value)
        self.assertEqual(len(queue.take_notices()), 1)
        self.assertFalse(transport.calls)

    def test_pause_resume_preserve_queue_and_robot_ack_required(self):
        queue = LiveChatSession("A->D", ["right"])
        value = status(queue)
        transport = Transport(value)
        queue.execute(Intent("pause", 5), transport, value)
        queue.execute(Intent("resume"), transport, value)
        self.assertEqual(queue.queue, ["right"])
        self.assertEqual([c[0] for c in transport.calls], ["pause", "resume"])

    def test_start_sends_only_initial_lane_in_managed_mode(self):
        planner = OfflineCompanionSession()
        planner.select_start("A->D")
        planner.select_destination("B->C")
        queue = LiveChatSession(planner.plan.start_approach, planner.plan.turns)
        transport = Transport(status(queue))
        start_route(transport, planner.plan, live_session=queue)
        self.assertEqual(transport.calls[0][1]["route"], ["A", "D"])
        self.assertTrue(transport.calls[0][1]["managed_session"])
        self.assertEqual(transport.calls[1][0], "continue")

    def test_start_rejects_old_deferred_pause_controller(self):
        planner = OfflineCompanionSession()
        planner.select_start("A->D")
        planner.select_destination("B->C")
        queue = LiveChatSession(planner.plan.start_approach, planner.plan.turns)
        value = status(queue)
        del value["live_session"]["pause_mode"]
        transport = Transport(value)
        with self.assertRaisesRegex(RuntimeError, "immediate-pause"):
            start_route(transport, planner.plan, live_session=queue)
        self.assertFalse(transport.calls)

    def test_pause_check_payload_and_no_turns(self):
        planner = OfflineCompanionSession()
        planner.select_start("A->E")
        planner.select_destination("A->E")
        queue = LiveChatSession("A->E", stop_at_next_red=True)
        value = status(queue)
        value["live_session"]["supports_pause_check"] = True
        transport = Transport(value)
        start_route(transport, planner.plan, live_session=queue)
        self.assertTrue(transport.calls[0][1]["stop_at_next_red"])
        self.assertEqual(transport.calls[0][1]["route"], ["A", "E"])
        with self.assertRaisesRegex(ValueError, "Pause check ends"):
            queue.replace_turns(["right"])


class LiveClockTests(unittest.TestCase):
    def new(self):
        state = LiveSession()
        state.start("test")
        self.assertEqual(state.profile, "normal")
        self.assertEqual(len(PROFILES), 3)
        return state

    def test_pause_immediate_in_every_phase_without_straight_classification(self):
        for phase in ("following", "crossing", "reacquiring", "red_stop"):
            with self.subTest(phase=phase):
                state = self.new()
                state.request_pause(5, now=10)
                self.assertEqual(state.paused_at, 10)
                self.assertFalse(state.pause_pending)
                state.tick(14.9, phase, 1, 10, True)
                self.assertIsNotNone(state.paused_at)
                state.tick(15, phase, 1, 10, True)
                self.assertIsNone(state.paused_at)
                self.assertTrue(state.active)

    def test_red_timeout_uses_arrival_even_if_first_tick_late(self):
        state = self.new()
        state.tick(20, "red_stop", 1, 10, True)
        self.assertEqual(state.snapshot(20)["wait_remaining"], 50)
        state.tick(69.9, "red_stop", 1, 10, True)
        self.assertTrue(state.active)
        state.tick(70, "red_stop", 1, 10, True)
        self.assertFalse(state.active)

    def test_indefinite_timeout_and_continue(self):
        for resume in (False, True):
            state = self.new()
            state.request_pause(now=1)
            state.straight = True
            state.tick(1, "following", 1, None, True)
            if resume:
                state.resume()
            state.tick(31, "following", 1, None, True)
            self.assertEqual(state.active, resume)

    def test_unhealthy_timed_resume_ends_instead(self):
        state = self.new()
        state.request_pause(2, now=0)
        state.straight = True
        state.tick(0, "following", 1, None, True)
        state.tick(2, "following", 1, None, False)
        self.assertFalse(state.active)

    def test_duplicate_pause_does_not_extend_deadline(self):
        state = self.new()
        state.request_pause(5, now=10)
        with self.assertRaisesRegex(ValueError, "Already paused"):
            state.request_pause(9, now=12)
        self.assertEqual(state.resume_at, 15)

    def test_long_timed_pause_freezes_red_instruction_wait(self):
        state = self.new()
        state.tick(10, "red_stop", 1, 10, True)
        state.request_pause(60, now=12)
        state.tick(72, "red_stop", 1, 10, True)
        self.assertTrue(state.active)
        self.assertIsNone(state.paused_at)
        self.assertEqual(state.snapshot(72)["wait_remaining"], 58)


if __name__ == "__main__":
    unittest.main()
