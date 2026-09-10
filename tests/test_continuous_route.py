"""Continuous launcher and route-control behavior without robot access."""

from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "laptop"))

from companion_core import OfflineCompanionSession
from route_control import start_route


class FakeTransport:
    def __init__(self):
        self.actions = []
        self.current = {"control_epoch": 4, "state": "awaiting_route",
                        "route": ["A", "B"], "wheel_speeds": [0, 0],
                        "next_junction": "B",
                        "wheel_publishers": ["/duck2/lane_follower_node"]}

    def poll_status(self):
        self.actions.append(("heartbeat", {}))
        return dict(self.current)

    def status(self):
        return dict(self.current)

    def send(self, action, **kwargs):
        self.actions.append((action, kwargs))
        if action == "set_route":
            self.current.update(state="following", route=kwargs["route"],
                                next_junction=kwargs["route"][1])
        return {"accepted": True, "reason": "Applied"}


class ContinuousRouteTests(unittest.TestCase):
    def test_live_payload_is_position_confirmed_then_explicitly_started(self):
        session = OfflineCompanionSession()
        session.select_start("A->B")
        session.select_destination("B->C")
        transport = FakeTransport()
        final = start_route(transport, session.plan)
        actions = [action for action, _ in transport.actions]
        self.assertEqual(actions, ["heartbeat", "set_route", "heartbeat",
                                   "continue", "heartbeat"])
        sent = transport.actions[1][1]
        self.assertEqual(sent["route"], ["A", "B", "C"])
        self.assertTrue(sent["position_confirmed"])
        self.assertEqual(transport.actions[3][1]["expected_control_epoch"], 4)
        self.assertEqual(final["state"], "following")

    def test_continuous_launcher_enables_route_and_disables_passing(self):
        text = (ROOT / "launchers" / "lane-continuous.sh").read_text(encoding="utf-8")
        for setting in ("_route_enabled:=true", "_junctions_calibrated:=true",
                        "_auto_continue:=true", "_sharp_corner_enabled:=true",
                        "_avoidance_enabled:=false", "_obstacle_enabled:=false",
                        "_require_client_heartbeat:=true",
                        "_road_heading_guard:=false", "_junction_left_visual_latch:=true",
                        "_junction_right_encoder_assist:=true"):
            self.assertIn(setting, text)

    def test_route_start_rejects_competing_wheel_publisher(self):
        session = OfflineCompanionSession()
        session.select_start("A->B")
        session.select_destination("B->C")
        transport = FakeTransport()
        transport.current["wheel_publishers"].append("/duck2/kinematics_node")
        with self.assertRaisesRegex(RuntimeError, "exclusive wheel ownership"):
            start_route(transport, session.plan)
        self.assertEqual([a for a, _ in transport.actions], ["heartbeat"])

    def test_stop_between_route_and_continue_cannot_be_overridden(self):
        session = OfflineCompanionSession()
        session.select_start("A->B")
        session.select_destination("B->C")
        transport = FakeTransport()
        original = transport.poll_status
        calls = [0]
        def poll():
            calls[0] += 1
            if calls[0] == 2:
                transport.current["control_epoch"] += 1
            return original()
        transport.poll_status = poll
        with self.assertRaisesRegex(RuntimeError, "Stop superseded"):
            start_route(transport, session.plan)
        self.assertNotIn("continue", [action for action, _ in transport.actions])

    def test_cancelled_queued_start_sends_no_route(self):
        session = OfflineCompanionSession()
        session.select_start("A->B")
        session.select_destination("B->C")
        transport = FakeTransport()
        self.assertIsNone(start_route(transport, session.plan, cancelled=lambda: True))
        self.assertEqual([a for a, _ in transport.actions], ["heartbeat"])

    def test_invalid_or_boolean_control_epoch_is_rejected(self):
        session = OfflineCompanionSession()
        session.select_start("A->B")
        session.select_destination("B->C")
        for invalid in (None, True, 2.5, "4"):
            with self.subTest(invalid=invalid):
                transport = FakeTransport()
                transport.current["control_epoch"] = invalid
                with self.assertRaisesRegex(RuntimeError, "control epoch"):
                    start_route(transport, session.plan)

    def test_camera_preview_uses_continuous_detector_settings_and_watchdog(self):
        text = (ROOT / "launchers" / "lane-continuous.sh").read_text(encoding="utf-8")
        camera = text.split("camera_gateway.py", 1)[1].split("&", 1)[0]
        for setting in ("_lane_target_fraction:=0.441",
                        '_yellow_lower:="[20,70,80]"',
                        '_white_lower:="[0,0,150]"',
                        "_road_white_reference_value:=170",
                        "_temporal_yellow_only_timeout:=5.0"):
            self.assertIn(setting, camera)
        self.assertIn("continuous_safety_watchdog.py", text)


if __name__ == "__main__":
    unittest.main()
