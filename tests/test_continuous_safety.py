"""Pure checks for the independent continuous-route fail-stop monitor."""
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages/duckie_lane_follower/src"))
from duckie_lane_follower.continuous_safety import SafetyState


class ContinuousSafetyTests(unittest.TestCase):
    def test_slow_initial_registration_is_bounded_and_requires_zero_output(self):
        state = SafetyState(0.)
        self.assertIsNone(state.fault(8., []))
        self.assertIn("ownership", state.fault(15., []))
        state = SafetyState(0.)
        state.record_executed(3., .01, 0.)
        self.assertIn("ownership", state.fault(3., []))

    def test_startup_allowance_cannot_hide_lost_or_competing_owner(self):
        state = SafetyState(0.)
        self.assertIsNone(state.fault(3., ["/lane_follower_node"]))
        self.assertIn("ownership", state.fault(3.1, []))
        self.assertIn("ownership", SafetyState(0.).fault(3., ["/other"]))

    def moving(self):
        state = SafetyState(0.0)
        state.record_status(2.0, {"camera_valid": True})
        state.record_request(2.0, .1, .1)
        state.record_executed(2.0, .1, .1)
        state.record_encoder(2.0, "left", 10)
        state.record_encoder(2.0, "right", 10)
        return state

    def test_stationary_state_does_not_require_active_feedback(self):
        state = SafetyState(0.0)
        self.assertIsNone(state.fault(3.0, ["/lane_follower_node"]))

    def test_stale_process_and_ownership_fail_closed(self):
        state = self.moving()
        self.assertIn("ownership", state.fault(2.1, ["/other"]))
        self.assertIn("stale", state.fault(2.6, ["/lane_follower_node"]))

    def test_encoder_stall_and_progress(self):
        state = self.moving()
        state.record_status(2.8, {"camera_valid": True})
        state.record_request(2.8, .1, .1)
        state.record_executed(2.8, .1, .1)
        state.record_encoder(2.8, "left", 10)
        state.record_encoder(2.8, "right", 11)
        self.assertIn("Left wheel stalled", state.fault(2.8, ["/lane_follower_node"]))
        state.record_encoder(2.8, "left", 11)
        self.assertIsNone(state.fault(2.8, ["/lane_follower_node"]))

    def test_stall_timer_restarts_at_last_encoder_progress(self):
        state = self.moving()
        state.record_encoder(2.4, "left", 11)
        state.record_encoder(2.4, "right", 11)
        state.record_status(3.0, {"camera_valid": True})
        state.record_request(3.0, .1, .1)
        state.record_executed(3.0, .1, .1)
        state.record_encoder(3.0, "left", 11)
        state.record_encoder(3.0, "right", 11)
        self.assertIsNone(state.fault(3.0, ["/lane_follower_node"]))
        state.record_status(3.2, {"camera_valid": True})
        state.record_request(3.2, .1, .1)
        state.record_executed(3.2, .1, .1)
        state.record_encoder(3.2, "left", 11)
        state.record_encoder(3.2, "right", 11)
        self.assertIn("stalled", state.fault(3.2, ["/lane_follower_node"]))

    def test_inner_zero_wheel_is_not_called_stalled(self):
        state = self.moving()
        state.record_executed(2.1, .1, 0.0)
        state.record_status(2.9, {"camera_valid": True})
        state.record_request(2.9, .1, 0.0)
        state.record_executed(2.9, .1, 0.0)
        state.record_encoder(2.9, "left", 11)
        self.assertIsNone(state.fault(2.9, ["/lane_follower_node"]))


if __name__ == "__main__":
    unittest.main()
