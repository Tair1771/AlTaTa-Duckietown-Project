"""Offline chat and route-draft tests; no network or robot imports."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "laptop"))

from companion_core import OfflineCompanionSession


class CompanionCoreTests(unittest.TestCase):
    def setUp(self):
        self.session = OfflineCompanionSession()

    def prepare(self):
        self.session.select_start("A->B")
        self.session.select_destination("D->A")

    def test_click_equivalent_selection_builds_nonexecuting_payload(self):
        self.prepare()
        payload = self.session.route_payload
        self.assertEqual(payload["start_approach"], "A->B")
        self.assertEqual(payload["destination_approach"], "D->A")
        self.assertFalse(payload["position_confirmed"])
        self.assertFalse(payload["execution_enabled"])
        self.assertTrue(payload["offline_only"])

    def test_natural_start_destination_and_context_status(self):
        self.session.chat("I am starting on A to B")
        result = self.session.chat("Go to red line C from B")
        self.assertEqual(self.session.start_approach, "A->B")
        self.assertEqual(self.session.destination_approach, "B->C")
        self.assertEqual(result.category, "route_selection")
        status = self.session.chat("Where are we going?")
        self.assertIn("B->C", status.explanation)
        self.session.chat("Destination: D->A")
        self.assertEqual(self.session.destination_approach, "D->A")

    def test_turn_correction_replans_to_same_exact_destination(self):
        self.prepare()
        destination = self.session.destination_approach
        result = self.session.chat("Actually, take the next left")
        self.assertEqual(result.category, "route_updated")
        self.assertEqual(self.session.plan.turns[0], "left")
        self.assertEqual(self.session.destination_approach, destination)

    def test_impossible_turn_invalidates_recordable_draft(self):
        self.session.select_start("B->D")
        self.session.select_destination("B->C")
        result = self.session.chat("Go straight at the next junction")
        self.assertTrue(result.needs_clarification)
        self.assertFalse(self.session.draft_valid)
        self.assertIsNone(self.session.route_payload)

    def test_cancel_and_unsupported_do_not_claim_movement(self):
        self.prepare()
        result = self.session.chat("Cancel route")
        self.assertIn("unexecuted", result.explanation)
        self.assertIsNone(self.session.plan)
        self.prepare()
        result = self.session.chat("Reverse to the red line")
        self.assertTrue(result.offline_only)
        self.assertFalse(self.session.draft_valid)

    def test_stop_is_interpretation_only(self):
        result = self.session.chat("Stop")
        self.assertEqual(result.action, "stop")
        self.assertTrue(result.offline_only)


if __name__ == "__main__":
    unittest.main()
