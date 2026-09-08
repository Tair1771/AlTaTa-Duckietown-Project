"""Offline checks for the bounded ground-test timing and reporting helpers."""

import importlib.util
import math
from pathlib import Path
import unittest


PATH = Path(__file__).resolve().parents[1] / "tools" / "bounded_ground_supervisor.py"
SPEC = importlib.util.spec_from_file_location("bounded_ground_supervisor", PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FakeClock:
    def __init__(self):
        self.value = 10.0
        self.sleeps = []

    def monotonic(self):
        return self.value

    def sleep(self, duration):
        self.sleeps.append(duration)
        self.value += duration


class GroundSupervisorTests(unittest.TestCase):
    def test_conservative_limits_are_accepted(self):
        MODULE.validate_limits(8.0, 0.05, 0.05, 0.02)

    def test_unsafe_or_nonfinite_limits_are_rejected(self):
        bad = [
            (0, .04, .05, .01), (8.01, .04, .05, .01),
            (.8, .051, .05, .01), (.8, .04, .051, .01),
            (.8, .04, .039, .01), (.8, .04, .05, .021),
            (math.nan, .04, .05, .01), (True, .04, .05, .01),
        ]
        for values in bad:
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    MODULE.validate_limits(*values)

    def test_wait_uses_absolute_deadline(self):
        clock = FakeClock()
        MODULE.wait_until(10.035, clock.monotonic, clock.sleep)
        self.assertAlmostEqual(clock.value, 10.035, places=9)
        self.assertTrue(clock.sleeps)
        self.assertLessEqual(max(clock.sleeps), .01)

    def test_summary_uses_complete_window_and_checks_post_stop(self):
        samples = [
            (1.0, 0.0, 0.0),
            (2.0, 0.03, 0.05),
            (2.2, 0.04, 0.04),
            (2.8, 0.0, 0.0),
            (3.0, 0.0, 0.0),
        ]
        result = MODULE.summarize_samples(samples, 1.5, 2.5)
        self.assertEqual(result["nonzero_executed_samples"], 2)
        self.assertEqual(result["executed_left_range"], [0.03, 0.04])
        self.assertEqual(result["executed_right_range"], [0.04, 0.05])
        self.assertEqual(result["post_stop_samples"], 2)
        self.assertTrue(result["post_stop_all_zero"])

    def test_summary_rejects_missing_or_nonzero_post_stop_feedback(self):
        self.assertFalse(MODULE.summarize_samples([], 1, 2)["post_stop_all_zero"])
        result = MODULE.summarize_samples([(2.1, 0.01, 0.0)], 1, 2)
        self.assertFalse(result["post_stop_all_zero"])

    def test_final_feedback_requires_eight_recent_zero_samples(self):
        zeros = [(float(i), 0.0, 0.0) for i in range(8)]
        self.assertTrue(MODULE.latest_samples_zero(zeros))
        self.assertFalse(MODULE.latest_samples_zero(zeros[:-1]))
        self.assertFalse(MODULE.latest_samples_zero(zeros[:-1] + [(8.0, .01, 0.0)]))


if __name__ == "__main__":
    unittest.main()
