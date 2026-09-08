"""No ROS imports or hardware required."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2"))
from safety import SafetyGate


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.gate = SafetyGate(lambda: self.now)

    def request(self, **values):
        request = dict(session=self.gate.session, epoch=self.gate.epoch, sequence=1,
                       issued_monotonic=self.now, left=.04, right=.05)
        request.update(values)
        return request

    def test_default_zero_and_explicit_ready_required(self):
        self.assertEqual(self.gate.output(), (0, 0))
        self.assertFalse(self.gate.arm(False))
        self.assertFalse(self.gate.accept(self.request()))

    def test_expiry_latches_and_new_requests_cannot_rearm(self):
        self.gate.arm(True)
        self.assertTrue(self.gate.accept(self.request()))
        self.now += .26
        self.assertEqual(self.gate.output(), (0, 0))
        self.assertFalse(self.gate.accept(self.request(sequence=2)))
        self.assertFalse(self.gate.armed)

    def test_stop_invalidates_queued_work_and_session(self):
        self.gate.arm(True)
        old = self.request()
        self.gate.stop()
        self.gate.arm(True)
        self.assertFalse(self.gate.accept(old))
        self.assertFalse(self.gate.accept(self.request(session="old")))
        self.assertTrue(self.gate.accept(self.request()))

    def test_limits_bad_timestamps_and_payloads_stop(self):
        for values in ({"left":-.1}, {"right":.051}, {"left":float("nan")},
                       {"left":True}, {"issued_monotonic":99.}, {"issued_monotonic":101.}):
            self.gate.arm(True)
            self.assertFalse(self.gate.accept(self.request(**values)))
            self.assertEqual(self.gate.output(), (0, 0))
            self.assertFalse(self.gate.armed)

    def test_duplicate_cannot_extend_deadline(self):
        self.gate.arm(True)
        request = self.request()
        self.gate.accept(request)
        self.now += .2
        self.assertFalse(self.gate.accept(request))
        self.now += .06
        self.assertEqual(self.gate.output(), (0, 0))

    def test_absolute_arm_window_cannot_be_extended_by_requests(self):
        self.gate.arm(True)
        for i in range(1, 82):
            self.now = 100 + i*.1
            self.gate.accept(self.request(sequence=i))
        self.assertFalse(self.gate.armed)
        self.assertEqual(self.gate.output(), (0, 0))
