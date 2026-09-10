"""Offline checks for stationary preparation of the live route controller."""
import pathlib
import shlex
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from prepare_companion_driving import launch_text, verify_stopped
from arm_driver_stopped import require_stopped


class DrivingPreparationTests(unittest.TestCase):
    def test_all_launcher_parameters_preserved(self):
        original = (ROOT / "launchers/lane-continuous.sh").read_text()
        params = lambda text: [v for v in shlex.split(text) if v.startswith("_")]
        self.assertEqual(params(original), params(launch_text()))
        self.assertNotIn("diagnostic_wheels_cmd", launch_text())
        self.assertIn("continuous_safety_watchdog.py", launch_text())

    def test_preparation_replaces_a_verified_stopped_running_session(self):
        text = (ROOT / "tools/prepare_companion_driving.py").read_text()
        verified = text.index("verify_stopped(status())")
        stopped = text.index('ssh("docker stop -t 10 " + NAME)', verified)
        removed = text.index('ssh("docker rm " + NAME)', stopped)
        deployed = text.index('ssh("docker run -d --name', removed)
        self.assertLess(verified, stopped)
        self.assertLess(stopped, removed)
        self.assertLess(removed, deployed)

    def test_requires_stopped_and_exclusive_ownership(self):
        good = dict(drive_enabled=True, manual_stop=True, state="awaiting_route",
                    wheel_speeds=[0, 0], require_client_heartbeat=True,
                    wheel_publishers=["/lane_follower_node"])
        verify_stopped(good)
        require_stopped(good)
        for key, invalid in (("drive_enabled", False), ("manual_stop", False),
                             ("state", "following"), ("wheel_speeds", [0.1, 0.1]),
                             ("require_client_heartbeat", False),
                             ("wheel_publishers", ["/lane_follower_node", "/duck2/kinematics_node"])):
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                verify_stopped(dict(good, **{key: invalid}))
            if key != "require_client_heartbeat":
                with self.subTest(arm_key=key), self.assertRaises(RuntimeError):
                    require_stopped(dict(good, **{key: invalid}))


if __name__ == "__main__":
    unittest.main()
