"""Offline tests for bench isolation and honest reboot readiness reporting."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch
import contextlib
import io
import sys
import json
import tempfile
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import isolated_bench_check as bench
import inspect_robot_setup as inspect
import connect_companion as preview
import check_stationary_camera as camera_check
from email.message import Message


class BenchTests(unittest.TestCase):
    def test_tunnel_reuse_requires_both_services(self):
        status = io.BytesIO(b'{"state":"awaiting_route"}')
        camera = io.BytesIO(b"P6")
        camera.headers = Message()
        camera.headers["Content-Type"] = "image/x-portable-pixmap"
        with patch.object(preview.urllib.request, "urlopen", side_effect=[status, camera]) as request, \
                patch.object(preview.subprocess, "Popen", side_effect=AssertionError("Redundant SSH tunnel")):
            preview.ensure_tunnel()
        self.assertEqual(request.call_count, 2)

    def test_tunnel_start_failure_is_reported_without_retry_loop(self):
        tunnel = SimpleNamespace(poll=lambda: 255)
        with patch.object(preview.urllib.request, "urlopen", side_effect=OSError("Refused")), \
                patch.object(preview.subprocess, "Popen", return_value=tunnel) as spawn:
            with self.assertRaisesRegex(RuntimeError, "SSH tunnel exited"):
                preview.ensure_tunnel()
        self.assertEqual(spawn.call_count, 1)

    def safe(self):
        return {"HostConfig": {"NetworkMode": "none", "Privileged": False,
                "CapDrop": ["ALL"], "SecurityOpt": ["no-new-privileges"],
                "RestartPolicy": {"Name": "no"}}, "Mounts": [],
                "Config": {"Labels": {bench.LABEL: "1"}}}

    def test_actual_docker_configuration_must_be_isolated(self):
        bench.verify_isolation(self.safe())
        for key, value in (("NetworkMode", "host"), ("Privileged", True),
                           ("Binds", ["/dev:/dev"]), ("Devices", ["/dev/i2c-1"]),
                           ("PortBindings", {"11311/tcp": [{}]}), ("PidMode", "host"),
                           ("RestartPolicy", {"Name": "always"}), ("CapDrop", [])):
            with self.subTest(key=key):
                info = self.safe()
                info["HostConfig"][key] = value
                with self.assertRaises(RuntimeError):
                    bench.verify_isolation(info)
        info = self.safe()
        info["Mounts"] = [{"Source": "/var/run/docker.sock"}]
        with self.assertRaises(RuntimeError):
            bench.verify_isolation(info)

    def test_default_does_not_connect_or_start_anything(self):
        with patch.object(sys, "argv", ["isolated_bench_check.py"]), \
                patch.object(bench, "execute", side_effect=AssertionError("Started")), \
                contextlib.redirect_stdout(io.StringIO()) as output:
            bench.main()
        self.assertIn("PLAN ONLY", output.getvalue())

    def test_check_has_local_deadline_no_network_no_hardware(self):
        args = bench.create_args("test", "pinned-image")
        self.assertEqual(args[args.index("--network")+1], "none")
        self.assertEqual(args[args.index("--entrypoint")+1], "/usr/bin/timeout")
        self.assertIn("480", args)
        self.assertEqual(args[args.index("--pull")+1], "never")
        for flag in ("--privileged", "--device", "--mount", "-v", "-p"):
            self.assertNotIn(flag, args)

    def test_readiness_requires_all_nodes_and_sole_normal_publisher(self):
        output = "\n".join(["ros base Up", "duckiebot-interface driver Up", "car-interface control Up",
            "CHECK: ROS master and normal wheel publisher", "/duck2/camera_node",
            "/duck2/wheels_driver_node", "/duck2/kinematics_node", "Publishers:",
            " * /duck2/kinematics_node (http://robot)", "Subscribers:", " * /duck2/wheels_driver_node"])
        self.assertEqual(inspect.readiness_issues(output), [])
        broken = output.replace("car-interface control Up", "").replace(
            " * /duck2/kinematics_node (http://robot)", "None")
        self.assertTrue(any("car-interface" in x for x in inspect.readiness_issues(broken)))
        self.assertTrue(any("ownership" in x for x in inspect.readiness_issues(broken)))
        competing = output.replace("Publishers:", "Publishers:\n * /other (http://other)")
        self.assertTrue(inspect.readiness_issues(competing))

    def test_uptime_alone_does_not_invalidate_runtime_cache(self):
        self.assertEqual(inspect.runtime_fingerprint("ros base Up 1 minute"),
                         inspect.runtime_fingerprint("ros base Up 2 hours"))
        self.assertNotEqual(inspect.runtime_fingerprint("ros base-a Up 1 minute"),
                            inspect.runtime_fingerprint("ros base-b Up 1 minute"))

    def test_rebooted_preview_is_recreated_without_starting_old_container(self):
        with patch.object(preview, "ssh", side_effect=["", preview.NAME, "false", "removed"]) as ssh, \
                patch.object(preview, "remote_status", side_effect=AssertionError("Stopped service queried")):
            self.assertFalse(preview.reuse_preview())
        self.assertEqual(ssh.call_args_list[-1].args[0], "docker rm " + preview.NAME)
        self.assertFalse(any("docker start" in call.args[0] for call in ssh.call_args_list))

    def test_preview_refuses_driving_service_and_uncertain_state(self):
        with patch.object(preview, "ssh", return_value="duck2-companion-driving") as ssh:
            with self.assertRaises(RuntimeError):
                preview.reuse_preview()
            self.assertEqual(ssh.call_count, 1)
        with patch.object(preview, "ssh", side_effect=["", preview.NAME, "unknown"]) as ssh:
            with self.assertRaises(RuntimeError):
                preview.reuse_preview()
            self.assertEqual(ssh.call_count, 3)

    def test_running_preview_requires_drive_disabled(self):
        with patch.object(preview, "ssh", side_effect=["", preview.NAME, "true"]), \
                patch.object(preview, "remote_status", return_value={"drive_enabled": True}), \
                patch.object(preview, "ensure_tunnel", side_effect=AssertionError("Unsafe preview reused")):
            with self.assertRaises(RuntimeError):
                preview.reuse_preview()

    @unittest.skipUnless(importlib.util.find_spec("PIL"), "Use bundled camera/UI Python")
    def test_camera_check_refuses_driving_without_requesting_frames(self):
        response = io.BytesIO(json.dumps({"drive_enabled": True, "wheel_speeds": [0, 0]}).encode())
        with patch.object(camera_check.urllib.request, "urlopen", return_value=response), \
                patch.object(camera_check, "CameraClient", side_effect=AssertionError("Camera requested")):
            with self.assertRaises(RuntimeError):
                camera_check.main()

    @unittest.skipUnless(importlib.util.find_spec("PIL"), "Use bundled camera/UI Python")
    def test_three_camera_views_decode_and_stale_frames_fail(self):
        def status():
            return io.BytesIO(b'{"drive_enabled":false,"wheel_speeds":[0,0]}')
        frames = [SimpleNamespace(ppm=b"P6\n1 1\n255\n\x00\x00\x00", age=.02,
                                  captured_at=float(i+1)) for i in range(9)]
        with tempfile.TemporaryDirectory() as folder, \
                patch.dict(camera_check.os.environ, {"LOCALAPPDATA": folder}), \
                patch.object(camera_check.urllib.request, "urlopen", side_effect=lambda *a, **k: status()), \
                patch.object(camera_check.time, "sleep"), \
                patch.object(camera_check.CameraClient, "frame", side_effect=frames), \
                contextlib.redirect_stdout(io.StringIO()) as output:
            camera_check.main()
            self.assertIn("PASS", output.getvalue())
            self.assertEqual(len(list(Path(folder).rglob("*-camera.json"))), 1)
        with patch.object(camera_check.urllib.request, "urlopen", return_value=status()), \
                patch.object(camera_check.CameraClient, "frame", return_value=SimpleNamespace(
                    ppm=frames[0].ppm, age=.6, captured_at=1.)):
            with self.assertRaises(RuntimeError):
                camera_check.main()


if __name__ == "__main__":
    unittest.main()
