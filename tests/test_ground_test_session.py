"""Offline checks for the Windows-side bounded-test session wrapper."""

import importlib.util
import io
import json
from pathlib import Path
import tarfile
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace


PATH = Path(__file__).resolve().parents[1] / "tools" / "run_duck2_ground_test.py"
SPEC = importlib.util.spec_from_file_location("run_duck2_ground_test", PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Result:
    def __init__(self, stdout="", returncode=0):
        self.stdout = stdout
        self.returncode = returncode


class FakeRunner:
    def __init__(self, replies):
        self.replies = list(replies)
        self.commands = []

    def remote(self, command, timeout=30, check=True):
        self.commands.append(command)
        if not self.replies:
            raise AssertionError("Unexpected remote command: {}".format(command))
        return self.replies.pop(0)


def write_valid_evidence(directory, frames=2):
    rows = []
    for index in range(frames):
        name = "frame-{:05d}.jpg".format(index)
        (directory / name).write_bytes(b"jpeg" + bytes([index]))
        rows.append({"file": name, "camera_stamp_s": 100.0 + index})
    (directory / "frames.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
    telemetry = {
        "requested_wheels": [],
        "executed_wheels": [],
        "lane_status": [],
    }
    (directory / "telemetry.json").write_text(json.dumps(telemetry), encoding="utf-8")
    summary = {
        "evidence_frames": frames,
        "final_feedback_all_zero": True,
        "post_stop_zero_confirmed": True,
    }
    (directory / "completion.json").write_text(json.dumps({
        "complete": True, "summary": summary,
    }), encoding="utf-8")


class GroundSessionTests(unittest.TestCase):
    def readiness_facts(self):
        return {"hostname": "duck2", "containers": {
            name: {"running": True, "image": "sha256:example"}
            for name in ("duckiebot-interface", "car-interface")}, "ros":
            "/duck2/camera_node\n/duck2/wheels_driver_node\n/duck2/kinematics_node\n"
            "Publishers:\n * /duck2/kinematics_node (http://duck2/)\nSubscribers:\n"}

    def test_readiness_rejects_incomplete_boot_and_competing_controllers(self):
        MODULE.validate_readiness(self.readiness_facts())
        for issue in ("hostname", "container", "node", "temporary", "publisher"):
            facts = self.readiness_facts()
            if issue == "hostname": facts["hostname"] = "different-robot"
            if issue == "container": facts["containers"]["car-interface"]["running"] = False
            if issue == "node": facts["ros"] = facts["ros"].replace("/duck2/camera_node\n", "")
            if issue == "temporary": facts["ros"] = "/lane_follower_node\n" + facts["ros"]
            if issue == "publisher": facts["ros"] = facts["ros"].replace("Subscribers:", " * /other (http://other/)\nSubscribers:")
            with self.subTest(issue=issue), self.assertRaises(MODULE.SessionError):
                MODULE.validate_readiness(facts)

    def test_read_only_check_classifies_login_identity_and_network_errors(self):
        for output, hint in (("Permission denied", "Unlock"),
                             ("Host key verification failed", "identity"),
                             ("Could not resolve hostname duck2", "alias"),
                             ("Connection timed out", "hotspot")):
            runner = MODULE.Runner()
            with patch.object(runner, "remote", return_value=Result(output, 255)) as call:
                with self.assertRaisesRegex(MODULE.SessionError, hint):
                    runner.check_readiness()
                call.assert_called_once()

    def test_windows_check_only_never_runs_a_session(self):
        with patch.object(MODULE.sys, "platform", "win32"), patch.object(MODULE, "Runner") as runner, patch.object(MODULE, "run_session") as session:
            self.assertEqual(MODULE.main(["--label", "startup", "--check-only"]), 0)
            runner.return_value.check_readiness.assert_called_once()
            session.assert_not_called()

    def test_connection_timeout_is_bounded_and_not_retried(self):
        with patch.object(MODULE.subprocess, "run", side_effect=MODULE.subprocess.TimeoutExpired("ssh", 40)) as call:
            with self.assertRaisesRegex(MODULE.SessionError, "timed out"):
                MODULE.Runner().check_readiness()
            call.assert_called_once()

    def test_wsl_invocation_fails_before_any_connection(self):
        with patch.object(MODULE.sys, "platform", "linux"), patch.object(MODULE, "Runner") as runner:
            with self.assertRaisesRegex(MODULE.SessionError, "Windows PowerShell"):
                MODULE.main(["--label", "startup", "--check-only"])
            runner.assert_not_called()

    def test_default_motion_window_is_fifteen_seconds(self):
        self.assertEqual(MODULE.parse_args(["--label", "test"]).duration, 15.0)

    def test_ground_right_pivot_selects_only_reviewed_two_second_profile(self):
        args = MODULE.parse_args([
            "--label", "pivot", "--duration", "2",
            "--ground-right-pivot", "--confirm-go",
        ])
        name, profile = MODULE.diagnostic_profile(args)
        self.assertEqual(name, "ground_right_pivot")
        self.assertEqual(profile, ["--fixed-left", "0.15",
                                   "--fixed-right", "0.0"])

    def test_strong_pivot_is_bounded_exclusive_and_requires_go(self):
        argv = ["--label", "pivot-start", "--ground-strong-pivot", "--duration", "2"]
        args = MODULE.parse_args(argv)
        self.assertEqual(MODULE.diagnostic_profile(args),
                         ("ground_strong_pivot",
                          ["--fixed-left", "0.20", "--fixed-right", "0.0"]))
        with self.assertRaisesRegex(ValueError, "fresh Go"):
            MODULE.run_session(args)
        args.confirm_go = True
        args.duration = 2.01
        with self.assertRaisesRegex(ValueError, "at most 2.0"):
            MODULE.run_session(args)
        args.ground_equal_wheels = True
        with self.assertRaisesRegex(ValueError, "only one"):
            MODULE.diagnostic_profile(args)

    def test_ground_rolling_right_selects_reviewed_two_second_profile(self):
        args = MODULE.parse_args([
            "--label", "rolling", "--duration", "2",
            "--ground-rolling-right", "--confirm-go",
        ])
        name, profile = MODULE.diagnostic_profile(args)
        self.assertEqual(name, "ground_rolling_right")
        self.assertEqual(profile, ["--fixed-left", "0.15",
                                   "--fixed-right", "0.03"])

    def test_ground_strong_rolling_right_selects_reviewed_profile(self):
        args = MODULE.parse_args([
            "--label", "strong-rolling", "--duration", "2",
            "--ground-strong-rolling-right", "--confirm-go",
        ])
        name, profile = MODULE.diagnostic_profile(args)
        self.assertEqual(name, "ground_strong_rolling_right")
        self.assertEqual(profile, ["--fixed-left", "0.20",
                                   "--fixed-right", "0.03"])

    def test_ground_equal_wheels_selects_reviewed_profile(self):
        args = MODULE.parse_args([
            "--label", "equal", "--duration", "2",
            "--ground-equal-wheels", "--confirm-go",
        ])
        name, profile = MODULE.diagnostic_profile(args)
        self.assertEqual(name, "ground_equal_wheels")
        self.assertEqual(profile, ["--fixed-left", "0.15",
                                   "--fixed-right", "0.15"])

    def test_junction_profile_passes_turn_and_red_trigger(self):
        args = MODULE.parse_args([
            "--label", "junction", "--duration", "15",
            "--junction-turn", "left",
            "--red-stop-trigger-bottom-fraction", ".80",
            "--confirm-go",
        ])
        name, profile = MODULE.diagnostic_profile(args)
        self.assertEqual(name, "junction_left")
        self.assertEqual(profile, [
            "--junction-turn", "left",
            "--red-stop-trigger-bottom-fraction", "0.8",
        ])

    def test_stationary_red_line_inspection_needs_no_go(self):
        args = MODULE.parse_args([
            "--label", "red-10cm", "--duration", "2",
            "--inspect-red-line",
            "--red-stop-trigger-bottom-fraction", ".80",
        ])
        name, profile = MODULE.diagnostic_profile(args)
        self.assertEqual(name, "red_line_inspection")
        self.assertEqual(profile, [
            "--inspect-red-line",
            "--red-stop-trigger-bottom-fraction", "0.8",
        ])

    def test_diagnostic_profiles_are_mutually_exclusive(self):
        args = MODULE.parse_args([
            "--label", "pivot", "--ground-right-pivot",
            "--upside-down-left-pivot",
        ])
        with self.assertRaisesRegex(ValueError, "only one diagnostic"):
            MODULE.diagnostic_profile(args)
        args = MODULE.parse_args([
            "--label", "mixed", "--junction-turn", "right",
            "--ground-right-pivot",
        ])
        with self.assertRaisesRegex(ValueError, "only one diagnostic"):
            MODULE.diagnostic_profile(args)

    def test_stop_reason_is_preserved_through_ros_logs_and_traceback(self):
        summary = dict(early_stop_reason="Left wheel stalled while commanded",
                       post_stop_zero_confirmed=True, final_feedback_all_zero=True)
        output = "ROS warning\n" + json.dumps(summary) + "\nTraceback: stopped early\n"
        self.assertEqual(MODULE.summary_from_log(output), summary)
        self.assertTrue(MODULE.confirmed_stop_from_log(output))
        self.assertIsNone(MODULE.summary_from_log("ROS warning without result"))

    def test_run_id_is_unique_and_safe(self):
        from datetime import datetime, timezone
        stamp = datetime(2026, 9, 8, 12, 30, tzinfo=timezone.utc)
        value = MODULE.make_run_id("Left curve / straight", stamp, "abc12345")
        self.assertEqual(value, "20260908T123000Z-left-curve-straight-abc12345")

    def test_valid_bundle_gets_hash_verification_record(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            write_valid_evidence(directory)
            result = MODULE.validate_evidence(directory)
            self.assertTrue(result["verified"])
            self.assertEqual(result["frame_count"], 2)
            saved = json.loads((directory / "evidence_verified.json").read_text())
            self.assertIn("frame-00001.jpg", saved["files_sha256"])

    def test_missing_frame_fails_without_removing_download(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            write_valid_evidence(directory)
            (directory / "frame-00001.jpg").unlink()
            with self.assertRaises(MODULE.SessionError):
                MODULE.validate_evidence(directory)
            self.assertTrue((directory / "telemetry.json").exists())
            self.assertTrue((directory / "completion.json").exists())

    def test_manifest_cannot_escape_evidence_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            write_valid_evidence(directory, frames=0)
            (directory / "frames.jsonl").write_text(
                json.dumps({"file": "../outside.jpg"}) + "\n", encoding="utf-8")
            with self.assertRaises(MODULE.SessionError):
                MODULE.validate_evidence(directory)

    def test_archive_traversal_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            archive_path = root / "bad.tar"
            with tarfile.open(str(archive_path), "w") as archive:
                info = tarfile.TarInfo("../outside.txt")
                payload = b"unsafe"
                info.size = len(payload)
                archive.addfile(info, io.BytesIO(payload))
            with self.assertRaises(MODULE.SessionError):
                MODULE.safe_extract_tar(archive_path, root / "extract")
            self.assertFalse((root / "outside.txt").exists())

    def test_cleanup_stops_test_nodes_before_normal_control(self):
        first_nodes = "/duck2_bounded_ground_supervisor\n/lane_follower_node\n"
        normal_topic = "Publishers:\n * /duck2/kinematics_node (http://duck2/)\nSubscribers:\n"
        runner = FakeRunner([
            Result(first_nodes), Result(), Result(""),
            Result("car-interface\n"), Result("true\n" + normal_topic),
        ])
        publishers = MODULE.restore_normal_control(
            runner, attempts=1, sleep=lambda _: None, stop_confirmed=True)
        self.assertEqual(publishers, {"/duck2/kinematics_node"})
        kill_index = next(i for i, command in enumerate(runner.commands)
                          if "rosnode kill" in command)
        start_index = runner.commands.index("docker start car-interface")
        self.assertLess(kill_index, start_index)

    def test_normal_control_is_not_started_if_test_node_persists(self):
        nodes = "/duck2_ground_watchdog\n"
        runner = FakeRunner([Result(nodes), Result(), Result(nodes)])
        with self.assertRaises(MODULE.SessionError):
            MODULE.restore_normal_control(runner, attempts=1, sleep=lambda _: None,
                                          stop_confirmed=True)
        self.assertNotIn("docker start car-interface", runner.commands)

    def test_publishers_parser_rejects_extra_owner(self):
        output = (
            "Publishers:\n"
            " * /duck2/kinematics_node (http://duck2/)\n"
            " * /unexpected (http://duck2/)\n"
            "Subscribers:\n"
        )
        self.assertEqual(MODULE.parse_topic_publishers(output), {
            "/duck2/kinematics_node", "/unexpected"})

    def test_missing_stop_feedback_blocks_restoration(self):
        runner = FakeRunner([Result("")])
        with self.assertRaisesRegex(MODULE.SessionError, "No confirmed final zero"):
            MODULE.restore_normal_control(runner)
        self.assertNotIn("docker start car-interface", runner.commands)
        self.assertFalse(MODULE.confirmed_stop_from_log("SUPERVISOR_COMPLETE"))
        self.assertFalse(MODULE.confirmed_stop_from_log(json.dumps({
            "post_stop_zero_confirmed": "true", "final_feedback_all_zero": True})))

    def test_upload_preserves_local_package_paths_in_one_connection(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            sources = [
                (directory / "supervisor.py", "supervisor.py"),
                (directory / "node.py", "node.py"),
                (directory / "route_map.py", "duckie_lane_follower/route_map.py"),
                (directory / "__init__.py", "duckie_lane_follower/__init__.py"),
            ]
            for source, _ in sources:
                source.write_text("# source\n")
            with patch.object(MODULE.subprocess, "run", return_value=Result(b"")) as run:
                MODULE.Runner().stage_sources(sources, "/tmp/unique-test-stage")
                run.assert_called_once()
                self.assertIn("StrictHostKeyChecking=yes", run.call_args[0][0])
                self.assertIn("docker exec -i", run.call_args[0][0][-1])
                payload = io.BytesIO(run.call_args[1]["input"])
                with tarfile.open(fileobj=payload, mode="r:gz") as archive:
                    self.assertEqual(archive.getnames(), [
                        "supervisor.py", "node.py",
                        "duckie_lane_follower/route_map.py",
                        "duckie_lane_follower/__init__.py",
                    ])

    def test_upload_rejects_unsafe_package_path(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "route_map.py"
            source.write_text("# source\n")
            with self.assertRaisesRegex(ValueError, "Unsafe staged source path"):
                MODULE.Runner().stage_sources(
                    [(source, "../duckie_lane_follower/route_map.py")],
                    "/tmp/unique-test-stage")

    def test_invalid_duration_and_missing_go_do_not_connect(self):
        for duration in (0, 15.01, float("nan"), float("inf"), True):
            with self.subTest(duration=duration), patch.object(MODULE, "Runner") as runner:
                with self.assertRaises(ValueError):
                    MODULE.run_session(SimpleNamespace(confirm_go=True, duration=duration))
                runner.assert_not_called()
        with patch.object(MODULE, "Runner") as runner:
            with self.assertRaises(ValueError):
                MODULE.run_session(SimpleNamespace(confirm_go=False))
            runner.assert_not_called()

    def test_upside_down_left_pivot_is_capped_before_connecting(self):
        args = SimpleNamespace(confirm_go=True, duration=2.01, label="pivot",
                               output_root="unused", ssh_alias="duck2",
                               upside_down_left_pivot=True)
        with patch.object(MODULE, "Runner") as runner:
            with self.assertRaisesRegex(ValueError, "at most 2.0"):
                MODULE.run_session(args)
            runner.assert_not_called()

    def test_upside_down_profiles_are_mutually_exclusive(self):
        args = SimpleNamespace(confirm_go=True, duration=2.0, label="pivot",
                               output_root="unused", ssh_alias="duck2",
                               upside_down_left_pivot=True,
                               upside_down_load_profile=True)
        with patch.object(MODULE, "Runner") as runner:
            with self.assertRaisesRegex(ValueError, "only one diagnostic"):
                MODULE.run_session(args)
            runner.assert_not_called()

    def test_ground_right_pivot_is_capped_before_connecting(self):
        args = SimpleNamespace(confirm_go=True, duration=2.01, label="pivot",
                               output_root="unused", ssh_alias="duck2",
                               ground_right_pivot=True)
        with patch.object(MODULE, "Runner") as runner:
            with self.assertRaisesRegex(ValueError, "at most 2.0"):
                MODULE.run_session(args)
            runner.assert_not_called()

    def test_ground_rolling_right_is_capped_before_connecting(self):
        args = SimpleNamespace(confirm_go=True, duration=2.01, label="rolling",
                               output_root="unused", ssh_alias="duck2",
                               ground_rolling_right=True)
        with patch.object(MODULE, "Runner") as runner:
            with self.assertRaisesRegex(ValueError, "at most 2.0"):
                MODULE.run_session(args)
            runner.assert_not_called()

    def test_ground_strong_rolling_right_is_capped_before_connecting(self):
        args = SimpleNamespace(confirm_go=True, duration=2.01,
                               label="strong-rolling", output_root="unused",
                               ssh_alias="duck2",
                               ground_strong_rolling_right=True)
        with patch.object(MODULE, "Runner") as runner:
            with self.assertRaisesRegex(ValueError, "at most 2.0"):
                MODULE.run_session(args)
            runner.assert_not_called()

    def test_full_session_fifteen_seconds_and_failure_cleanup(self):
        normal = "true\nPublishers:\n * /duck2/kinematics_node (http://duck2/)\nSubscribers:\n"
        zero_log = json.dumps({"post_stop_zero_confirmed": True,
                               "final_feedback_all_zero": True}) + "\nSUPERVISOR_COMPLETE"
        for failure in (None, "startup", "upload", "interrupt", "download"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as temporary:
                calls = []
                class SessionRunner:
                    def __init__(self, alias):
                        pass
                    def check_readiness(self):
                        calls.append("startup")
                        if failure == "startup":
                            raise MODULE.SessionError("startup failed")
                        return self_test.readiness_facts()
                    def stage_sources(self, sources, destination):
                        calls.append("upload")
                        if failure == "upload":
                            raise MODULE.SessionError("upload failed")
                    def remote(self, command, **kwargs):
                        calls.append(command)
                        if "--camera-guided-curve" in command:
                            self_test.assertIn("--duration 15.0", command)
                            self_test.assertTrue(command.startswith("docker exec"))
                            self_test.assertIn("--kill-after=5s 45s", command)
                            self_test.assertEqual(kwargs["timeout"], 55)
                            if failure == "interrupt":
                                raise KeyboardInterrupt()
                            return Result(zero_log)
                        if "docker inspect" in command:
                            return Result(normal)
                        return Result("")
                    def export_tar(self, remote_directory, destination):
                        calls.append("download")
                        if failure == "download":
                            destination.write_bytes(b"partial archive")
                            raise MODULE.SessionError("download failed")
                        source = Path(temporary) / "robot-copy"
                        source.mkdir()
                        write_valid_evidence(source)
                        with tarfile.open(str(destination), "w:gz") as archive:
                            for item in source.iterdir():
                                archive.add(str(item), arcname=item.name)
                self_test = self
                args = SimpleNamespace(confirm_go=True, duration=15.0, label="test",
                                       output_root=temporary, ssh_alias="duck2")
                with patch.object(MODULE, "Runner", SessionRunner), patch("sys.stdout", io.StringIO()):
                    if failure is None:
                        self.assertEqual(MODULE.run_session(args), 0)
                    else:
                        with self.assertRaises(MODULE.SessionError):
                            MODULE.run_session(args)
                if failure == "startup":
                    self.assertEqual(calls, ["startup"])
                elif failure == "upload":
                    self.assertEqual(calls, ["startup", "upload"])
                elif failure == "interrupt":
                    self.assertNotIn("docker start car-interface", calls)
                    self.assertIn("download", calls)
                else:
                    self.assertIn("docker start car-interface", calls)
                self.assertFalse(any("rm " in command for command in calls))

    def test_combined_restore_check_rejects_extra_owner(self):
        topic = "true\nPublishers:\n * /duck2/kinematics_node (http://duck2/)\n * /other (http://duck2/)\nSubscribers:\n"
        runner = FakeRunner([Result(""), Result("car-interface"), Result(topic)])
        with self.assertRaisesRegex(MODULE.SessionError, "ownership was not restored"):
            MODULE.restore_normal_control(runner, attempts=1, sleep=lambda _: None,
                                          stop_confirmed=True)


if __name__ == "__main__":
    unittest.main()
