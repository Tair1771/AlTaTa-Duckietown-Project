"""Offline checks for the bounded ground-test timing and reporting helpers."""

import importlib.util
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace


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
    def test_watchdog_readiness_ignores_ros_diagnostics_before_sentinel(self):
        watchdog = SimpleNamespace(
            stdout=io.StringIO(
                "[WARN] Inbound TCP/IP connection ended before handshake\n"
                "WATCHDOG_READY\n"
            ),
            poll=lambda: None,
            returncode=None,
        )
        MODULE.wait_for_watchdog(watchdog)

    def test_watchdog_readiness_requires_exact_sentinel(self):
        watchdog = SimpleNamespace(
            stdout=io.StringIO("prefix WATCHDOG_READY suffix\n"),
            poll=lambda: 2,
            returncode=2,
        )
        with self.assertRaisesRegex(RuntimeError, "exited before readiness") as error:
            MODULE.wait_for_watchdog(watchdog, timeout=.1)
        self.assertIn("prefix WATCHDOG_READY suffix", str(error.exception))

    def test_watchdog_readiness_reports_exit_and_prior_diagnostics(self):
        watchdog = SimpleNamespace(
            stdout=io.StringIO("[WARN] startup detail\n"),
            poll=lambda: 2,
            returncode=2,
        )
        with self.assertRaisesRegex(RuntimeError, "exited before readiness") as error:
            MODULE.wait_for_watchdog(watchdog)
        self.assertIn("startup detail", str(error.exception))

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

    def test_fixed_command_refreshes_at_a_bounded_interval(self):
        self.assertEqual(MODULE.fixed_command_due(10.0, 10.1), (False, 10.1))
        due, next_publish_at = MODULE.fixed_command_due(10.1, 10.1)
        self.assertTrue(due)
        self.assertAlmostEqual(next_publish_at, 10.2)

    def test_preflight_requires_zero_feedback_after_current_stop(self):
        samples = [(9.9, 0.0, 0.0), (10.1, 0.1, 0.1)]
        self.assertFalse(MODULE.preflight_zero_feedback_ready(samples, 10.0))
        samples.append((10.2, 0.0, 0.0))
        self.assertTrue(MODULE.preflight_zero_feedback_ready(samples, 10.0))

    def test_watchdog_detects_parent_pipe_loss_during_motion(self):
        clock = FakeClock()
        disconnected = io.StringIO("")
        selector = lambda read, write, errors, timeout: ([disconnected], [], [])
        self.assertFalse(MODULE.parent_pipe_alive_until(
            11.0, disconnected, selector, clock.monotonic
        ))

    def test_watchdog_keeps_parent_until_deadline(self):
        clock = FakeClock()
        connected = io.StringIO("unused")
        def selector(read, write, errors, timeout):
            clock.value += timeout
            return [], [], []
        self.assertTrue(MODULE.parent_pipe_alive_until(
            10.12, connected, selector, clock.monotonic
        ))

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

    def test_post_stop_confirmation_requires_new_zero_feedback(self):
        samples = [(1.0, 0.03, 0.15)]
        self.assertFalse(MODULE.post_stop_zero_confirmed(samples, 1.1))
        samples.extend((1.2 + index * .01, 0.0, 0.0) for index in range(8))
        self.assertTrue(MODULE.post_stop_zero_confirmed(samples, 1.1))

    def test_wait_for_post_stop_zero_is_bounded_and_accepts_late_zero_window(self):
        clock = FakeClock()
        samples = [(9.9, 0.03, 0.15), (10.01, 0.03, 0.15)]

        def publish_feedback(duration):
            clock.sleep(duration)
            if len(samples) < 10:
                samples.append((clock.monotonic(), 0.0, 0.0))

        self.assertTrue(MODULE.wait_for_post_stop_zero(
            samples, 10.0, timeout_s=.25,
            monotonic=clock.monotonic, sleep=publish_feedback,
        ))
        self.assertGreaterEqual(len(samples), 10)
    def test_fixed_turn_accepts_only_the_bounded_diagnostic_values(self):
        MODULE.validate_fixed_turn(3.0, .03, .15)
        MODULE.validate_fixed_turn(8.0, .03, .15)
        MODULE.validate_fixed_turn(2.0, .15, 0.0)
        MODULE.validate_fixed_turn(2.0, .15, .03)
        MODULE.validate_fixed_turn(2.0, .15, .15)
        MODULE.validate_fixed_turn(2.0, .20, .03)
        MODULE.validate_fixed_turn(2.0, .20, 0.0)
        for values in [(8.01, .03, .15), (3.0, .04, .15),
                       (3.0, .03, .14), (2.01, .15, 0.0),
                       (2.0, .15, .01), (2.01, .15, .03),
                       (2.01, .15, .15),
                       (2.01, .20, .03), (2.01, .20, 0.0),
                       (2.0, .21, 0.0),
                       (2.0, .20, .04), (True, .03, .15)]:
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    MODULE.validate_fixed_turn(*values)

    def test_fixed_turn_requires_both_wheel_values(self):
        left_only = MODULE.parse_args(["--fixed-left", ".03"])
        right_only = MODULE.parse_args(["--fixed-right", ".15"])
        self.assertTrue(MODULE.fixed_turn_requested(left_only))
        self.assertTrue(MODULE.fixed_turn_requested(right_only))
        self.assertIsNone(left_only.fixed_right)
        self.assertIsNone(right_only.fixed_left)

    def test_encoder_delta_uses_only_the_motion_window(self):
        samples = [(1.0, 40), (2.0, 45), (2.5, 60), (3.0, 75), (4.0, 99)]
        self.assertEqual(MODULE.encoder_delta_in_window(samples, 2.1, 3.1), 30)
        self.assertIsNone(MODULE.encoder_delta_in_window([], 2.1, 3.1))

    def test_evidence_recorder_writes_compressed_frame_and_manifest(self):
        class Stamp:
            def to_sec(self):
                return 123.0

        class Header:
            stamp = Stamp()

        class Image:
            header = Header()
            data = b"jpeg-test-data"

        with tempfile.TemporaryDirectory() as directory:
            recorder = MODULE.CameraEvidenceRecorder(directory, 4.0)
            recorder.callback(Image())
            recorder.close()
            self.assertEqual((Path(directory) / "frame-00000.jpg").read_bytes(), Image.data)
            record = json.loads((Path(directory) / "frames.jsonl").read_text())
            self.assertEqual(record["camera_stamp_s"], 123.0)
            self.assertEqual(record["phase"], "preflight")

    def test_evidence_recorder_rejects_reused_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            (Path(directory) / "old-frame.jpg").write_bytes(b"old")
            with self.assertRaises(ValueError):
                MODULE.CameraEvidenceRecorder(directory, 4.0)

    def test_evidence_recorder_freezes_before_completion_count(self):
        class Stamp:
            def to_sec(self):
                return 123.0

        class Header:
            stamp = Stamp()

        class Image:
            header = Header()
            data = b"jpeg-test-data"

        with tempfile.TemporaryDirectory() as directory:
            recorder = MODULE.CameraEvidenceRecorder(directory, 4.0)
            recorder.callback(Image())
            self.assertEqual(recorder.freeze(), 1)
            recorder.callback(Image())
            recorder.close()
            self.assertEqual(len(list(Path(directory).glob("*.jpg"))), 1)
            self.assertEqual(len((Path(directory) / "frames.jsonl").read_text().splitlines()), 1)

    def test_evidence_telemetry_preserves_raw_samples(self):
        with tempfile.TemporaryDirectory() as directory:
            output = MODULE.write_evidence_telemetry(
                directory, "fixed_turn", 10.0, 13.0,
                [(10.0, .03, .10)], [(10.1, .03, .10)],
                [(10.0, 20)], [(10.0, 30)],
            )
            data = json.loads(output.read_text())
            self.assertEqual(data["mode"], "fixed_turn")
            self.assertEqual(data["motion_window_monotonic_s"]["released"], 10.0)
            self.assertEqual(data["requested_wheels"][0], [10.0, .03, .10])

    def test_evidence_completion_preserves_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            summary = {
                "evidence_frames": 2,
                "final_feedback_all_zero": True,
                "post_stop_zero_confirmed": True,
            }
            output = MODULE.write_evidence_completion(directory, summary)
            data = json.loads(output.read_text())
            self.assertTrue(data["complete"])
            self.assertEqual(data["summary"], summary)

    def test_aborted_preflight_keeps_evidence_and_requires_fresh_zero(self):
        for has_fresh_zero in (False, True):
            with self.subTest(has_fresh_zero=has_fresh_zero), tempfile.TemporaryDirectory() as directory:
                clock = FakeClock()
                events = []
                executed = [(9.0, 0., 0.)]  # Old zero must not confirm this stop.
                def stop_again():
                    events.append("stop")
                    if has_fresh_zero:
                        executed.extend((clock.value + i*.001, 0., 0.) for i in range(1, 9))
                def freeze():
                    events.append("freeze")
                    return 0
                recorder = SimpleNamespace(output_dir=Path(directory), freeze=freeze)
                statuses = [(9., {"lane_error": None,
                                   "lane_diagnostic": "Lane lost: boundary gap exceeded"})]
                with patch.object(MODULE, "time", clock):
                    summary = MODULE.record_aborted_run(
                        recorder, "camera_guided_curve", None, [], executed,
                        [], [], statuses, "scene rejected", stop_again)
                self.assertEqual(events, ["stop", "freeze"])
                self.assertFalse(summary["motion_started"])
                self.assertEqual(summary["post_stop_zero_confirmed"], has_fresh_zero)
                self.assertEqual(summary["final_feedback_all_zero"], has_fresh_zero)
                telemetry = json.loads((Path(directory)/"telemetry.json").read_text())
                self.assertIsNone(telemetry["motion_window_monotonic_s"]["released"])
                self.assertEqual(telemetry["lane_status"][0][1], statuses[0][1])
                self.assertEqual(telemetry["early_stop_reason"], "scene rejected")
                self.assertEqual(json.loads((Path(directory)/"completion.json").read_text())["summary"], summary)

    def test_camera_guided_preset_is_exact_and_bounded(self):
        args = MODULE.parse_args(["--camera-guided-curve", "--duration", "15"])
        MODULE.apply_camera_guided_preset(args)
        self.assertTrue(args.smooth_steering_deadband)
        self.assertTrue(args.temporal_lane_width_fallback)
        self.assertTrue(args.boundary_risk_stop)
        self.assertEqual(args.temporal_lane_width_timeout, .30)
        self.assertEqual(args.temporal_yellow_only_timeout, 5.00)
        self.assertEqual(args.min_active_wheel_speed, .03)
        self.assertFalse(args.taper_inner_wheel_floor)
        self.assertEqual(args.white_boundary_risk_fraction, .43)
        self.assertEqual(
            (args.base_speed, args.max_speed, args.max_steering, args.k_p,
             args.near_center_k_p, args.full_gain_error,
             args.lane_target_fraction, args.alpha, args.deadband,
             args.steering_bias),
            (.09, .20, .11, .75, .35, .09, .441, .20, .05, .0075),
        )
        self.assertEqual(args.yellow_lower, [20, 70, 80])
        self.assertEqual(args.yellow_upper, [35, 255, 255])
        self.assertEqual(args.white_lower, [0, 0, 150])
        self.assertEqual(args.white_upper, [180, 55, 255])
        self.assertTrue(args.sharp_corner_enabled)
        self.assertEqual(
            (args.sharp_corner_confirm_seconds,
             args.sharp_corner_approach_seconds,
             args.sharp_corner_recent_lane_seconds,
             args.sharp_corner_turn_speed,
             args.sharp_corner_min_turn_seconds,
             args.sharp_corner_white_confirm_seconds,
             args.sharp_corner_pivot_seconds,
             args.sharp_corner_relief_seconds,
             args.sharp_corner_relief_inner_speed,
             args.sharp_corner_max_turn_seconds,
             args.sharp_corner_reacquire_seconds,
             args.sharp_corner_trigger_error,
             args.sharp_corner_exit_error),
            (.20, 1.0, 1.0, .20, 0.0, .10,
             .45, .25, 0.0, 10.0, .30, .10, .13),
        )
        for duration in (0, 15.01, math.nan, math.inf, True):
            with self.subTest(duration=duration):
                args = MODULE.parse_args(["--camera-guided-curve"])
                args.duration = duration
                with self.assertRaises(ValueError):
                    MODULE.apply_camera_guided_preset(args)
        combined = MODULE.parse_args([
            "--camera-guided-curve", "--fixed-left", ".03", "--fixed-right", ".15"
        ])
        with self.assertRaises(ValueError):
            MODULE.apply_camera_guided_preset(combined)

    def test_junction_presets_are_direction_specific_and_bounded(self):
        expected_routes = {
            "straight": ["A", "B", "C"],
            "left": ["A", "B", "D"],
            "right": ["A", "B", "E"],
        }
        for turn in expected_routes:
            with self.subTest(turn=turn):
                args = MODULE.parse_args([
                    "--junction-turn", turn, "--duration", "15",
                    "--red-stop-trigger-bottom-fraction", ".80",
                ])
                MODULE.apply_junction_preset(args)
                self.assertFalse(args.sharp_corner_enabled)
                self.assertEqual(MODULE.JUNCTION_TEST_ROUTES[turn], expected_routes[turn])
                self.assertEqual(args.junction_straight_speed, .15)
                self.assertEqual(args.junction_straight_approach_max_steering, .03)
                self.assertTrue(args.junction_straight_visual_approach)
                self.assertEqual(args.junction_straight_lane_target_fraction, .49)
                self.assertEqual(args.junction_straight_lateral_gain, .25)
                self.assertEqual(args.junction_straight_heading_gain, .30)
                self.assertEqual(args.junction_straight_reacquire_seconds, .50)
                self.assertEqual(args.junction_straight_settle_seconds, .60)
                self.assertTrue(args.junction_straight_encoder_balance)
                self.assertEqual(args.junction_straight_encoder_balance_gain, .12)
                self.assertEqual(args.junction_straight_encoder_balance_max, .015)
                self.assertEqual(args.junction_straight_encoder_balance_min_ticks, 12.)
                self.assertEqual(args.junction_reacquire_timeout,
                                 {"straight": 9.0, "left": 5.0,
                                  "right": 5.0}[turn])
                self.assertEqual(args.junction_reacquire_max_error,
                                 .10 if turn == "straight" else .35)
                self.assertEqual(MODULE.junction_post_reacquire_seconds(turn),
                                 0.0 if turn == "straight" else 1.0)
                self.assertEqual(args.duration, 15)
                self.assertEqual((args.junction_left_speed, args.junction_left_bias),
                                 (.105, .075))
                self.assertEqual((args.junction_right_speed, args.junction_right_bias),
                                 (.10, .10))
        for value in (.64, .95, math.nan, math.inf):
            args = MODULE.parse_args(["--junction-turn", "straight"])
            args.red_stop_trigger_bottom_fraction = value
            with self.assertRaises(ValueError):
                MODULE.apply_junction_preset(args)

    def test_straight_release_rejects_old_search_timeout(self):
        from unittest.mock import patch
        args = MODULE.parse_args(["--junction-turn", "straight", "--duration", "15"])
        MODULE.apply_junction_preset(args)
        fields = ("entry_seconds", "straight_seconds", "left_seconds", "right_seconds",
                  "reacquire_timeout", "reacquire_max_error",
                  "straight_speed", "straight_approach_max_steering",
                  "straight_visual_approach", "straight_lane_target_fraction",
                  "straight_lateral_gain", "straight_heading_gain",
                  "straight_departure_max_heading",
                  "straight_reacquire_max_lateral",
                  "straight_reacquire_max_heading", "straight_reacquire_seconds",
                  "straight_settle_max_lateral", "straight_settle_max_heading",
                  "straight_settle_max_steering", "straight_settle_seconds",
                  "straight_encoder_balance", "straight_encoder_balance_gain",
                  "straight_encoder_balance_max",
                  "straight_encoder_balance_min_ticks",
                  "left_speed", "right_speed",
                  "left_bias", "right_bias")
        status = dict(route_enabled=True, junctions_calibrated=True,
                      sharp_corner_enabled=False, red_stop_trigger_bottom_fraction=.86,
                      route=["A", "B", "C"], route_index=1, state="following",
                      manual_stop=False,
                      junction_settings={f: getattr(args, "junction_" + f) for f in fields})
        with patch.object(MODULE, "require_camera_guided_mode"):
            MODULE.require_junction_mode(status, "straight", .86)
            status["junction_settings"]["reacquire_timeout"] = 3.0
            with self.assertRaisesRegex(RuntimeError, "reacquire_timeout"):
                MODULE.require_junction_mode(status, "straight", .86)
            status["junction_settings"]["reacquire_timeout"] = 9.0
            status["junction_settings"]["reacquire_max_error"] = .35
            with self.assertRaisesRegex(RuntimeError, "reacquire_max_error"):
                MODULE.require_junction_mode(status, "straight", .86)

    def test_fifteen_second_watchdog_stops_at_deadline_without_real_wait(self):
        clock = FakeClock()
        connected = io.StringIO("unused")
        def selector(read, write, errors, timeout):
            clock.value += timeout
            return [], [], []
        deadline = clock.monotonic() + MODULE.MAX_CAMERA_GUIDED_DURATION + MODULE.WATCHDOG_MARGIN
        self.assertTrue(MODULE.parent_pipe_alive_until(
            deadline, connected, selector, clock.monotonic))
        self.assertAlmostEqual(clock.value, 25.25)
        from unittest.mock import patch
        with patch.object(MODULE, "watchdog_main", return_value=0) as watchdog:
            self.assertEqual(MODULE.main(["--watchdog", "--watchdog-delay", "15.25"]), 0)
            watchdog.assert_called_once()
            with self.assertRaises(ValueError):
                MODULE.main(["--watchdog", "--watchdog-delay", "15.26"])

    def test_motion_monitor_accepts_current_healthy_state(self):
        status = (10.0, {"camera_valid": True, "lane_error": -.4,
                         "red_stop": False, "fault": None})
        self.assertIsNone(MODULE.motion_fault(
            now=10.1, camera_fresh=True, status_sample=status,
            lane_running=True, topic_publishers=["supervisor", "lane"],
            expected_publishers={"supervisor", "lane"},
            executed_samples=[(10.05, .03, .15)], require_lane_status=True,
        ))

    def test_junction_mode_allows_only_expected_unmarked_phases(self):
        baseline = dict(
            now=10.1, camera_fresh=True, lane_running=True,
            topic_publishers=["supervisor", "lane"],
            expected_publishers={"supervisor", "lane"},
            executed_samples=[(10.05, .09, .09)],
            require_lane_status=True, junction_mode=True,
        )
        for state, phase in (("crossing", "entry"),
                             ("crossing", "turning"),
                             ("reacquiring", "searching")):
            status = {"camera_valid": True, "lane_error": None,
                      "red_stop": False, "fault": None,
                      "state": state, "junction_phase": phase}
            self.assertIsNone(MODULE.motion_fault(
                status_sample=(10.0, status), **baseline))
        stopped = {"camera_valid": True, "lane_error": .1,
                   "red_stop": True, "fault": None,
                   "state": "red_stop", "junction_phase": "stopped"}
        self.assertIsNone(MODULE.motion_fault(
            status_sample=(10.0, stopped), **baseline))
        for state, phase in (("following", "idle"),
                             ("reacquiring", "aligning"),
                             ("crossing", "fault")):
            status = {"camera_valid": True, "lane_error": None,
                      "red_stop": False, "fault": None,
                      "state": state, "junction_phase": phase}
            self.assertEqual(MODULE.motion_fault(
                status_sample=(10.0, status), **baseline), "Lane lost")

    def test_junction_completion_requires_matching_reacquisition(self):
        status = {
            "state": "following", "junction_phase": "complete",
            "route_index": 2,
            "junction_last_result": {"outcome": "reacquired", "turn": "left"},
        }
        self.assertTrue(MODULE.junction_completion_status(status, "left"))
        self.assertFalse(MODULE.junction_completion_status(status, "straight"))
        straight = dict(status, junction_settled=True)
        straight["junction_last_result"] = {
            "outcome": "reacquired", "turn": "straight"}
        self.assertTrue(MODULE.junction_completion_status(straight, "straight"))
        self.assertFalse(MODULE.junction_completion_status(status, "right"))
        self.assertFalse(MODULE.junction_completion_status(
            dict(status, route_index=1), "left"))

    def test_command_ack_is_explicit_and_rejections_are_reported(self):
        accepted = [{"last_command": {
            "id": "junction-continue-left", "accepted": True,
            "reason": "Applied"}}]
        self.assertTrue(MODULE.command_acknowledged(
            accepted, "junction-continue-left"))
        self.assertFalse(MODULE.command_acknowledged(accepted, "different"))
        with self.assertRaisesRegex(RuntimeError, "rejected"):
            MODULE.command_acknowledged([{"last_command": {
                "id": "junction-continue-left", "accepted": False,
                "reason": "test rejection"}}], "junction-continue-left")

    def test_junction_continue_waits_for_fresh_complete_incoming_lane(self):
        route = MODULE.JUNCTION_TEST_ROUTES["straight"]
        ready = {
            "camera_valid": True, "camera_age": .08,
            "lane_both_visible": True, "lane_error": .03,
            "red_stop": False, "fault": None,
            "state": "following", "manual_stop": True,
            "route": route, "route_index": 1,
        }
        self.assertTrue(MODULE.junction_continue_ready(ready, route))
        for change in (
                {"camera_valid": False}, {"camera_age": .5},
                {"camera_age": float("nan")}, {"lane_both_visible": False},
                {"lane_error": None}, {"red_stop": True},
                {"fault": "test"}, {"manual_stop": False},
                {"route": ["A", "D", "C"]}, {"route_index": 0}):
            with self.subTest(change=change):
                self.assertFalse(MODULE.junction_continue_ready(
                    dict(ready, **change), route))

    def test_supervised_junction_does_not_publish_continue_before_camera_ready(self):
        # The real node publishes an initial status before it accepts commands.
        statuses = [{"last_command": None}]
        published = []
        route = MODULE.JUNCTION_TEST_ROUTES["straight"]
        publisher = SimpleNamespace(get_num_connections=lambda: 1)

        def publish(_rospy, _String, _publisher, command, repeats=3):
            published.append(command["action"])
            status = {"last_command": {
                "id": command["id"], "accepted": True, "reason": "Applied"}}
            if command["action"] == "set_route":
                status.update({
                    "camera_valid": False, "camera_age": None,
                    "lane_both_visible": False, "lane_error": None,
                    "red_stop": False, "fault": None,
                    "state": "following", "manual_stop": True,
                    "route": route, "route_index": 1,
                })
            statuses.append(status)

        holds = []
        def hold():
            holds.append(True)
            if (published == ["set_route"] and len(holds) >= 3
                    and not MODULE.junction_continue_ready(statuses[-1], route)):
                statuses.append({
                    "camera_valid": True, "camera_age": .06,
                    "lane_both_visible": True, "lane_error": .02,
                    "red_stop": False, "fault": None,
                    "state": "following", "manual_stop": True,
                    "route": route, "route_index": 1,
                    "last_command": statuses[-1]["last_command"],
                })

        with patch.object(MODULE, "publish_json_command", side_effect=publish), \
                patch.object(MODULE.time, "sleep", return_value=None):
            MODULE.configure_supervised_junction(
                object(), object(), publisher, statuses, "straight", hold)
        self.assertEqual(published, ["set_route", "continue"])
        self.assertGreaterEqual(len(holds), 3)

    def test_camera_guided_release_rejects_old_or_misconfigured_node(self):
        for status in ({}, {"smooth_steering_deadband": False},
                       {"smooth_steering_deadband": "true"},
                       {"smooth_steering_deadband": 1},
                       {"smooth_steering_deadband": True},
                       {"smooth_steering_deadband": True,
                        "temporal_lane_width_fallback": False},
                       {"smooth_steering_deadband": True,
                        "temporal_lane_width_fallback": "true"}):
            with self.subTest(status=status), self.assertRaises(RuntimeError):
                MODULE.require_camera_guided_mode(status)
        MODULE.require_camera_guided_mode({
            "smooth_steering_deadband": True,
            "temporal_lane_width_fallback": True,
            "temporal_lane_width_timeout": .30,
            "temporal_yellow_only_timeout": 5.00,
            "boundary_risk_stop": True,
            "white_boundary_risk_fraction": .43,
            "max_steering": .11,
            "min_active_wheel_speed": .03,
            "taper_inner_wheel_floor": False,
            "k_p": .75,
            "near_center_k_p": .35,
            "full_gain_error": .09,
            "yellow_lower": [20, 70, 80],
            "yellow_upper": [35, 255, 255],
            "white_lower": [0, 0, 150],
            "white_upper": [180, 55, 255],
            "sharp_corner_enabled": True,
            "sharp_corner_confirm_seconds": .20,
            "sharp_corner_approach_seconds": 1.0,
            "sharp_corner_recent_lane_seconds": 1.0,
            "sharp_corner_turn_speed": .20,
            "sharp_corner_min_turn_seconds": 0.0,
            "sharp_corner_white_confirm_seconds": .10,
            "sharp_corner_pivot_seconds": .45,
            "sharp_corner_relief_seconds": .25,
            "sharp_corner_relief_inner_speed": 0.0,
            "sharp_corner_max_turn_seconds": 10.0,
            "sharp_corner_reacquire_seconds": .30,
            "sharp_corner_trigger_error": .10,
            "sharp_corner_exit_error": .13,
        })

    def test_camera_guided_release_requires_both_boundaries(self):
        with self.assertRaises(RuntimeError):
            MODULE.require_two_boundary_preflight({"lane_both_visible": False})
        with self.assertRaises(RuntimeError):
            MODULE.require_two_boundary_preflight({})
        MODULE.require_two_boundary_preflight({"lane_both_visible": True})

    def test_camera_guided_scene_accepts_a_valid_bend(self):
        status = {"camera_valid": True, "red_stop": False,
                  "lane_error": -.31, "lane_both_visible": True}
        self.assertIsNone(MODULE.camera_guided_scene_issue(status))

    def test_camera_guided_scene_reports_the_specific_safety_reason(self):
        self.assertEqual(MODULE.camera_guided_scene_issue({}),
                         "no current valid camera status")
        self.assertEqual(MODULE.camera_guided_scene_issue({
            "camera_valid": True, "red_stop": True,
            "lane_error": .1, "lane_both_visible": True}),
                         "red-stop detection is active")
        self.assertEqual(MODULE.camera_guided_scene_issue({
            "camera_valid": True, "red_stop": False,
            "lane_error": None, "lane_both_visible": True}),
                         "no finite lane estimate (no perception diagnostic)")
        self.assertIn("visible yellow and white", MODULE.camera_guided_scene_issue({
            "camera_valid": True, "red_stop": False,
            "lane_error": .1, "lane_both_visible": False}))

    def test_motion_monitor_stops_for_each_live_fault(self):
        healthy = (10.0, {"camera_valid": True, "lane_error": -.4,
                          "red_stop": False, "fault": None})
        baseline = dict(
            now=10.1, camera_fresh=True, status_sample=healthy,
            lane_running=True, topic_publishers=["supervisor", "lane"],
            expected_publishers={"supervisor", "lane"},
            executed_samples=[(10.05, .03, .15)], require_lane_status=True,
        )
        cases = {
            "camera": dict(camera_fresh=False),
            "publishers": dict(topic_publishers=["supervisor", "lane", "other"]),
            "process": dict(lane_running=False),
            "wheel feedback": dict(executed_samples=[(9.0, .03, .15)]),
            "status": dict(status_sample=(9.0, healthy[1])),
            "lane": dict(status_sample=(10.0, dict(healthy[1], lane_error=None))),
            "red": dict(status_sample=(10.0, dict(healthy[1], red_stop=True))),
            "fault": dict(status_sample=(10.0, dict(healthy[1], fault="test"))),
        }
        for label, changes in cases.items():
            with self.subTest(label=label):
                values = dict(baseline)
                values.update(changes)
                self.assertIsNotNone(MODULE.motion_fault(**values))

    def test_encoder_watchdog_detects_commanded_stall(self):
        # A sustained motor command with fresh, unchanged encoder counts is
        # a stall even while the corner is allowed to wait for white.
        commands = [(9.25, .09, .09), (9.5, .09, .09),
                    (9.75, .09, .09), (10.0, .09, .09)]
        stopped = [(9.25, 100), (9.5, 100), (9.75, 100), (10.0, 100)]
        moving_left = [(9.25, 100), (9.5, 104), (9.75, 108), (10.0, 112)]
        moving_right = [(9.25, 200), (9.5, 205), (9.75, 210), (10.0, 215)]

        self.assertIn("Left wheel stalled", MODULE.encoder_motion_fault(
            10.0, commands, stopped, moving_right, released_at=8.0))
        self.assertIn("Right wheel stalled", MODULE.encoder_motion_fault(
            10.0, commands, moving_left, stopped, released_at=8.0))
        self.assertIsNone(MODULE.encoder_motion_fault(
            10.0, commands, moving_left, moving_right, released_at=8.0))

    def test_only_confirmed_pivot_can_outlast_lane_width_estimate(self):
        status = dict(camera_valid=True, red_stop=False, fault=None,
                      lane_error=None, sharp_corner_enabled=True,
                      sharp_corner_state="turning", sharp_corner_phase="pivot",
                      yellow_boundary_visible=True, white_boundary_visible=False,
                      lane_diagnostic="Lane lost: boundary gap exceeded")
        baseline = dict(now=10., camera_fresh=True, status_sample=(9.95, status),
                        lane_running=True, topic_publishers=["supervisor", "lane"],
                        expected_publishers={"supervisor", "lane"},
                        executed_samples=[(9.2, .15, 0.), (9.55, .15, 0.),
                                          (9.8, .15, 0.), (10., .15, 0.)],
                        require_lane_status=True, released_at=8.)
        self.assertIsNone(MODULE.motion_fault(**baseline))
        for changes in ({"sharp_corner_enabled": False}, {"sharp_corner_state": "idle"},
                        {"sharp_corner_state": "reacquiring"}, {"yellow_boundary_visible": False},
                        {"white_boundary_visible": True}, {"red_stop": True},
                        {"fault": "corner deadline"}, {"camera_valid": False},
                        {"lane_diagnostic": "Lane lost: centre outside image"}):
            args = dict(baseline, status_sample=(9.95, dict(status, **changes)))
            with self.subTest(changes=changes):
                self.assertIsNotNone(MODULE.motion_fault(**args))
        for changes in ({"camera_fresh": False}, {"lane_running": False},
                        {"status_sample": (8., status)},
                        {"topic_publishers": ["other"]}):
            self.assertIsNotNone(MODULE.motion_fault(**dict(baseline, **changes)))
        stopped = [(9.3, 100), (9.55, 100), (9.8, 100), (10., 100)]
        self.assertIn("Left wheel stalled", MODULE.motion_fault(**dict(
            baseline, left_encoder_samples=stopped, right_encoder_samples=stopped)))

    def test_encoder_watchdog_allows_startup_and_intentional_slow_wheel(self):
        commands = [(9.25, .03, .15), (9.5, .03, .15),
                    (9.75, .03, .15), (10.0, .03, .15)]
        stopped = [(9.25, 100), (9.5, 100), (9.75, 100), (10.0, 100)]
        moving = [(9.25, 200), (9.5, 205), (9.75, 210), (10.0, 215)]
        self.assertIsNone(MODULE.encoder_motion_fault(
            10.0, commands, stopped, moving, released_at=8.0))
        self.assertIsNone(MODULE.encoder_motion_fault(
            8.5, commands, stopped, stopped, released_at=8.0))

    def test_encoder_watchdog_checks_outer_wheel_during_full_pivot(self):
        commands = [(9.25, .20, 0.0), (9.5, .20, 0.0),
                    (9.75, .20, 0.0), (10.0, .20, 0.0)]
        stopped = [(9.25, 100), (9.5, 100), (9.75, 100), (10.0, 100)]
        moving_left = [(9.25, 100), (9.5, 106),
                       (9.75, 112), (10.0, 118)]
        self.assertIn("Left wheel stalled", MODULE.encoder_motion_fault(
            10.0, commands, stopped, stopped, released_at=8.0))
        self.assertIsNone(MODULE.encoder_motion_fault(
            10.0, commands, moving_left, stopped, released_at=8.0))

    def test_stall_monitor_carries_command_across_bursty_refreshes(self):
        # Previously skipped: the in-window echo span is only 0.50 seconds,
        # despite a continuously active command across the 0.75-second window.
        commands = [(9.20, .15, 0.), (9.4, .15, 0.), (9.7, .15, 0.), (9.9, .15, 0.)]
        ticks = [(9.2, 6), (9.5, 6), (9.8, 6), (10., 6)]
        self.assertIn('Left wheel stalled', MODULE.encoder_motion_fault(
            10., commands, ticks, ticks, 8.))
        # A real stop/restart within the window must receive its startup grace.
        commands.insert(2, (9.6, 0., 0.))
        self.assertIsNone(MODULE.encoder_motion_fault(10., commands, ticks, ticks, 8.))

    def test_motor_register_readback_never_writes_or_initializes_outputs(self):
        class ReadOnlyBus:
            def __init__(self): self.calls = []
            def read_byte_data(self, address, register):
                self.calls.append((address, register))
                return 0
        bus = ReadOnlyBus()
        record = MODULE.read_motor_registers(bus)
        self.assertTrue(record['stable'])
        self.assertEqual(len(bus.calls), 38)
        self.assertTrue(all(address == 0x60 for address, _ in bus.calls))
        self.assertIn(str(0x06 + 4*8), record['registers'])
        self.assertIn(str(0x06 + 4*13), record['registers'])

    def test_motion_is_not_misreported_as_successful_turning(self):
        for left, right in ((132, 121), (139, 131)):
            result = MODULE.wheel_response(left, right, .2, .03)
            self.assertEqual(result['assessment'], 'weak_or_wrong_turn_response')
            self.assertEqual(result['physical_success'], 'unverified')
        self.assertEqual(MODULE.wheel_response(137, 0, .2, 0)['assessment'],
                         'encoder_turn_response_present')
        self.assertEqual(MODULE.wheel_response(216, 215, .15, .15)['assessment'],
                         'equal_command_comparison')
        self.assertEqual(MODULE.wheel_response(None, 0, .2, 0)['assessment'],
                         'unavailable')

    def test_readback_cleanup_bounds_a_hung_reader(self):
        from unittest.mock import Mock
        child = Mock()
        child.wait.side_effect = [MODULE.subprocess.TimeoutExpired('reader', 1), 0]
        log = Mock()
        MODULE.finish_reader(child, log)
        child.stdin.close.assert_called_once()
        child.kill.assert_called_once()
        log.close.assert_called_once()

    def test_motor_register_readback_marks_torn_reads_and_propagates_io_error(self):
        class ChangingBus:
            def __init__(self): self.count = 0
            def read_byte_data(self, address, register):
                self.count += 1
                return int(self.count > 19)
        self.assertFalse(MODULE.read_motor_registers(ChangingBus())['stable'])
        bus = SimpleNamespace(read_byte_data=lambda *_: (_ for _ in ()).throw(OSError('I2C failure')))
        with self.assertRaises(OSError):
            MODULE.read_motor_registers(bus)

    def test_motion_monitor_includes_encoder_stall(self):
        commands = [(9.25, .09, .09), (9.5, .09, .09),
                    (9.75, .09, .09), (10.0, .09, .09)]
        stopped = [(9.25, 100), (9.5, 100), (9.75, 100), (10.0, 100)]
        reason = MODULE.motion_fault(
            now=10.0, camera_fresh=True,
            status_sample=(9.9, {"camera_valid": True, "lane_error": 0.0,
                                 "red_stop": False, "fault": None}),
            lane_running=True, topic_publishers=["supervisor", "lane"],
            expected_publishers={"supervisor", "lane"},
            executed_samples=commands, require_lane_status=True,
            left_encoder_samples=stopped, right_encoder_samples=stopped,
            released_at=8.0,
        )
        self.assertIn("stalled while commanded", reason)

    def test_status_summary_uses_only_motion_window(self):
        samples = [
            (1.0, {"camera_age": .4, "wheel_speeds": [0, 0]}),
            (2.0, {"camera_age": .03, "wheel_speeds": [.03, .15]}),
            (2.5, {"camera_age": .08, "wheel_speeds": [.09, .09]}),
            (4.0, {"camera_age": .5, "wheel_speeds": [0, 0]}),
        ]
        result = MODULE.summarize_status_samples(samples, 1.5, 3.0)
        self.assertEqual(result["lane_status_samples"], 2)
        self.assertEqual(result["camera_age_range_s"], [.03, .08])
        self.assertEqual(result["requested_steering_range"], [0.0, .06])

    def test_stationary_red_summary_reports_geometry_without_distance_claim(self):
        samples = [
            (1.0, {"red_line_detection": None}),
            (1.1, {"red_line_detection": {
                "bottom_fraction": .70, "width_fraction": .52,
                "area_fraction": .02, "triggered": False}}),
            (1.2, {"red_line_detection": {
                "bottom_fraction": .74, "width_fraction": .60,
                "area_fraction": .03, "triggered": True}}),
        ]
        result = MODULE.summarize_red_line_samples(samples)
        self.assertEqual(result["valid_detection_samples"], 2)
        self.assertEqual(result["triggered_samples"], 1)
        self.assertEqual(result["bottom_fraction_range"], [.70, .74])
        self.assertAlmostEqual(result["median_bottom_fraction"], .72)
        self.assertNotIn("distance", result)


if __name__ == "__main__":
    unittest.main()
