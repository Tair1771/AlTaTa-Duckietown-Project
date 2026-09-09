#!/usr/bin/env python3
"""Run one explicitly authorized duck2 ground test and preserve its evidence.

This Windows-side tool is intentionally limited to reviewed bounded profiles.
It stages temporary source, gives the test exclusive wheel control,
runs the bounded robot-side supervisor, restores normal control, and downloads
an evidence archive.  Robot-side evidence is never deleted by this tool.
"""

import argparse
import base64
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tarfile
import time
import uuid
from datetime import datetime, timezone


SAFE_NAME = re.compile(r"^[a-zA-Z0-9_.-]+$")
FRAME_NAME = re.compile(r"^frame-[0-9]{5}\.jpg$")
TEST_NODES = {
    "/duck2_bounded_ground_supervisor",
    "/duck2_ground_watchdog",
    "/lane_follower_node",
}
MAX_DURATION = 15.0
SSH_OPTIONS = ["-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
               "-o", "ConnectTimeout=8"]


class SessionError(RuntimeError):
    pass


def safe_name(value, label):
    if not SAFE_NAME.fullmatch(value):
        raise ValueError("{} contains unsupported characters".format(label))
    return value


def make_run_id(label, now=None, suffix=None):
    cleaned = re.sub(r"[^a-z0-9]+", "-", label.lower()).strip("-")
    if not cleaned:
        raise ValueError("Evidence label must contain a letter or number")
    stamp = (now or datetime.now(timezone.utc)).strftime("%Y%m%dT%H%M%SZ")
    token = suffix or uuid.uuid4().hex[:8]
    return safe_name("{}-{}-{}".format(stamp, cleaned, token), "run id")


def quote(value):
    return shlex.quote(str(value))


class Runner:
    def __init__(self, alias="duck2"):
        self.alias = safe_name(alias, "SSH alias")

    def command(self, argv, timeout=30, check=True):
        try:
            result = subprocess.run(
                argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, timeout=timeout,
            )
        except subprocess.TimeoutExpired as error:
            raise SessionError("SSH check timed out; confirm hotspot and robot boot completion. "
                               "No automatic retry will run.") from error
        except OSError as error:
            raise SessionError("Windows OpenSSH could not start: {}".format(error)) from error
        if check and result.returncode != 0:
            raise SessionError(
                "Command failed ({}): {}".format(
                    result.returncode, result.stdout.strip()))
        return result

    def remote(self, command, timeout=30, check=True):
        return self.command(
            ["ssh"] + SSH_OPTIONS + [self.alias, command],
            timeout=timeout, check=check,
        )

    def check_readiness(self):
        """One read-only SSH call before upload or controller handover."""
        source = '''import json, subprocess
def run(args):
    return subprocess.check_output(args, timeout=15, universal_newlines=True)
containers = json.loads(run(["docker", "inspect", "duckiebot-interface", "car-interface"]))
state = run(["docker", "exec", "duckiebot-interface", "bash", "-lc",
    "source /environment.sh >/dev/null 2>&1; set -e; rosnode list; "
    "rostopic info /duck2/wheels_driver_node/wheels_cmd"])
print(json.dumps({"hostname": run(["hostname"]).strip(),
    "boot_id": open("/proc/sys/kernel/random/boot_id").read().strip(),
    "containers": {c["Name"].lstrip("/"): {"running": c["State"]["Running"],
        "image": c["Image"], "started_at": c["State"]["StartedAt"]}
        for c in containers}, "ros": state}))
'''
        payload = base64.b64encode(source.encode()).decode("ascii")
        result = self.remote("echo {} | base64 -d | python3".format(payload),
                             timeout=40, check=False)
        if result.returncode:
            detail = result.stdout.strip()
            lowered = detail.lower()
            if "host key" in lowered or "identification has changed" in lowered:
                hint = "Host identity mismatch: verify the trusted robot key; do not bypass it."
            elif "permission denied" in lowered:
                hint = "Unlock the Windows duck2 key with ssh-add; do not regenerate it."
            elif "resolve hostname" in lowered:
                hint = "Check the Windows duck2 SSH alias and hotspot hostname."
            else:
                hint = "Check hotspot/boot completion and the reported container or ROS error."
            raise SessionError("Read-only startup check failed. {}\n{}".format(hint, detail))
        try:
            facts = json.loads(result.stdout.strip().splitlines()[-1])
            validate_readiness(facts)
        except (ValueError, KeyError, IndexError, TypeError) as error:
            raise SessionError("Invalid startup check response: {}".format(error))
        return facts

    def stage_sources(self, sources, destination, timeout=30):
        """Upload the reviewed test sources and their local package in one SSH call."""
        payload = io.BytesIO()
        with tarfile.open(fileobj=payload, mode="w:gz") as archive:
            for item in sources:
                if isinstance(item, tuple):
                    source, arcname = item
                else:
                    source, arcname = item, item.name
                arcname = str(arcname).replace("\\", "/")
                path = Path(arcname)
                if (path.is_absolute() or ".." in path.parts
                        or not arcname or arcname.startswith("/")):
                    raise ValueError("Unsafe staged source path: {}".format(arcname))
                archive.add(str(source), arcname=arcname, recursive=False)
        script = "mkdir {path} && tar -xzf - -C {path}".format(path=quote(destination))
        command = "docker exec -i duckiebot-interface sh -c {}".format(quote(script))
        result = subprocess.run(
            ["ssh"] + SSH_OPTIONS + [self.alias, command], input=payload.getvalue(),
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout,
        )
        if result.returncode:
            raise SessionError("Source upload failed: {}".format(
                result.stdout.decode("utf-8", errors="replace")))

    def export_tar(self, remote_directory, destination, timeout=90):
        remote_command = "docker exec duckiebot-interface tar -C {} -czf - .".format(
            quote(remote_directory))
        with destination.open("wb") as output:
            result = subprocess.run(
                ["ssh"] + SSH_OPTIONS + [self.alias, remote_command],
                stdout=output, stderr=subprocess.PIPE, timeout=timeout,
            )
        if result.returncode != 0:
            detail = result.stderr.decode("utf-8", errors="replace").strip()
            raise SessionError("Evidence download failed: {}".format(detail))


def ros_command(inner):
    script = "source /environment.sh >/dev/null 2>&1; {}".format(inner)
    return "docker exec duckiebot-interface bash -lc {}".format(quote(script))


def parse_nodes(output):
    return {line.strip() for line in output.splitlines()
            if line.strip().startswith("/")}


def parse_topic_publishers(output):
    publishers = set()
    in_publishers = False
    for raw in output.splitlines():
        line = raw.strip()
        if line == "Publishers:":
            in_publishers = True
            continue
        if in_publishers and line.endswith(":"):
            break
        if in_publishers and line.startswith("*"):
            publishers.add(line[1:].strip().split()[0])
    return publishers


def validate_readiness(facts):
    if facts["hostname"] != "duck2":
        raise SessionError("SSH alias reached a different robot; expected duck2")
    for name in ("duckiebot-interface", "car-interface"):
        if facts["containers"][name]["running"] is not True:
            raise SessionError("{} is stopped; inspect it before a movement session".format(name))
    nodes = parse_nodes(facts["ros"])
    required = {"/duck2/camera_node", "/duck2/wheels_driver_node", "/duck2/kinematics_node"}
    if not required <= nodes or nodes & TEST_NODES:
        raise SessionError("ROS startup is incomplete or a temporary controller remains active")
    if parse_topic_publishers(facts["ros"]) != {"/duck2/kinematics_node"}:
        raise SessionError("Unexpected wheel publisher before test; inspect controller ownership")


def assert_temporary_nodes_stopped(runner):
    list_nodes = ros_command("rosnode list")
    nodes = parse_nodes(runner.remote(list_nodes).stdout)
    remaining = nodes & TEST_NODES
    if remaining:
        kill = "rosnode kill {}".format(" ".join(quote(node) for node in sorted(remaining)))
        runner.remote(ros_command(kill), timeout=20, check=False)
        time.sleep(0.5)
        nodes = parse_nodes(runner.remote(list_nodes).stdout)
        remaining = nodes & TEST_NODES
    if remaining:
        raise SessionError(
            "Temporary ROS nodes are still running; normal control was not restored: {}".format(
                ", ".join(sorted(remaining))))


def restore_normal_control(runner, attempts=12, sleep=time.sleep, stop_confirmed=False):
    """Restore only after test nodes exit, then verify sole normal ownership."""
    assert_temporary_nodes_stopped(runner)
    if not stop_confirmed:
        raise SessionError(
            "No confirmed final zero feedback; normal control remains stopped for review")
    runner.remote("docker start car-interface", timeout=30)
    last_detail = "not checked"
    for _ in range(attempts):
        state = runner.remote(
            "docker inspect -f '{{.State.Running}}' car-interface && " +
            ros_command("rostopic info /duck2/wheels_driver_node/wheels_cmd"),
            timeout=15, check=False,
        )
        publishers = parse_topic_publishers(state.stdout) if state.returncode == 0 else set()
        running = state.stdout.strip().splitlines()[:1] == ["true"]
        if state.returncode == 0 and running and publishers == {
                "/duck2/kinematics_node"}:
            return publishers
        last_detail = "container={!r}, publishers={}".format(
            state.stdout.strip(), sorted(publishers))
        sleep(1.0)
    raise SessionError("Normal wheel ownership was not restored: {}".format(last_detail))


def safe_extract_tar(archive_path, destination):
    destination.mkdir(parents=True, exist_ok=False)
    root = destination.resolve()
    with tarfile.open(str(archive_path), "r:*") as archive:
        members = archive.getmembers()
        for member in members:
            if not (member.isdir() or member.isfile()):
                raise SessionError("Evidence archive contains a link or special file")
            target = (root / member.name).resolve()
            try:
                target.relative_to(root)
            except ValueError:
                raise SessionError("Evidence archive contains an unsafe path")
        if hasattr(tarfile, "data_filter"):
            archive.extractall(str(root), filter="data")
        else:
            archive.extractall(str(root))


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        while True:
            block = source.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def validate_evidence(directory):
    required = ["frames.jsonl", "telemetry.json", "completion.json"]
    for name in required:
        path = directory / name
        if not path.is_file() or path.stat().st_size == 0:
            raise SessionError("Evidence is missing {}".format(name))

    rows = []
    for line_number, line in enumerate(
            (directory / "frames.jsonl").read_text(encoding="utf-8").splitlines(), 1):
        try:
            row = json.loads(line)
        except ValueError as error:
            raise SessionError("Invalid frame manifest line {}: {}".format(
                line_number, error))
        filename = row.get("file")
        if not isinstance(filename, str) or not FRAME_NAME.fullmatch(filename):
            raise SessionError("Unsafe frame name on manifest line {}".format(line_number))
        frame = directory / filename
        if not frame.is_file() or frame.stat().st_size == 0:
            raise SessionError("Manifest frame is missing or empty: {}".format(filename))
        rows.append(row)

    completion = json.loads(
        (directory / "completion.json").read_text(encoding="utf-8"))
    telemetry = json.loads(
        (directory / "telemetry.json").read_text(encoding="utf-8"))
    summary = completion.get("summary")
    if completion.get("complete") is not True or not isinstance(summary, dict):
        raise SessionError("Evidence completion marker is invalid")
    if summary.get("evidence_frames") != len(rows):
        raise SessionError("Frame count does not match the completion record")
    if summary.get("final_feedback_all_zero") is not True:
        raise SessionError("Completion record does not confirm final zero feedback")
    if summary.get("post_stop_zero_confirmed") is not True:
        raise SessionError("Completion record does not confirm the post-stop zero window")
    for key in ("requested_wheels", "executed_wheels", "lane_status"):
        if not isinstance(telemetry.get(key), list):
            raise SessionError("Telemetry field is invalid: {}".format(key))

    files = required + [row["file"] for row in rows]
    verification = {
        "verified": True,
        "verified_utc": datetime.now(timezone.utc).isoformat(),
        "frame_count": len(rows),
        "files_sha256": {name: sha256(directory / name) for name in files},
        "summary": summary,
    }
    destination = directory / "evidence_verified.json"
    destination.write_text(
        json.dumps(verification, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return verification


def update_record(path, record):
    path.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8")


def summary_from_log(output):
    """Extract the structured result despite ROS warnings or a later traceback."""
    summary = None
    for line in output.splitlines():
        try:
            value = json.loads(line)
        except ValueError:
            continue
        if isinstance(value, dict) and "post_stop_zero_confirmed" in value:
            summary = value
    return summary


def confirmed_stop_from_log(output):
    """Use the supervisor's fresh stop window, never absence of wheel messages."""
    summary = summary_from_log(output)
    return bool(summary and summary.get("post_stop_zero_confirmed") is True
                and summary.get("final_feedback_all_zero") is True)


def diagnostic_profile(args):
    """Return the explicitly selected bounded diagnostic profile."""
    selected = []
    if getattr(args, "ground_right_pivot", False):
        selected.append(("ground_right_pivot",
                         ["--fixed-left", "0.15", "--fixed-right", "0.0"]))
    if getattr(args, "ground_rolling_right", False):
        selected.append(("ground_rolling_right",
                         ["--fixed-left", "0.15", "--fixed-right", "0.03"]))
    if getattr(args, "ground_strong_rolling_right", False):
        selected.append(("ground_strong_rolling_right",
                         ["--fixed-left", "0.20", "--fixed-right", "0.03"]))
    if getattr(args, "ground_equal_wheels", False):
        selected.append(("ground_equal_wheels",
                         ["--fixed-left", "0.15", "--fixed-right", "0.15"]))
    if getattr(args, "upside_down_left_pivot", False):
        selected.append(("upside_down_left_pivot",
                         ["--fixed-left", "0.15", "--fixed-right", "0.0"]))
    if getattr(args, "upside_down_load_profile", False):
        selected.append(("upside_down_load_profile",
                         ["--fixed-left", "0.20", "--fixed-right", "0.03"]))
    if getattr(args, "junction_turn", None):
        selected.append(("junction_{}".format(args.junction_turn),
                         ["--junction-turn", args.junction_turn,
                          "--red-stop-trigger-bottom-fraction",
                          str(args.red_stop_trigger_bottom_fraction)]))
    if getattr(args, "inspect_red_line", False):
        selected.append(("red_line_inspection", [
            "--inspect-red-line",
            "--red-stop-trigger-bottom-fraction",
            str(args.red_stop_trigger_bottom_fraction),
        ]))
    if len(selected) > 1:
        raise ValueError("Select only one diagnostic profile")
    return selected[0] if selected else ("camera_guided_curve",
                                         ["--camera-guided-curve"])


def run_session(args):
    if (not getattr(args, "confirm_go", False)
            and not getattr(args, "inspect_red_line", False)):
        raise ValueError("Physical movement requires --confirm-go after the user's fresh Go")
    if (type(args.duration) is bool or not math.isfinite(args.duration)
            or not 0 < args.duration <= MAX_DURATION):
        raise ValueError("Duration must be in (0, 15.0] seconds")
    profile_name, profile_args = diagnostic_profile(args)
    if (profile_name != "camera_guided_curve"
            and not profile_name.startswith("junction_")
            and profile_name != "red_line_inspection"
            and args.duration > 2.0):
        raise ValueError("Diagnostic duration must be at most 2.0 seconds")
    if profile_name == "red_line_inspection" and args.duration > 3.0:
        raise ValueError("Red-line inspection must be at most 3.0 seconds")

    repository = Path(__file__).resolve().parents[1]
    supervisor = repository / "tools" / "bounded_ground_supervisor.py"
    lane_node = repository / "packages" / "duckie_lane_follower" / "src" / "lane_follower_node.py"
    lane_package = lane_node.parent / "duckie_lane_follower"
    route_map = lane_package / "route_map.py"
    package_init = lane_package / "__init__.py"
    if not all(path.is_file() for path in (supervisor, lane_node, route_map, package_init)):
        raise SessionError("Required project source is missing")

    run_id = make_run_id(args.label)
    local_root = Path(args.output_root).expanduser().resolve()
    if local_root == repository or repository in local_root.parents:
        raise ValueError("Evidence must be stored outside the repository")
    local_directory = local_root / run_id
    local_directory.mkdir(parents=True, exist_ok=False)
    record_path = local_directory / "session.json"
    archive_path = local_directory / "robot-evidence.tar.gz"
    robot_stage = "/tmp/duck2-ground-stage-{}".format(run_id)
    evidence_dir = "/tmp/duck2-ground-evidence-{}".format(run_id)
    runner = Runner(args.ssh_alias)
    record = {
        "run_id": run_id,
        "label": args.label,
        "profile": profile_name,
        "requested_duration_s": args.duration,
        "robot_evidence_dir": evidence_dir,
        "robot_evidence_retained": True,
        "status": "preparing",
    }
    update_record(record_path, record)
    errors = []
    supervisor_result = None
    control_handover_attempted = False
    stage_finished = False
    timings = record["phase_seconds"] = {}

    try:
        print("Checking Windows SSH, installed containers and normal ROS ownership.", flush=True)
        record["startup"] = runner.check_readiness()
        record["source_sha256"] = {str(path.relative_to(repository)): sha256(path)
                                    for path in (supervisor, lane_node, route_map, package_init)}
        print("Uploading test sources (one connection); movement has not started.", flush=True)
        phase_start = time.monotonic()
        runner.stage_sources([
            (supervisor, supervisor.name),
            (lane_node, lane_node.name),
            (package_init, "duckie_lane_follower/__init__.py"),
            (route_map, "duckie_lane_follower/route_map.py"),
        ], robot_stage)
        timings["source_upload"] = time.monotonic() - phase_start
        stage_finished = True
        control_handover_attempted = True
        runner.remote("docker stop car-interface", timeout=30)
        record["status"] = "running"
        update_record(record_path, record)
        inner = " ".join([
            "VEHICLE_NAME=duck2", "python3",
            quote(robot_stage + "/bounded_ground_supervisor.py"),
        ] + profile_args + [
            "--duration", quote(args.duration),
            "--node-path", quote(robot_stage + "/lane_follower_node.py"),
            "--record-dir", quote(evidence_dir),
            "--acknowledge-upright-clear",
        ])
        print("Starting bounded preflight and authorized motion window.", flush=True)
        phase_start = time.monotonic()
        # Run the outer timeout INSIDE the container; timing out docker exec
        # on the host alone does not reliably terminate its container process.
        outer_timeout = math.ceil(args.duration) + 30
        command = ros_command("timeout --signal=INT --kill-after=5s {}s bash -c {}".format(
            outer_timeout, quote(inner)))
        supervisor_result = runner.remote(command, timeout=outer_timeout + 10, check=False)
        timings["preflight_motion_shutdown"] = time.monotonic() - phase_start
        (local_directory / "supervisor.log").write_text(
            supervisor_result.stdout, encoding="utf-8")
        result_summary = summary_from_log(supervisor_result.stdout)
        record["motion_summary"] = result_summary
        if supervisor_result.returncode != 0 or "SUPERVISOR_COMPLETE" not in supervisor_result.stdout:
            reason = result_summary.get("early_stop_reason") if result_summary else None
            errors.append("Motion stopped: {}".format(reason) if reason else
                          "Bounded supervisor did not complete successfully; inspect supervisor.log")
    except (OSError, SessionError, subprocess.TimeoutExpired, KeyboardInterrupt) as error:
        errors.append("Test execution: {}".format(error))
    finally:
        try:
            if control_handover_attempted:
                phase_start = time.monotonic()
                print("Verifying test exit and restoring normal ownership.", flush=True)
                stop_confirmed = (supervisor_result is not None and
                                  confirmed_stop_from_log(supervisor_result.stdout))
                record["stop_confirmed"] = stop_confirmed
                restored = restore_normal_control(runner, stop_confirmed=stop_confirmed)
                record["restored_publishers"] = sorted(restored)
                timings["restore_control"] = time.monotonic() - phase_start
        except (OSError, SessionError, subprocess.TimeoutExpired) as error:
            errors.append("Control restoration: {}".format(error))
        try:
            if stage_finished and control_handover_attempted:
                phase_start = time.monotonic()
                print("Downloading compressed evidence; robot copy retained.", flush=True)
                runner.export_tar(evidence_dir, archive_path)
                incoming = local_directory / "downloaded-evidence"
                safe_extract_tar(archive_path, incoming)
                verification = validate_evidence(incoming)
                record["evidence_verified"] = True
                record["frame_count"] = verification["frame_count"]
                timings["download_and_verify"] = time.monotonic() - phase_start
        except (OSError, SessionError, ValueError, tarfile.TarError,
                subprocess.TimeoutExpired) as error:
            errors.append("Evidence preservation: {}".format(error))
            record["evidence_verified"] = False

    record["status"] = "complete" if not errors else "attention_required"
    record["errors"] = errors
    update_record(record_path, record)
    print(json.dumps(record, indent=2, sort_keys=True))
    if errors:
        raise SessionError("; ".join(errors))
    return 0


def default_output_root():
    base = os.environ.get("LOCALAPPDATA")
    if not base:
        raise SessionError("LOCALAPPDATA is unavailable; pass --output-root")
    return str(Path(base) / "Duck2" / "evidence")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--label", required=True,
                        help="Short description such as left-curve-to-straight")
    parser.add_argument("--duration", type=float, default=15.0,
                        help="Motion window in seconds, maximum 15 (default 15)")
    parser.add_argument("--ground-right-pivot", action="store_true",
                        help="Reviewed upright diagnostic: turn right with left 0.15, right 0.00; maximum 2 seconds")
    parser.add_argument("--ground-rolling-right", action="store_true",
                        help="Reviewed upright diagnostic: rolling right with left 0.15, right 0.03; maximum 2 seconds")
    parser.add_argument("--ground-strong-rolling-right", action="store_true",
                        help="Reviewed upright diagnostic: stronger rolling right with left 0.20, right 0.03; maximum 2 seconds")
    parser.add_argument("--ground-equal-wheels", action="store_true",
                        help="Reviewed upright diagnostic: both wheels 0.15; maximum 2 seconds")
    parser.add_argument("--upside-down-left-pivot", action="store_true",
                        help="Reviewed lifted-wheel diagnostic: left 0.15, right 0.00; maximum 2 seconds")
    parser.add_argument("--upside-down-load-profile", action="store_true",
                        help="Lifted-wheel check for the load profile: left 0.20, right 0.03; maximum 2 seconds")
    parser.add_argument("--junction-turn", choices=("straight", "left", "right"),
                        help="Supervised one-intersection test; maximum 15 seconds")
    parser.add_argument("--red-stop-trigger-bottom-fraction", type=float,
                        default=0.65,
                        help="Calibrated red-line proximity threshold for a junction test")
    parser.add_argument("--inspect-red-line", action="store_true",
                        help="Stationary camera/status sample; no movement and no Go required")
    parser.add_argument("--ssh-alias", default="duck2")
    parser.add_argument("--output-root")
    parser.add_argument("--check-only", action="store_true",
                        help="Read-only startup check; no upload, camera subscription or movement")
    parser.add_argument("--confirm-go", action="store_true",
                        help="Confirm the user gave a fresh Go for this run")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if sys.platform != "win32":
        raise SessionError("Use tools/Start-Duck2-GroundTest.ps1 from Windows PowerShell. "
                           "WSL does not share the verified Windows SSH agent or routing.")
    if args.check_only:
        Runner(args.ssh_alias).check_readiness()
        print("STARTUP_OK: key login, containers, ROS nodes and normal wheel ownership verified.")
        return 0
    if args.output_root is None:
        args.output_root = default_output_root()
    return run_session(args)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ValueError, SessionError) as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        sys.exit(1)
