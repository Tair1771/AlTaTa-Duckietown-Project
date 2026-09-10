"""Prepare or execute synthetic checks in a disposable, network-isolated container.

Default is a local plan only. --execute --target robot uses trusted Windows SSH.
No host devices, ports, directories, Docker socket, or robot ROS master are shared.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import tarfile
import tempfile
import uuid

ROOT = Path(__file__).resolve().parents[1]
LABEL = "duck2.synthetic-bench"
SSH = ["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
       "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=10",
       "-o", "ServerAliveCountMax=2", "duck2"]
BOOTSTRAP = r'''set -eo pipefail
tar --no-same-owner -xzf /tmp/bench.tar.gz -C /
export ROS_MASTER_URI=http://localhost:11311 ROS_HOSTNAME=localhost VEHICLE_NAME=duck2
export DT_REPO_PATH=/code/catkin_ws/src/duckiebot-ros DT_LAUNCH_PATH=/launch/duckiebot-ros
mkdir -p "$DT_REPO_PATH" "$DT_LAUNCH_PATH"
cp -r /project/packages "$DT_REPO_PATH/"
cp /project/launchers/*.sh "$DT_LAUNCH_PATH/"
chmod +x "$DT_REPO_PATH"/packages/duckie_lane_follower/src/*.py "$DT_LAUNCH_PATH"/*.sh
source /opt/ros/noetic/setup.bash
set -eo pipefail
catkin build --workspace /code/catkin_ws duckie_lane_follower --no-status -j1 -p1
dt-install-launchers "$DT_LAUNCH_PATH"
source /environment.sh
set -eo pipefail
export ROS_MASTER_URI=http://localhost:11311 ROS_HOSTNAME=localhost VEHICLE_NAME=duck2
export PYTHONPATH=/project/tests:/project/laptop:$DT_REPO_PATH/packages/duckie_lane_follower/src:${PYTHONPATH:-}
cd /project
python3 -m unittest test_live_chat test_live_navigation test_obstacles_and_connection
python3 /project/tests/test_ros_transport.py
echo BENCH_SYNTHETIC_PASS
'''


def create_args(name, image):
    return ["create", "--name", name, "--label", LABEL + "=1", "--pull", "never",
            "--network", "none", "--restart", "no", "--cap-drop", "ALL",
            "--security-opt", "no-new-privileges", "--memory", "1g", "--cpus", "1",
            "--pids-limit", "256", "--no-healthcheck", "--entrypoint", "/usr/bin/timeout",
            image, "--signal=TERM", "--kill-after=10", "480", "bash", "-lc", BOOTSTRAP]


def verify_isolation(info):
    host = info["HostConfig"]
    if (host.get("NetworkMode") != "none" or host.get("Privileged")
            or host.get("Binds") or info.get("Mounts") or host.get("Devices")
            or host.get("DeviceRequests") or host.get("VolumesFrom") or host.get("PortBindings")
            or host.get("PidMode") or host.get("IpcMode") == "host"
            or host.get("RestartPolicy", {}).get("Name") not in ("no", "")
            or "ALL" not in host.get("CapDrop", [])
            or not any(x.startswith("no-new-privileges") for x in host.get("SecurityOpt", []))
            or info["Config"].get("Labels", {}).get(LABEL) != "1"):
        raise RuntimeError("Container isolation verification failed; test not started")


def make_bundle(path):
    files = []
    for folder in ("packages", "laptop", "tests", "launchers", "config", "tools"):
        files.extend(p for p in (ROOT / folder).rglob("*") if p.is_file()
                     and p.suffix in (".py", ".sh", ".xml", ".txt", ".json")
                     and "__pycache__" not in p.parts)
    manifest = {}
    with tarfile.open(path, "w:gz") as archive:
        for source in sorted(files):
            rel = source.relative_to(ROOT).as_posix()
            if source.is_symlink():
                raise RuntimeError("Unexpected source symlink: " + rel)
            manifest[rel] = hashlib.sha256(source.read_bytes()).hexdigest()
            archive.add(source, arcname="project/" + rel, recursive=False)
    return manifest


class Target:
    def __init__(self, robot):
        self.robot = robot

    def run(self, args, timeout=30, check=True):
        command = SSH + [shlex.join(args)] if self.robot else args
        result = subprocess.run(command, capture_output=True, text=True,
                                encoding="utf-8", errors="replace", timeout=timeout)
        if check and result.returncode:
            raise RuntimeError(result.stdout + result.stderr)
        return result

    def docker(self, args, **kwargs):
        prefix = ["docker"] if self.robot else ["docker", "--context", "desktop-linux"]
        return self.run(prefix + args, **kwargs)


def checks_passed(state, output, production_watchdog=False):
    marker = "PRODUCTION_WATCHDOG_PASS" if production_watchdog else "PASS actual ROS: experimental pass/return"
    return (not state["Running"] and state["ExitCode"] == 0
            and "BENCH_SYNTHETIC_PASS" in output and marker in output)


def execute(robot, production_watchdog=False):
    target = Target(robot)
    config = json.loads((ROOT / "config/duck2.json").read_text())
    image = config["base_images"]["arm64v8" if robot else "amd64"]
    identifier = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:8]
    name = "duck2-synthetic-bench-" + identifier.lower()
    output = Path(os.environ.get("LOCALAPPDATA", Path.home())) / "Duck2" / "bench-checks" / identifier
    output.mkdir(parents=True)
    remote_dir = None
    created = False
    try:
        facts = target.docker(["image", "inspect", image]).stdout
        (output / "image.json").write_text(facts, encoding="utf-8")
        (output / "containers-before.txt").write_text(target.docker([
            "ps", "--format", "{{.ID}} {{.Names}} {{.Status}}"
        ]).stdout, encoding="utf-8")
        with tempfile.TemporaryDirectory() as temporary:
            bundle = Path(temporary) / "bench.tar.gz"
            manifest = make_bundle(bundle)
            (output / "sources.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
            archive_path = str(bundle)
            if robot:
                # Fixed unique /tmp path; cleanup removes these two exact entries only.
                remote_dir = "/tmp/" + name
                target.run(["mkdir", "-m", "700", remote_dir])
                subprocess.run(["scp", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
                                "-o", "ConnectTimeout=5", str(bundle),
                                "duck2:" + remote_dir + "/bench.tar.gz"], check=True, timeout=90)
                archive_path = remote_dir + "/bench.tar.gz"
            creation = create_args(name, image)
            if production_watchdog:
                # Production camera/controller processing uses roughly two ARM
                # cores. A one-core cap tests artificial CPU starvation instead
                # of the intended watchdog faults.
                creation[creation.index("--cpus")+1] = "2"
                creation[-1] = '''set -eo pipefail
tar --no-same-owner -xzf /tmp/bench.tar.gz -C /
source /environment.sh
set -eo pipefail
export ROS_MASTER_URI=http://localhost:11311 ROS_HOSTNAME=localhost VEHICLE_NAME=duck2
export DUCK2_ISOLATED_BENCH=1 PYTHONPATH=/project/tools:/project/laptop:/project/packages/duckie_lane_follower/src:${PYTHONPATH:-}
python3 /project/tools/verify_continuous_watchdog_ros.py
echo BENCH_SYNTHETIC_PASS
'''
            target.docker(creation)
            created = True
            info = json.loads(target.docker(["inspect", name]).stdout)[0]
            verify_isolation(info)
            (output / "isolation.json").write_text(json.dumps(info, indent=2), encoding="utf-8")
            target.docker(["cp", archive_path, name + ":/tmp/bench.tar.gz"], timeout=90)
            print("Verified isolation. Running synthetic checks only; report: " + str(output), flush=True)
            result = target.docker(["start", "-a", name], timeout=520, check=False)
            (output / "checks.log").write_text(result.stdout + result.stderr, encoding="utf-8")
            state = json.loads(target.docker(["inspect", name]).stdout)[0]["State"]
            passed = checks_passed(state, result.stdout, production_watchdog)
            (output / "result.json").write_text(json.dumps({"passed": passed, "state": state,
                 "target": "robot" if robot else "local", "physical_tests": False,
                 "production_watchdog": production_watchdog}, indent=2), encoding="utf-8")
            if not passed:
                raise RuntimeError("Synthetic check failed. Read " + str(output / "checks.log"))
            print("PASS: synthetic ROS/session tests; no hardware access. " + str(output), flush=True)
    finally:
        if created:
            # Only this invocation's named container is eligible for cleanup.
            info = json.loads(target.docker(["inspect", name]).stdout)[0]
            verify_isolation(info)
            logs = target.docker(["logs", name], check=False)
            (output / "container.log").write_text(logs.stdout + logs.stderr, encoding="utf-8")
            target.docker(["rm", "-f", name])
        if remote_dir:
            target.run(["rm", "-f", remote_dir + "/bench.tar.gz"])
            target.run(["rmdir", remote_dir])
        if created:
            (output / "containers-after.txt").write_text(target.docker([
                "ps", "--format", "{{.ID}} {{.Names}} {{.Status}}"
            ]).stdout, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--target", choices=("local", "robot"), default="local")
    parser.add_argument("--production-watchdog", action="store_true",
                        help="Exercise the production launcher with isolated fake hardware")
    args = parser.parse_args()
    if not args.execute:
        print("PLAN ONLY: isolated synthetic ROS tests, no hardware. Use --execute --target local|robot.")
        return
    execute(args.target == "robot", args.production_watchdog)


if __name__ == "__main__":
    main()
