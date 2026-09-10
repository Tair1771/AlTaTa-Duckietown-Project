"""Start a stationary app connection using Windows SSH; never enable driving."""
import json
import pathlib
import shlex
import subprocess
import tarfile
import tempfile
import time
import urllib.request
import uuid

ROOT = pathlib.Path(__file__).resolve().parents[1]
NAME = "duck2-companion-preview"


def remote_status():
    code = "import urllib.request; print(urllib.request.urlopen('http://127.0.0.1:8765/status', timeout=3).read().decode())"
    return json.loads(ssh("docker exec " + NAME + " python3 -c " + shlex.quote(code)))


def ensure_tunnel():
    def reachable():
        # A working command port does not prove the camera port was forwarded.
        from urllib.error import HTTPError
        with urllib.request.urlopen("http://127.0.0.1:8765/status", timeout=3) as response:
            value = json.load(response)
            if not isinstance(value, dict) or "state" not in value:
                raise RuntimeError("Unexpected service on command port")
        try:
            with urllib.request.urlopen("http://127.0.0.1:8766/camera?view=normal", timeout=3) as response:
                if response.headers.get_content_type() != "image/x-portable-pixmap":
                    raise RuntimeError("Unexpected service on camera port")
        except HTTPError as error:
            # Camera service is reachable even while waiting for its first frame.
            if error.code != 503:
                raise
    try:
        reachable()
        return
    except Exception:
        pass
    tunnel = subprocess.Popen(["ssh", "-N", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes", "-o", "ExitOnForwardFailure=yes",
                      "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=2",
                      "-L", "8765:127.0.0.1:8765", "-L", "8766:127.0.0.1:8766", "duck2"],
                     creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    for _ in range(10):
        if tunnel.poll() is not None:
            raise RuntimeError("SSH tunnel exited. Check for an old tunnel occupying ports 8765/8766; close that tunnel before retrying.")
        time.sleep(0.5)
        try:
            reachable()
            return
        except Exception:
            pass
    tunnel.terminate()
    raise RuntimeError("Gateway is running, but local tunnel failed. Check ports 8765/8766.")


def ssh(command):
    try:
        return subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes", "-o", "ConnectTimeout=8",
                               "duck2", command], check=True, capture_output=True,
                              text=True, timeout=60).stdout.strip()
    except subprocess.CalledProcessError as error:
        # Preserve the exception type used by startup polling while exposing
        # the actual remote diagnostic instead of a command-only traceback.
        print((error.stderr or error.stdout or "SSH request failed").strip(), flush=True)
        raise


def reuse_preview():
    if ssh("docker ps --filter name=^/duck2-companion-driving$ --format '{{.Names}}'"):
        raise RuntimeError("Driving service is running. End that session before preparing a stationary preview.")
    existing = ssh("docker ps -a --filter name=^/" + NAME + "$ --format '{{.Names}}'")
    if not existing:
        return False
    running = ssh("docker inspect --format '{{.State.Running}}' " + NAME)
    if running == "false":
        # Never restart yesterday's source or its lost /tmp mount after reboot.
        # Remove only this stopped temporary preview container, preserving images.
        ssh("docker rm " + NAME)
        return False
    if running != "true":
        raise RuntimeError("Cannot establish preview state; left untouched")
    status = remote_status()
    if status.get("drive_enabled") is not False:
        raise RuntimeError("Existing service is not a stationary preview; left untouched.")
    ensure_tunnel()
    print("Existing preview connected. Click Connect to duck2 and Start viewing. Driving is disabled.")
    return True


def main():
    if reuse_preview():
        return
    stage = "/tmp/duck2-companion-" + uuid.uuid4().hex[:12]
    source = ROOT / "packages/duckie_lane_follower/src"
    # Keep the actual wheel topic untouched even if someone presses Start.
    launch = """#!/bin/bash
set -eo pipefail
source /environment.sh
export VEHICLE_NAME=duck2 ROS_MASTER_URI=http://127.0.0.1:11311 ROS_HOSTNAME=duck2.local
export PYTHONPATH=/app:${PYTHONPATH:-}
pids=""
cleanup() { trap - INT TERM EXIT; for pid in $pids; do kill -INT "$pid" 2>/dev/null || true; done; wait || true; }
trap cleanup INT TERM EXIT
python3 /app/lane_follower_node.py _drive_enabled:=false _show_debug:=false _route_enabled:=true _require_client_heartbeat:=true _obstacle_enabled:=false /duck2/wheels_driver_node/wheels_cmd:=/duck2/lane_follower/diagnostic_wheels_cmd &
pids="$pids $!"
python3 /app/command_gateway.py &
pids="$pids $!"
python3 /app/camera_gateway.py &
pids="$pids $!"
wait -n $pids
"""
    with tempfile.TemporaryDirectory() as temporary:
        folder = pathlib.Path(temporary)
        (folder / "start.sh").write_text(launch, encoding="utf-8", newline="\n")
        archive = folder / "preview.tar.gz"
        with tarfile.open(archive, "w:gz") as bundle:
            for path in source.rglob("*.py"):
                if "__pycache__" not in path.parts:
                    bundle.add(path, arcname=str(path.relative_to(source)))
            bundle.add(folder / "start.sh", arcname="start.sh")
        ssh("mkdir -p " + shlex.quote(stage))
        subprocess.run(["scp", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes", str(archive),
                        "duck2:" + stage + "/preview.tar.gz"], check=True, timeout=60)
        ssh("tar -xzf {0}/preview.tar.gz -C {0}".format(shlex.quote(stage)))
    base = json.loads((ROOT / "config/duck2.json").read_text())["base_images"]["arm64v8"]
    ssh("docker image inspect " + shlex.quote(base) + " --format '{{.Architecture}}'")
    ssh("docker run -d --name {0} --network host --restart no --pull never "
        "--tmpfs /tmp --tmpfs /root/.ros -e VEHICLE_NAME=duck2 "
        "-e PYTHONDONTWRITEBYTECODE=1 -v {1}:/app:ro --entrypoint bash "
        "{2} /app/start.sh".format(NAME, stage, shlex.quote(base)))
    for _ in range(20):
        try:
            status = remote_status()
            print("Robot gateway ready:", status.get("state"), flush=True)
            break
        except (subprocess.CalledProcessError, ValueError):
            time.sleep(1)
    else:
        print(ssh("docker logs --tail 35 " + NAME))
        raise RuntimeError("Preview startup failed; inspect logs above")
    ensure_tunnel()
    print("Click Connect to duck2 and Start viewing. Driving is disabled in this preview.")


if __name__ == "__main__":
    main()
