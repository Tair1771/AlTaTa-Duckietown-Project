"""Prepare continuous route control over Windows SSH, without starting a route."""
import json
import pathlib
import shlex
import subprocess
import tarfile
import tempfile
import time
import uuid

from connect_companion import ROOT, ensure_tunnel, ssh

NAME = "duck2-companion-driving"


def launch_text():
    text = (ROOT / "launchers/lane-continuous.sh").read_text(encoding="utf-8")
    # Execute the exact launcher parameters in the installed runtime; package
    # sources are mounted read-only instead of requiring a robot-side build.
    text = text.replace("dt-launchfile-init", "export VEHICLE_NAME=duck2\n"
                        "export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_HOSTNAME=duck2.local\n"
                        "export PYTHONPATH=/app:${PYTHONPATH:-}")
    for name in ("lane_follower_node", "command_gateway", "camera_gateway",
                 "continuous_safety_watchdog"):
        text = text.replace("rosrun duckie_lane_follower " + name + ".py",
                            "python3 /app/" + name + ".py")
    if "rosrun " in text or "_require_client_heartbeat:=true" not in text:
        raise RuntimeError("Unsupported launcher layout; preparation aborted")
    return text


def status():
    code = "import urllib.request; print(urllib.request.urlopen('http://127.0.0.1:8765/status', timeout=3).read().decode())"
    return json.loads(ssh("docker exec " + NAME + " python3 -c " + shlex.quote(code)))


def verify_stopped(value):
    if (value.get("drive_enabled") is not True
            or value.get("manual_stop") is not True
            or value.get("state") != "awaiting_route"
            or value.get("wheel_speeds") != [0.0, 0.0]
            or value.get("require_client_heartbeat") is not True
            or value.get("wheel_publishers") != ["/lane_follower_node"]):
        raise RuntimeError("Expected stopped, awaiting-route controller with exclusive ownership")


def release_driver():
    inner = ("source /environment.sh >/dev/null 2>&1; "
             "export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_HOSTNAME=duck2.local; "
             "python3 /app/arm_driver_stopped.py")
    print(ssh("docker exec " + NAME + " bash -lc " + shlex.quote(inner)))


def main():
    if ssh("docker ps -a --filter name=^/" + NAME + "$ --format '{{.Names}}'"):
        running = ssh("docker inspect --format '{{.State.Running}}' " + NAME)
        if running == "true":
            # A stopped-looking container can still contain yesterday's source.
            # Verify that it is safe to replace, then redeploy the current files.
            verify_stopped(status())
            ssh("docker stop -t 10 " + NAME)
        if running != "false":
            # The only accepted non-false value above was a running container
            # that has now been stopped deliberately.
            running = ssh("docker inspect --format '{{.State.Running}}' " + NAME)
            if running != "false":
                raise RuntimeError("Cannot stop the existing driving container safely")
        ssh("docker rm " + NAME)
    config = json.loads((ROOT / "config/duck2.json").read_text())
    base = config["base_images"]["arm64v8"]
    ssh("docker image inspect " + shlex.quote(base) + " --format '{{.Architecture}}'")
    stage = "/tmp/duck2-driving-" + uuid.uuid4().hex[:12]
    source = ROOT / "packages/duckie_lane_follower/src"
    with tempfile.TemporaryDirectory() as temporary:
        folder = pathlib.Path(temporary)
        (folder / "start.sh").write_text(launch_text(), encoding="utf-8", newline="\n")
        archive = folder / "app.tar.gz"
        with tarfile.open(archive, "w:gz") as bundle:
            for path in source.rglob("*.py"):
                if "__pycache__" not in path.parts:
                    compile(path.read_text(encoding="utf-8"), str(path), "exec")
                    bundle.add(path, arcname=str(path.relative_to(source)))
            bundle.add(folder / "start.sh", arcname="start.sh")
            bundle.add(ROOT / "tools/arm_driver_stopped.py", arcname="arm_driver_stopped.py")
        ssh("mkdir -p " + stage)
        subprocess.run(["scp", "-o", "BatchMode=yes", str(archive),
                        "duck2:" + stage + "/app.tar.gz"], check=True, timeout=60)
        ssh("tar -xzf {0}/app.tar.gz -C {0}; bash -n {0}/start.sh".format(stage))
    # Stop preview before using the same node names and ports. Never stop drivers.
    if ssh("docker ps --filter name=^/duck2-companion-preview$ --format '{{.Names}}'"):
        ssh("docker stop -t 10 duck2-companion-preview")
    ssh("docker stop -t 10 car-interface")
    started = False
    try:
        ssh("docker run -d --name {0} --network host --restart no --pull never "
            "--tmpfs /tmp --tmpfs /root/.ros -e VEHICLE_NAME=duck2 "
            "-e PYTHONDONTWRITEBYTECODE=1 -v {1}:/app:ro --entrypoint bash "
            "{2} /app/start.sh".format(NAME, stage, shlex.quote(base)))
        started = True
        for _ in range(30):
            try:
                value = status()
                verify_stopped(value)
                break
            except (subprocess.CalledProcessError, ValueError, RuntimeError):
                time.sleep(1)
        else:
            raise RuntimeError("Driving startup did not reach the verified stopped state")
        ensure_tunnel()
        release_driver()
        print(json.dumps({key: value.get(key) for key in (
            "state", "drive_enabled", "manual_stop", "wheel_speeds",
            "wheel_publishers", "camera_valid", "camera_age")}, indent=2))
        print("Prepared only. No set_route, Continue, or Start was sent.")
    except BaseException as error:
        if isinstance(error, subprocess.CalledProcessError):
            print(error.stdout or "")
            print(error.stderr or "")
        if started:
            ssh("docker stop -t 10 " + NAME)
        print("Preparation failed. Temporary controller stopped; car-interface remains stopped.")
        raise


if __name__ == "__main__":
    main()
