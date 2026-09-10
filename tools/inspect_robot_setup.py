"""Read-only duck2 connection checks for Windows OpenSSH.

No mode subscribes to ROS topics, publishes messages, calls services, or
modifies the robot. --connect performs the compact check; --full adds runtime
and message-definition inspection.
"""
import argparse
import base64
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
QUICK_REMOTE = r'''
import subprocess

def run(args):
    result = subprocess.run(args, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, universal_newlines=True, timeout=20)
    if result.returncode:
        print("REMOTE_ERROR:", " ".join(args))
        print(result.stdout.strip())
        raise SystemExit(result.returncode)
    return result.stdout.strip()

print("CHECK: hostname")
print(run(["hostname"]))
print("CHECK: essential containers")
print(run(["docker", "ps", "--format", "{{.Names}} {{.Image}} {{.Status}}"]))
print("CHECK: ROS master and normal wheel publisher")
print(run(["docker", "exec", "ros", "bash", "-lc",
           "source /environment.sh >/dev/null 2>&1; "
           "rosnode list | grep -E '^/duck2/(camera_node|kinematics_node|wheels_driver_node)$'; "
           "rostopic info /duck2/wheels_driver_node/wheels_cmd"]))
'''

FULL_REMOTE = QUICK_REMOTE + r'''
print("CHECK: runtime and message definitions")
print(run(["uname", "-m"]))
print(run(["cat", "/etc/os-release"]))
print(run(["docker", "exec", "ros", "bash", "-lc",
           "source /environment.sh >/dev/null 2>&1; rosversion -d; "
           "rosmsg md5 sensor_msgs/CompressedImage; "
           "rosmsg md5 duckietown_msgs/WheelsCmdStamped; "
           "rostopic info /duck2/camera_node/image/compressed"]))
'''

FAILURE_HINTS = (
    ("host identity", ("host key verification failed", "remote host identification has changed"),
     "Stop. Confirm duck2's host key from trusted robot information before retrying."),
    ("login", ("permission denied", "no supported authentication methods"),
     "Unlock the duck2 SSH key with ssh-add, then retry."),
    ("network", ("connection timed out", "operation timed out", "connection refused", "no route to host"),
     "Confirm the hotspot and wait for duck2 to finish booting."),
)


def config():
    return json.loads((ROOT / "config/duck2.json").read_text())


def validate_host(host):
    if not re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9.-]*", host):
        raise ValueError("Use a hostname or IPv4 address")


def remote_source(mode):
    return FULL_REMOTE if mode == "full" else QUICK_REMOTE


def ssh_arguments(host, user, mode="quick", target=None):
    validate_host(host)
    if not re.fullmatch(r"[a-zA-Z_][a-zA-Z0-9_-]*", user):
        raise ValueError("Use a simple SSH username")
    payload = base64.b64encode(remote_source(mode).encode()).decode("ascii")
    return ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
            "-o", "NumberOfPasswordPrompts=0", "-o", "StrictHostKeyChecking=yes", target or user + "@" + host,
            "echo " + payload + " | base64 -d | python3"]


def classify_failure(output):
    lowered = output.lower()
    if "could not resolve hostname" in lowered or "name or service not known" in lowered:
        return "name resolution", "Check that the laptop hotspot is active and duck2 has finished booting."
    for kind, markers, hint in FAILURE_HINTS:
        if any(marker in lowered for marker in markers):
            return kind, hint
    return "remote check", "Read the SSH output below; do not retry automatically."


def resolve(host):
    try:
        addresses = socket.getaddrinfo(host, 22, type=socket.SOCK_STREAM)
    except socket.gaierror as error:
        return [], str(error)
    return sorted({entry[4][0] for entry in addresses}), None


def cache_directory():
    base = Path(os.environ.get("LOCALAPPDATA", Path.home() / ".local"))
    return base / "Duck2" / "connection-checks"


def runtime_fingerprint(output):
    containers = [" ".join(line.split()[:2]) for line in output.splitlines()
                  if line.startswith(("ros ", "duckiebot-interface ", "car-interface "))]
    return hashlib.sha256("\n".join(sorted(containers)).encode()).hexdigest()


def readiness_issues(output):
    """SSH success is not proof that normal controller ownership is restored."""
    lines = output.splitlines()
    issues = []
    for name in ("ros", "duckiebot-interface", "car-interface"):
        if not any(line.startswith(name + " ") for line in lines):
            issues.append(name + " is not running")
    for name in ("camera_node", "wheels_driver_node", "kinematics_node"):
        if "/duck2/" + name not in lines:
            issues.append(name + " is not registered")
    wheel_info = output.split("CHECK: ROS master and normal wheel publisher", 1)[-1]
    wheel_info = wheel_info.split("CHECK:", 1)[0]
    publishers = re.search(r"Publishers:(.*?)(?:Subscribers:|$)", wheel_info, re.S)
    names = re.findall(r"\*\s+([^\s]+)", publishers.group(1)) if publishers else []
    if names != ["/duck2/kinematics_node"]:
        issues.append("normal wheel ownership is not established (" + ", ".join(names or ["none"]) + ")")
    return issues


def previous_fingerprint(directory):
    summaries = sorted(directory.glob("*.json"))
    if not summaries:
        return None
    try:
        return json.loads(summaries[-1].read_text()).get("runtime_fingerprint")
    except (OSError, json.JSONDecodeError):
        return None


def save_summary(host, addresses, mode, output):
    directory = cache_directory()
    directory.mkdir(parents=True, exist_ok=True)
    fingerprint = runtime_fingerprint(output)
    prior = previous_fingerprint(directory)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    path = directory / (stamp + ".json")
    path.write_text(json.dumps({
        "checked_at_utc": stamp,
        "host": host,
        "addresses": addresses,
        "mode": mode,
        "runtime_fingerprint": fingerprint,
        "runtime_identity_changed": prior is not None and prior != fingerprint,
        "output": output,
    }, indent=2) + "\n")
    return path, prior is not None and prior != fingerprint


def run_check(host, user, mode, target, save):
    addresses, resolution_error = resolve(host)
    if resolution_error:
        print("duck2 connection check: name resolution failed")
        print("Action: Check that the laptop hotspot is active and duck2 has finished booting.")
        return 2
    print("Resolved {}: {}".format(host, ", ".join(addresses)))
    result = subprocess.run(
        ssh_arguments(host, user, mode, target),
        universal_newlines=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=30,
    )
    if result.returncode:
        kind, hint = classify_failure(result.stdout)
        print("duck2 connection check: {} failed".format(kind))
        print("Action: " + hint)
        print(result.stdout.strip())
        return result.returncode or 1
    issues = readiness_issues(result.stdout)
    print("duck2 connection check: {} ({})".format(
        "SSH/ROS reachable; normal control NOT ready" if issues else "passed", mode))
    print(result.stdout.strip())
    if save:
        path, changed = save_summary(host, addresses, mode, result.stdout)
        print("Saved local summary: " + str(path))
        if changed:
            print("Runtime/container inventory changed; " + (
                "full metadata recorded in this check." if mode == "full"
                else "run --full before live work."))
    for issue in issues:
        print("NOT READY: " + issue)
    if issues:
        print("No services were restarted. Resolve controller ownership before driving.")
    return 3 if issues else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connect", action="store_true", help="Run a read-only SSH check")
    parser.add_argument("--full", action="store_true", help="Include full runtime compatibility metadata")
    parser.add_argument("--host", help="Robot hostname or current IPv4 address")
    parser.add_argument("--save-summary", action="store_true", help="Save non-secret output outside the repository")
    args = parser.parse_args()
    settings = config()
    if not args.connect:
        print(json.dumps(settings, indent=2))
        return 0
    host = args.host or settings["hostname"]
    target = settings.get("ssh_alias") if not args.host else None
    return run_check(host, settings["ssh_user"], "full" if args.full else "quick", target,
                     args.save_summary)


if __name__ == "__main__":
    sys.exit(main())
