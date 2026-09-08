"""Read-only SSH metadata inspection. Never subscribes or publishes ROS data.

Without --connect, print the observed configuration and make no connections.
With --connect, SSH asks for credentials using its normal prompt/key handling.
"""
import argparse
import base64
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]

# Fixed command list: do not accept arbitrary robot/container commands.
REMOTE = """
import subprocess
checks = [
    ["hostname"],
    ["uname", "-m"],
    ["cat", "/etc/os-release"],
    ["docker", "ps", "--format", "{{.Names}} {{.Image}} {{.Status}}"],
    ["docker", "exec", "ros", "bash", "-lc",
     "source /environment.sh >/dev/null 2>&1; rosversion -d; "
     "rosmsg md5 sensor_msgs/CompressedImage; "
     "rosmsg md5 duckietown_msgs/WheelsCmdStamped; "
     "rostopic info /duck2/camera_node/image/compressed; "
     "rostopic info /duck2/wheels_driver_node/wheels_cmd"]
]
for args in checks:
    print("CHECK:", " ".join(args), flush=True)
    result = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            universal_newlines=True, timeout=30)
    print(result.stdout, flush=True)
    if result.returncode:
        raise SystemExit(result.returncode)
"""


def ssh_arguments(host, user):
    if (not re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9.-]*", host)
            or not re.fullmatch(r"[a-zA-Z_][a-zA-Z0-9_-]*", user)):
        raise ValueError("Use a hostname or IPv4 address and a simple SSH username")
    payload = base64.b64encode(REMOTE.encode()).decode("ascii")
    return ["ssh", "-o", "ConnectTimeout=5", "-o", "NumberOfPasswordPrompts=1",
            user + "@" + host, "echo " + payload + " | base64 -d | python3"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connect", action="store_true")
    parser.add_argument("--host", help="Current address; hotspot addresses can change")
    args = parser.parse_args()
    config = json.loads((ROOT / "config/duck2.json").read_text())
    if not args.connect:
        print(json.dumps(config, indent=2))
        return
    command = ssh_arguments(args.host or config["hostname"], config["ssh_user"])
    subprocess.run(command, check=True)


if __name__ == "__main__":
    main()
