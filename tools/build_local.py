"""Build our application locally. No robot deployment or ROS execution.

Run from Windows or WSL with Docker Desktop running. No dts profile or API key
is needed. Existing base images can be used without downloading dependencies.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def local_docker(command, env):
    # A build command must never be accidentally routed to the robot's daemon.
    if env.get("DOCKER_HOST") and not env["DOCKER_HOST"].startswith(("unix://", "npipe://")):
        raise ValueError("Remote DOCKER_HOST is not allowed for this local build")
    result = subprocess.run(command + ["context", "inspect", "--format",
                                      "{{.Name}} {{.Endpoints.docker.Host}}"],
                            check=True, capture_output=True, text=True, env=env)
    context, endpoint = result.stdout.strip().split(" ", 1)
    if not endpoint.startswith(("unix://", "npipe://")):
        raise ValueError("Select a local Docker Desktop context before building")
    return context


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arch", choices=("amd64", "arm64v8"), default="amd64")
    parser.add_argument("--pull", action="store_true", help="Download the pinned base first")
    args = parser.parse_args()
    config = json.loads((ROOT / "config/duck2.json").read_text())
    base = config["base_images"][args.arch]
    tag = "altata-duck2:noetic-" + args.arch
    env = os.environ.copy()
    command = ["docker"]
    context = local_docker(command, env)
    command += ["--context", context]
    env.pop("DOCKER_HOST", None)
    env.pop("DOCKER_CONTEXT", None)
    env["BUILDX_BUILDER"] = context
    if args.pull:
        subprocess.run(command + ["pull", base], check=True, env=env)
    subprocess.run(command + ["build", "--builder", context, "--network", "none",
                             "--platform", "linux/amd64" if args.arch == "amd64" else "linux/arm64",
                             "--build-arg", "BASE_IMAGE=" + base,
                             "--tag", tag, str(ROOT)], check=True, env=env)
    print("Built " + tag + ". Nothing was deployed or started on the robot.")


if __name__ == "__main__":
    main()
