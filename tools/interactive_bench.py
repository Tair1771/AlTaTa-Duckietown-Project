"""Serve the companion app from an isolated onboard simulation for up to 20 minutes."""
import base64
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import queue
import shlex
import subprocess
import tarfile
import tempfile
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from isolated_bench_check import ROOT, Target, SSH, create_args, verify_isolation, make_bundle

BOOT = '''set -eo pipefail
tar --no-same-owner -xzf /tmp/bench.tar.gz -C /
source /environment.sh
export ROS_MASTER_URI=http://localhost:11311 ROS_HOSTNAME=localhost VEHICLE_NAME=duck2
export PYTHONPATH=/project/packages/duckie_lane_follower/src:/project/laptop:${PYTHONPATH:-}
exec python3 /project/tools/bench_scene.py
'''


class Relay:
    def __init__(self, process):
        self.process = process
        self.lock = threading.Lock()
        self.pending = {}
        threading.Thread(target=self.read, daemon=True).start()

    def read(self):
        for line in self.process.stdout:
            try:
                value = json.loads(line)
                with self.lock:
                    waiter = self.pending.get(value.get("id"))
                if waiter:
                    waiter.put(value)
            except (ValueError, OSError):
                pass

    def request(self, method, path, body):
        key = uuid.uuid4().hex
        waiter = queue.Queue()
        with self.lock:
            self.pending[key] = waiter
            self.process.stdin.write(json.dumps({"id": key, "method": method, "path": path,
                "body": base64.b64encode(body).decode()}) + "\n")
            self.process.stdin.flush()
        try:
            return waiter.get(timeout=5)
        finally:
            with self.lock:
                self.pending.pop(key, None)


def main():
    target = Target(True)
    tag = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:8]
    name = "duck2-interactive-bench-" + tag.lower()
    output = Path(os.environ["LOCALAPPDATA"]) / "Duck2/bench-checks" / tag
    output.mkdir(parents=True)
    remote = "/tmp/" + name
    image = json.loads((ROOT / "config/duck2.json").read_text())["base_images"]["arm64v8"]
    servers = []
    created = False
    bridge = None
    try:
        target.docker(["image", "inspect", image])
        target.run(["mkdir", "-m", "700", remote])
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "bench.tar.gz"
            manifest = make_bundle(path)
            (output / "sources.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
            subprocess.run(["scp", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
                            str(path), "duck2:" + remote + "/bench.tar.gz"], check=True, timeout=60)
        args = create_args(name, image)
        args[args.index("480")] = "1200"
        args[-1] = BOOT
        target.docker(args)
        created = True
        info = json.loads(target.docker(["inspect", name]).stdout)[0]
        verify_isolation(info)
        (output / "isolation.json").write_text(json.dumps(info, indent=2), encoding="utf-8")
        target.docker(["cp", remote + "/bench.tar.gz", name + ":/tmp/bench.tar.gz"])
        target.docker(["start", name])
        # Wait for extraction before invoking the stdio relay.
        for _ in range(30):
            if target.docker(["exec", name, "test", "-f", "/project/tools/bench_rpc.py"], check=False).returncode == 0:
                break
            time.sleep(1)
        bridge = subprocess.Popen(SSH + [shlex.join(["docker", "exec", "-i", name,
             "python3", "-u", "/project/tools/bench_rpc.py"])], stdin=subprocess.PIPE,
             stdout=subprocess.PIPE, stderr=open(output / "relay.log", "w"), text=True,
             encoding="utf-8", creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        relay = Relay(bridge)
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass
            def handle_request(self):
                try:
                    length = int(self.headers.get("Content-Length", 0))
                    if not 0 <= length <= 8192:
                        raise ValueError("Request too large")
                    value = relay.request(self.command, self.path, self.rfile.read(length))
                    data = base64.b64decode(value["body"])
                    self.send_response(value["code"])
                    for key, val in value["headers"].items():
                        if key.lower() in ("content-type", "x-camera-age", "x-captured-at", "x-diagnostic"):
                            self.send_header(key, val)
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                except Exception:
                    self.send_error(503, "Bench simulation unavailable")
            do_GET = handle_request
            do_POST = handle_request
        for port in (18765, 18766):
            server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
            server.daemon_threads = True
            servers.append(server)
            threading.Thread(target=server.serve_forever, daemon=True).start()
        (output / "session.json").write_text(json.dumps({"container": name, "ports": [18765,18766],
                 "physical_motion": False, "deadline_seconds": 1200}), encoding="utf-8")
        print("BENCH READY: isolated simulation on 18765/18766; " + str(output), flush=True)
        deadline = time.monotonic() + 1180
        while time.monotonic() < deadline and bridge.poll() is None:
            time.sleep(1)
    finally:
        for server in servers:
            server.shutdown()
            server.server_close()
        if bridge is not None:
            bridge.terminate()
        if created:
            info = json.loads(target.docker(["inspect", name]).stdout)[0]
            verify_isolation(info)
            if info["State"]["Running"]:
                controller_log = target.docker(["exec", name, "cat", "/tmp/bench-services.log"], check=False)
                (output / "controller.log").write_text(controller_log.stdout + controller_log.stderr,
                                                       encoding="utf-8")
            logs = target.docker(["logs", name], check=False)
            (output / "services.log").write_text(logs.stdout + logs.stderr, encoding="utf-8")
            target.docker(["rm", "-f", name])
        target.run(["rm", "-f", remote + "/bench.tar.gz"])
        target.run(["rmdir", remote])


if __name__ == "__main__":
    main()
