#!/usr/bin/env python3
"""Deterministic JPEG/PNG replay: shared core, synthetic clock, no ROS/network.

Manifest is JSONL: {"t": 0.1, "image": "frame.jpg", "label_error": 0.0}.
Optional command object is delivered before that frame. Missing images fail
the run, never silently skip. Commands and wheel values are simulated only.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import statistics
import sys
import time
from types import SimpleNamespace as NS
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "packages/duckie_lane_follower/src"))
from duck2_core import AutonomyCore
import cv2


class ReplayRuntime:
    class WheelsCmdStamped:
        def __init__(self):
            self.header = NS(stamp=None)

    CompressedImage = object
    String = NS
    CvBridgeError = ValueError
    Duration = staticmethod(float)
    loginfo = logwarn = staticmethod(lambda *a: None)
    on_shutdown = staticmethod(lambda *a: None)
    Subscriber = Timer = staticmethod(lambda *a, **kw: NS())
    CvBridge = staticmethod(lambda: NS(compressed_imgmsg_to_cv2=lambda msg, **kw: msg.image))

    def __init__(self, clock, params):
        self.clock, self.params = clock, params
        self.Time = NS(now=lambda: NS(to_sec=lambda: clock.value))
        self.messages = []

    def get_param(self, name, default):
        return self.params.get(name.lstrip("~"), default)

    def Publisher(self, topic, kind, **kwargs):
        return NS(publish=self.messages.append)


def replay(manifest, output, params=None):
    output.mkdir(parents=True, exist_ok=False)
    clock = NS(value=1000.)
    clock.monotonic = clock.time = lambda: clock.value
    runtime = ReplayRuntime(clock, dict({"drive_enabled": True}, **(params or {})))
    os.environ.setdefault("VEHICLE_NAME", "duck2")
    core = AutonomyCore("replay", runtime, clock)
    previous = -1.
    origin = None
    rows, errors, durations = [], [], []
    for line in manifest.read_text().splitlines():
        if not line.strip():
            continue
        entry = json.loads(line)
        if "t" in entry:
            t = entry["t"]
        else:
            stamp = entry["camera_stamp"]
            if type(stamp) not in (float, int) or not math.isfinite(stamp):
                raise ValueError("Invalid recorded camera timestamp")
            origin = stamp if origin is None else origin
            t = stamp - origin
        filename = entry.get("image", entry.get("file"))
        if not isinstance(filename, str):
            raise ValueError("Manifest requires image or recorder file field")
        if type(t) not in (float, int) or not math.isfinite(t) or t < 0 or t <= previous:
            raise ValueError("Manifest timestamps must be finite, nonnegative and increasing")
        previous = t
        clock.value = 1000. + t
        if "command" in entry:
            command = dict(entry["command"], issued_at=clock.value)
            core.command_callback(NS(data=json.dumps(command)))
        image = cv2.imread(str(manifest.parent / filename))
        if image is None:
            raise ValueError("Unreadable frame: " + filename)
        core.check_camera_timeout(None)
        started = time.perf_counter()
        core.callback(NS(image=image, header=NS(stamp=NS(to_sec=lambda: clock.value))))
        elapsed = (time.perf_counter()-started)*1000
        state = core.status()
        duration = dict(t=t, image=filename, error=state["lane_error"],
                        left=state["wheel_speeds"][0], right=state["wheel_speeds"][1],
                        obstacle=state["obstacle_visible"], stop_reason=state["stop_reason"],
                        processing_ms=elapsed)
        rows.append(duration)
        durations.append(elapsed)
        if "label_error" in entry and state["lane_error"] is not None:
            label = entry["label_error"]
            if type(label) not in (float, int) or not math.isfinite(label):
                raise ValueError("label_error must be finite")
            errors.append(abs(label-state["lane_error"]))
    if not rows:
        raise ValueError("Empty replay manifest")
    with (output / "frames.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    summary = dict(frames=len(rows), lane_detected=sum(r["error"] is not None for r in rows),
                   labelled_detected=len(errors), lane_error_mae=statistics.mean(errors) if errors else None,
                   processing_ms_p50=statistics.median(durations), processing_ms_max=max(durations),
                   simulated_only=True, parameters=params or {})
    (output / "summary.json").write_text(json.dumps(summary, indent=2)+"\n")
    # Minimal SVG trace, no plotting dependency. Y range is normalized lane error.
    points = " ".join("%.1f,%.1f" % (30+i*740/max(1,len(rows)-1), 110-80*r["error"])
                      for i,r in enumerate(rows) if r["error"] is not None)
    (output / "lane-error.svg").write_text('<svg xmlns="http://www.w3.org/2000/svg" width="800" height="240">'
        '<rect width="800" height="240" fill="white"/><text x="30" y="20">'
        'Simulated replay: normalized lane error vs frame index (missing detections omitted)</text>'
        '<path d="M30 110H770" stroke="gray"/><polyline points="'+points+'" fill="none" stroke="blue"/></svg>')
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--params", type=Path)
    args = parser.parse_args()
    print(json.dumps(replay(args.manifest, args.output,
                           json.loads(args.params.read_text()) if args.params else None), indent=2))
