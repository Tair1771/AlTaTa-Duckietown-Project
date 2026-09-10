"""Read-only laptop check of the stationary preview's three camera views."""
from datetime import datetime, timezone
from io import BytesIO
import json
import os
from pathlib import Path
import sys
import time
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "laptop"))
from camera_client import CameraClient


def main():
    from PIL import Image
    with urllib.request.urlopen("http://127.0.0.1:8765/status", timeout=3) as response:
        status = json.load(response)
    if status.get("drive_enabled") is not False or status.get("wheel_speeds") != [0.0, 0.0]:
        raise RuntimeError("Expected driving-disabled stationary preview; no further check performed")
    client = CameraClient()
    samples = []
    for _ in range(3):
        for view in client.VIEWS:
            frame = client.frame(view)
            with Image.open(BytesIO(frame.ppm)) as image:
                image.load()
                size = image.size
            if min(size) < 1 or frame.age > .5:
                raise RuntimeError("Camera decoding/freshness check failed: %s %.3fs" % (view, frame.age))
            samples.append({"view": view, "age_seconds": frame.age,
                            "captured_at": frame.captured_at, "size": size})
            time.sleep(.2)
    for view in client.VIEWS:
        stamps = [item["captured_at"] for item in samples if item["view"] == view]
        if not all(b > a for a, b in zip(stamps, stamps[1:])):
            raise RuntimeError("Camera timestamps did not advance: " + view)
    directory = Path(os.environ.get("LOCALAPPDATA", Path.home())) / "Duck2" / "bench-checks"
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-camera.json")
    path.write_text(json.dumps(samples, indent=2), encoding="utf-8")
    print("PASS: normal/mask/overlay decode, timestamps advance, all nine sampled frame ages <=0.5s.")
    print("Driving disabled; no camera images saved. Metadata: " + str(path))


if __name__ == "__main__":
    main()
