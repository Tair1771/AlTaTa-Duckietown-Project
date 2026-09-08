import json
from pathlib import Path
import sys
import tempfile
import unittest
import cv2
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from replay import replay, ReplayRuntime, AutonomyCore, NS


class ReplayTests(unittest.TestCase):
    def test_observation_is_immutable_and_does_not_commit_perception(self):
        import os
        from unittest.mock import patch
        clock = NS(value=100., monotonic=lambda:100., time=lambda:100.)
        runtime = ReplayRuntime(clock, {})
        with patch.dict(os.environ, VEHICLE_NAME="duck2"):
            core = AutonomyCore("test", runtime, clock)
        image = np.zeros((480,640,3), np.uint8)
        cv2.rectangle(image, (160,240), (180,455), (0,255,255), -1)
        cv2.rectangle(image, (470,240), (490,455), (255,255,255), -1)
        observation, _, _ = core.observe_bgr(image, 99.9, 100.)
        self.assertEqual(observation.source_stamp, 99.9)
        self.assertIsInstance(observation.lane_limits, tuple)
        self.assertIsNone(core._lane_limits)
        with self.assertRaises(AttributeError):
            observation.lane_error = 0

    def test_recorder_format_deterministic_outputs_and_report(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            image = np.zeros((480, 640, 3), np.uint8)
            cv2.rectangle(image, (160,240), (180,455), (0,255,255), -1)
            cv2.rectangle(image, (470,240), (490,455), (255,255,255), -1)
            cv2.imwrite(str(root/"frame.png"), image)
            manifest = root/"frames.jsonl"
            manifest.write_text("\n".join(json.dumps(dict(camera_stamp=100+i*.05,
                file="frame.png", label_error=.015625)) for i in range(8)))
            first = replay(manifest, root/"one")
            second = replay(manifest, root/"two")
            self.assertEqual(first["lane_detected"], 8)
            self.assertLess(first["lane_error_mae"], .001)
            self.assertTrue((root/"one/lane-error.svg").exists())
            self.assertEqual(first["lane_error_mae"], second["lane_error_mae"])
            import csv
            def values(path):
                with path.open() as stream:
                    return [(r["left"],r["right"],r["error"]) for r in csv.DictReader(stream)]
            self.assertEqual(values(root/"one/frames.csv"), values(root/"two/frames.csv"))
            with self.assertRaises(FileExistsError):
                replay(manifest, root/"one")

    def test_duplicate_or_missing_frame_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = root/"bad.jsonl"
            manifest.write_text(json.dumps(dict(t=0,image="missing.png")))
            with self.assertRaisesRegex(ValueError, "Unreadable"):
                replay(manifest, root/"out")
