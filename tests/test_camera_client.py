"""Read-only camera client tests with HTTP access fully mocked."""

from email.message import Message
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "laptop"))

from camera_client import CameraClient


class Response:
    def __init__(self, body=b"P6\n1 1\n255\n\x00\x00\x00", content_type="image/x-portable-pixmap"):
        self.body = body
        self.headers = Message()
        self.headers["Content-Type"] = content_type
        self.headers["X-Camera-Age"] = "0.125"
        self.headers["X-Captured-At"] = "123.5"
        self.headers["X-Diagnostic"] = "Both boundaries"

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def read(self, size):
        return self.body


class CameraClientTests(unittest.TestCase):
    def test_requires_loopback_ssh_tunnel(self):
        for url in ("http://duck2.local:8766", "https://127.0.0.1:8766", "bad"):
            with self.assertRaises(ValueError):
                CameraClient(url)

    def test_reads_bounded_ppm_and_metadata(self):
        with patch("urllib.request.urlopen", return_value=Response()) as opened:
            frame = CameraClient().frame("overlay")
        self.assertEqual(frame.view, "overlay")
        self.assertEqual(frame.age, .125)
        self.assertEqual(frame.diagnostic, "Both boundaries")
        self.assertIn("view=overlay", opened.call_args.args[0].full_url)

    def test_rejects_wrong_view_and_malformed_response(self):
        with self.assertRaises(ValueError):
            CameraClient().frame("wheels")
        with patch("urllib.request.urlopen", return_value=Response(b"not ppm")):
            with self.assertRaises(RuntimeError):
                CameraClient().frame()


if __name__ == "__main__":
    unittest.main()
