"""Detector parity and no-publisher checks for the read-only camera service."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

from test_lane_follower import LaneTests, load_node


def load_gateway(node_module):
    source = Path(__file__).resolve().parents[1] / "packages/duckie_lane_follower/src"
    stubs = {name: ModuleType(name) for name in
             ("rospy", "cv_bridge", "sensor_msgs", "sensor_msgs.msg",
              "duckietown", "duckietown.dtros")}
    stubs["rospy"].get_param = lambda name, default: default
    stubs["cv_bridge"].CvBridge = lambda: NS()
    stubs["cv_bridge"].CvBridgeError = type("BridgeError", (Exception,), {})
    stubs["sensor_msgs.msg"].CompressedImage = object
    stubs["duckietown.dtros"].DTROS = object
    stubs["duckietown.dtros"].NodeType = NS(PERCEPTION="PERCEPTION")
    path = source / "camera_gateway.py"
    spec = importlib.util.spec_from_file_location("camera_gateway_under_test", path)
    module = importlib.util.module_from_spec(spec)
    modules = dict(stubs, lane_follower_node=node_module)
    with patch.dict(sys.modules, modules), patch.object(sys, "path", [str(source)] + sys.path):
        spec.loader.exec_module(module)
    return module


class CameraGatewayTests(unittest.TestCase):
    def setUp(self):
        case = LaneTests(methodName="test_topics_and_shutdown_registration")
        case.setUp()
        self.case = case
        self.node = case.node
        self.module = load_gateway(case.mod)

    def test_preview_uses_exact_controller_detector(self):
        preview = self.module.LanePreview()
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        image[250:450, 150:175] = (0, 255, 255)
        image[250:450, 470:495] = (255, 255, 255)
        expected = self.node.detect_lane_bgr(image.copy())
        actual = preview.detect_lane_bgr(image.copy())
        self.assertAlmostEqual(expected[0], actual[0])
        np.testing.assert_array_equal(expected[1], actual[1])
        np.testing.assert_array_equal(expected[2], actual[2])

    def test_preview_skips_excess_decode_without_refreshing_frame_age(self):
        from unittest.mock import Mock
        clock = [100.0]
        self.module.time = NS(monotonic=lambda: clock[0])
        self.module.rospy.Subscriber = lambda *args, **kwargs: NS()
        feed = self.module.CameraFeed("duck2")
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        feed.bridge = NS(compressed_imgmsg_to_cv2=Mock(return_value=image))
        message = NS(header=NS(stamp=NS(to_sec=lambda: 100.0)))
        feed.receive(message)
        self.assertEqual(feed.bridge.compressed_imgmsg_to_cv2.call_count, 1)
        for value in [100.02, 100.04, 100.08]:
            clock[0] = value
            feed.receive(message)
        self.assertEqual(feed.bridge.compressed_imgmsg_to_cv2.call_count, 1)
        self.assertEqual(feed.received_at, 100.0)
        self.assertAlmostEqual(feed.get("normal")[1], .08)
        clock[0] = 100.11
        feed.receive(message)
        self.assertEqual(feed.bridge.compressed_imgmsg_to_cv2.call_count, 2)
        self.assertEqual(feed.received_at, 100.11)
        for view in ("normal", "mask", "overlay"):
            self.assertTrue(feed.get(view)[0].startswith(b"P6\n"))
        clock[0] = 100.8
        self.assertGreater(feed.get("normal")[1], .5)

    def test_preview_fps_setting_is_bounded(self):
        self.module.rospy.Subscriber = lambda *args, **kwargs: NS()
        for invalid in [0, 31, float("nan")]:
            self.module.rospy.get_param = lambda name, default: invalid if name == "~preview_max_fps" else default
            with self.assertRaisesRegex(ValueError, "preview_max_fps"):
                self.module.CameraFeed("duck2")

    def test_ppm_encoding_and_service_source_have_no_publisher(self):
        image = np.zeros((20, 30, 3), dtype=np.uint8)
        data = self.module.ppm_bytes(image)
        self.assertTrue(data.startswith(b"P6\n30 20\n255\n"))
        source = Path(self.module.__file__).read_text(encoding="utf8")
        self.assertNotIn("rospy.Publisher", source)
        self.assertNotIn("wheels_cmd", source)


if __name__ == "__main__":
    unittest.main()
