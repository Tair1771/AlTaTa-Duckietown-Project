"""Offline regression tests: actual OpenCV and controller, mocked ROS transport."""
import importlib.util
import json
import sys
import threading
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import patch

import cv2
import numpy as np

class WheelMessage:
    def __init__(self):
        self.header = NS(stamp=None)

class BridgeError(Exception):
    pass

class Base:
    def __init__(self, **kwargs):
        pass

def load_node():
    stubs = {}
    for name in ("rospy", "cv_bridge", "duckietown", "duckietown.dtros",
                 "duckietown_msgs", "duckietown_msgs.msg", "sensor_msgs", "sensor_msgs.msg",
                 "std_msgs", "std_msgs.msg"):
        stubs[name] = ModuleType(name)
    stubs["duckietown.dtros"].DTROS = Base
    stubs["duckietown.dtros"].NodeType = NS(PERCEPTION=1)
    stubs["cv_bridge"].CvBridge = lambda: NS(compressed_imgmsg_to_cv2=lambda msg, **kw: msg.image)
    stubs["cv_bridge"].CvBridgeError = BridgeError
    stubs["duckietown_msgs.msg"].WheelsCmdStamped = WheelMessage
    stubs["sensor_msgs.msg"].CompressedImage = object
    stubs["std_msgs.msg"].String = lambda **kw: NS(**kw)
    path = Path(__file__).resolve().parents[1] / "packages/duckie_lane_follower/src/lane_follower_node.py"
    spec = importlib.util.spec_from_file_location("lane_under_test", path)
    mod = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, stubs):
        spec.loader.exec_module(mod)
    return mod

class LaneTests(unittest.TestCase):
    def setUp(self):
        self.mod = load_node()
        self.now = 100.0
        self.mod.time = NS(monotonic=lambda: self.now, time=lambda: self.now)
        self.messages = []
        self.hooks = []
        self.params = {}
        self.mod.rospy = NS(
            get_param=lambda name, default: self.params.get(name, default),
            Publisher=lambda topic, *a, **kw: NS(publish=(self.messages.append
                if topic.endswith("wheels_cmd") else lambda msg: None)),
            Subscriber=self.subscribe,
            on_shutdown=self.hooks.append, Timer=lambda *a: NS(),
            Duration=lambda seconds: seconds, Time=NS(now=lambda: NS(to_sec=lambda: self.now)),
            loginfo=lambda *a: None, logwarn=lambda *a: None)
        with patch.dict(self.mod.os.environ, VEHICLE_NAME="duck2"):
            self.node = self.mod.LaneFollowerNode("test")

    def subscribe(self, topic, message_type, callback, **kwargs):
        # ROS is allowed to deliver an image immediately during construction.
        if topic.endswith("image/compressed"):
            callback(self.message(np.zeros((480, 640, 3), dtype=np.uint8)))
        return NS()

    def message(self, image, stamp=None):
        value = self.now if stamp is None else stamp
        return NS(image=image, header=NS(stamp=NS(to_sec=lambda: value)))

    def deliver(self, image):
        self.node.callback(self.message(image))

    def speeds(self):
        msg = self.messages[-1]
        return msg.vel_left, msg.vel_right

    def image(self, red=None):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(img, (160, 240), (180, 455), (0, 255, 255), -1)
        cv2.rectangle(img, (470, 240), (490, 455), (255, 255, 255), -1)
        if red:
            cv2.rectangle(img, red[0], red[1], (0, 0, 255), -1)
        return img

    def test_topics_and_shutdown_registration(self):
        self.assertEqual(self.node.camera_topic, "/duck2/camera_node/image/compressed")
        self.assertEqual(self.node.wheels_topic, "/duck2/wheels_driver_node/wheels_cmd")
        self.assertEqual(self.hooks, [self.node.on_shutdown])

    def test_diagnostic_topic_override_is_absolute_and_validated(self):
        self.params["~wheels_topic"] = "/duck2/lane_follower/diagnostic_wheels_cmd"
        with patch.dict(self.mod.os.environ, VEHICLE_NAME="duck2"):
            diagnostic_node = self.mod.LaneFollowerNode("diagnostic")
        self.assertEqual(diagnostic_node.wheels_topic,
                         "/duck2/lane_follower/diagnostic_wheels_cmd")
        self.params["~wheels_topic"] = "diagnostic_wheels_cmd"
        with patch.dict(self.mod.os.environ, VEHICLE_NAME="duck2"):
            with self.assertRaisesRegex(ValueError, "absolute ROS topic"):
                self.mod.LaneFollowerNode("invalid-topic")

    def test_camera_debug_never_drives(self):
        for _ in range(10):
            self.now += .1
            self.deliver(self.image())
            self.assertEqual(self.speeds(), (0, 0))

    def test_lane_center_and_missing_lines(self):
        error, _, _ = self.node.detect_lane_bgr(self.image())
        self.assertLess(abs(error), .04)
        error, _, _ = self.node.detect_lane_bgr(np.zeros((480, 640, 3), np.uint8))
        self.assertIsNone(error)

    def test_red_line_geometry_and_both_hue_ranges(self):
        self.assertTrue(self.node.detect_red_stop(self.image(((260, 370), (580, 390)))))
        hsv = cv2.cvtColor(self.image(((260, 370), (580, 390))), cv2.COLOR_BGR2HSV)
        hsv[370:391, 260:581, 0] = 179
        self.assertTrue(self.node.detect_red_stop(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)))
        for box in [((260, 170), (580, 190)), ((270, 350), (290, 430)),
                    ((260, 350), (285, 375)), ((20, 370), (200, 390))]:
            self.assertFalse(self.node.detect_red_stop(self.image(box)), str(box))

    def test_stop_latches_even_after_line_leaves_image(self):
        self.node.drive_enabled = True
        self.now += .1
        self.deliver(self.image())
        self.assertGreater(max(self.speeds()), 0)
        self.now += .1
        self.deliver(self.image(((260, 370), (580, 390))))
        self.assertEqual(self.speeds(), (0, 0))
        self.assertTrue(self.node.red_stop_latched)
        for _ in range(20):
            self.now += .1
            self.deliver(self.image())
            self.assertEqual(self.speeds(), (0, 0))

    def test_acceleration_limit_and_immediate_stop(self):
        self.node.drive_enabled = True
        self.now += .1
        self.node.publish_wheels(.1, .1)
        self.assertAlmostEqual(self.speeds()[0], .015)
        self.node.publish_wheels(0, 0)
        self.assertEqual(self.speeds(), (0, 0))

    def test_camera_timeout_and_resume_ramp(self):
        self.node.drive_enabled = True
        self.now += .1
        self.deliver(self.image())
        self.now += .6
        self.node.check_camera_timeout(None)
        self.assertEqual(self.speeds(), (0, 0))
        self.node.publish_wheels(.1, .1)
        self.assertEqual(self.speeds(), (0, 0))
        self.now += .1
        self.deliver(self.image())
        self.assertLessEqual(max(self.speeds()), .0150001)

    def test_shutdown_blocks_late_publish(self):
        self.node.drive_enabled = True
        self.hooks[0]()
        self.now += .1
        worker = threading.Thread(target=lambda: self.node.publish_wheels(.1, .1))
        worker.start()
        worker.join()
        self.assertEqual(self.speeds(), (0, 0))

    def test_empty_and_malformed_images_stop(self):
        self.node.drive_enabled = True
        for bad in [None, np.zeros((0, 0, 3), np.uint8), np.zeros((480, 640), np.uint8)]:
            self.now += .1
            self.deliver(self.image())
            self.now += .001
            self.deliver(bad)
            self.assertEqual(self.speeds(), (0, 0))

    def test_mode_caps_and_lane_loss(self):
        self.node.drive_enabled = True
        for base, cap, steer in [(.05, .07, .02), (.08, .18, .08)]:
            self.node.base_speed, self.node.max_speed, self.node.max_steering = base, cap, steer
            for error in [-10., 0., 10., None]:
                for _ in range(100):
                    self.now += 1/30
                    left, right, _ = self.node.compute_wheel_speeds(error)
                    self.assertTrue(0 <= left <= cap and 0 <= right <= cap)
                if error is None:
                    self.assertEqual((left, right), (0, 0))

    def test_filter_is_consistent_across_frame_rates(self):
        results = []
        for hz in [10, 30, 60]:
            self.node.filtered_error = self.node.prev_steering = 0.
            self.node._last_control_time = self.now
            for _ in range(hz):
                self.now += 1/hz
                self.node.compute_wheel_speeds(.5)
            results.append(self.node.filtered_error)
        self.assertAlmostEqual(results[0], results[1], places=6)
        self.assertAlmostEqual(results[1], results[2], places=6)

if __name__ == "__main__":
    unittest.main()
