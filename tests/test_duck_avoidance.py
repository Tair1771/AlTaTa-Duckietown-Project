"""Synthetic passing tests; image feedback is not physical trajectory evidence."""
import json
import unittest
import cv2
import numpy as np
from types import SimpleNamespace as NS
from unittest.mock import patch
from test_lane_follower import LaneTests


class DuckTests(LaneTests):
    def scene(self, left_lane=False, duck=True, near=False):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        edges = (230, 410, 590) if left_lane else (50, 230, 410)
        for x, color in zip(edges, ((255,255,255), (0,255,255), (255,255,255))):
            cv2.rectangle(image, (x-6,240), (x+6,475), color, -1)
        if duck:
            x = 460 if left_lane else 280
            y = 409 if near else 345
            cv2.rectangle(image, (x,y), (x+48,y+48), (0,255,255), -1)
        return image

    def frame(self, image):
        self.now += .1
        self.deliver(image)
        self.assertTrue(all(0 <= v <= self.node.max_speed for v in self.speeds()))

    def enable(self):
        self.node.avoidance_enabled = self.node.avoidance_calibrated = True
        self.node.drive_enabled = True

    def start_pass(self):
        self.enable()
        self.frame(self.scene())
        self.assertEqual(self.node.avoidance_state, "shift_left")
        for _ in range(5):
            self.frame(self.scene(left_lane=True))
        self.assertEqual(self.node.avoidance_state, "passing")

    def test_default_stops_and_does_not_enable_passing(self):
        self.node.drive_enabled = True
        self.frame(self.scene())
        self.assertEqual(self.node.avoidance_state, "idle")
        self.assertTrue(self.node.obstacle_stop_latched)
        self.assertEqual(self.speeds(), (0,0))

    def test_detection_excludes_duck_from_lane_and_keeps_defaults(self):
        image=self.scene()
        self.frame(image)
        self.assertEqual(len(self.node._duck_boxes), 1)
        self.assertAlmostEqual(self.node._last_lane_error, 0, delta=.03)
        self.assertEqual(self.node.yellow_lower.tolist(), [24,140,120])
        self.assertEqual(self.node.duck_lower.tolist(), [15,90,70])
        self.assertIsNotNone(self.node._road_geometry)

    def test_thin_dashed_dividers_are_not_ducks(self):
        image=self.scene(duck=False)
        image[240:480,224:237]=0
        for y in (245,300,355,410):
            cv2.rectangle(image,(224,y),(236,y+25),(0,255,255),-1)
        self.assertEqual(self.node.detect_ducks_bgr(image), [])

    def test_missing_boundary_cannot_start_pass(self):
        self.enable()
        image=self.scene()
        image[:,40:65]=0
        self.frame(image)
        self.assertEqual(self.node.avoidance_state, "idle")
        self.assertEqual(self.speeds(), (0,0))
        self.assertTrue(self.node.obstacle_stop_latched)

    def test_full_visual_sequence_retains_route_and_speed(self):
        route=list(self.node.route)
        index=self.node.route_index
        scale=self.node.speed_scale
        self.start_pass()
        self.frame(self.scene(left_lane=True, near=True))
        for _ in range(12):
            self.frame(self.scene(left_lane=True, duck=False))
            self.assertLessEqual(max(self.speeds()), self.node.avoidance_speed)
        self.assertEqual(self.node.avoidance_state, "return_right")
        for _ in range(5):
            self.frame(self.scene(duck=False))
        self.assertEqual(self.node.avoidance_state, "idle")
        self.assertEqual((self.node.route,self.node.route_index,self.node.speed_scale),
                         (route,index,scale))
        self.frame(self.scene(duck=False))
        self.assertGreater(max(self.speeds()), 0)

    def test_early_disappearance_latches_fault(self):
        self.start_pass()
        self.frame(self.scene(left_lane=True,duck=False))
        self.assertEqual(self.node.avoidance_state,"fault")
        self.assertEqual(self.speeds(),(0,0))
        self.frame(self.scene())
        self.assertEqual(self.speeds(),(0,0))

    def test_blocked_left_lane_and_second_duck_stop(self):
        for color in ((255,0,0),(0,255,255)):
            self.setUp()
            self.enable()
            image=self.scene()
            cv2.rectangle(image,(95,350),(150,420),color,-1)
            self.frame(image)
            self.assertEqual(self.speeds(),(0,0))
            self.assertTrue(self.node.obstacle_stop_latched)

    def test_camera_timeout_and_invalid_stamp_abort(self):
        for timeout in (False,True):
            self.setUp()
            self.start_pass()
            if timeout:
                self.now+=1
                self.node.check_camera_timeout(None)
            else:
                self.node.callback(self.message(self.scene(),stamp=0))
            self.assertEqual(self.node.avoidance_state,"fault")
            self.assertEqual(self.speeds(),(0,0))

    def test_camera_gap_without_watchdog_aborts(self):
        self.start_pass()
        self.now+=1
        self.frame(self.scene(left_lane=True))
        self.assertEqual(self.node.avoidance_state,"fault")
        self.assertEqual(self.speeds(),(0,0))

    def test_red_line_and_geometry_loss_abort(self):
        for red in (True,False):
            self.setUp()
            self.start_pass()
            image=self.scene(left_lane=True)
            if red:
                cv2.rectangle(image,(270,350),(580,375),(0,0,255),-1)
            else:
                image[:,580:605]=0
            self.frame(image)
            self.assertEqual(self.node.avoidance_state,"fault")
            self.assertEqual(self.speeds(),(0,0))
            if red:
                self.assertTrue(self.node.red_stop_latched)

    def test_stop_shutdown_and_command_during_pass(self):
        self.start_pass()
        self.node.command_callback(NS(data=json.dumps(dict(
            id="turn",action="turn",value="left",issued_at=self.now))))
        self.assertFalse(self.node._last_command["accepted"])
        self.node.command_callback(NS(data=json.dumps(dict(id="s",action="stop"))))
        self.assertEqual(self.speeds(),(0,0))
        self.assertEqual(self.node.avoidance_state,"fault")
        self.setUp()
        self.start_pass()
        self.node.on_shutdown()
        self.frame(self.scene(left_lane=True))
        self.assertEqual(self.speeds(),(0,0))

    def test_debug_and_stand_cap(self):
        self.node.avoidance_enabled=self.node.avoidance_calibrated=True
        self.frame(self.scene())
        self.assertEqual(self.speeds(),(0,0))
        self.setUp()
        self.node.max_speed=.07
        self.node.base_speed=.05
        self.start_pass()
        self.assertLessEqual(max(self.speeds()),.04)

    def test_timeout_and_lost_heartbeat_abort(self):
        self.start_pass()
        self.node._avoidance_started=self.now-self.node.avoidance_timeout
        self.frame(self.scene(left_lane=True))
        self.assertEqual(self.node.avoidance_state,"fault")
        self.assertEqual(self.speeds(),(0,0))
        self.setUp()
        self.start_pass()
        self.node._active_client_id="lost"
        self.frame(self.scene(left_lane=True))
        self.assertEqual(self.node.avoidance_state,"fault")
        self.assertEqual(self.speeds(),(0,0))

    def test_return_corridor_reappearance_aborts(self):
        self.start_pass()
        self.frame(self.scene(left_lane=True,near=True))
        for _ in range(12):
            self.frame(self.scene(left_lane=True,duck=False))
        self.assertEqual(self.node.avoidance_state,"return_right")
        self.frame(self.scene(left_lane=True))
        self.assertEqual(self.node.avoidance_state,"fault")
        self.assertEqual(self.speeds(),(0,0))

    def test_passing_steering_matches_existing_flip_convention(self):
        for flip in (True,False):
            self.setUp()
            self.enable()
            self.node.flip_steering=flip
            for _ in range(5):
                self.frame(self.scene())
            left,right=self.speeds()
            self.assertGreater(right if flip else left, left if flip else right)

    def test_low_speed_requires_longer_clearance_allowance(self):
        self.start_pass()
        self.node.speed_scale=.25
        self.frame(self.scene(left_lane=True,near=True))
        for _ in range(12):
            self.frame(self.scene(left_lane=True,duck=False))
        self.assertEqual(self.node.avoidance_state,"passing")
        for _ in range(34):
            self.frame(self.scene(left_lane=True,duck=False))
        self.assertEqual(self.node.avoidance_state,"return_right")

    def test_slow_extra_perception_does_not_advance_pass(self):
        self.start_pass()
        old_index=self.node.route_index
        original=self.mod.LaneFollowerNode.detect_obstacle_bgr
        def slow(node, image):
            self.now+=.6
            return original(node,image)
        with patch.object(self.mod.LaneFollowerNode,"detect_obstacle_bgr",slow):
            self.frame(self.scene(left_lane=True,near=True))
        self.assertEqual(self.node.avoidance_state,"fault")
        self.assertEqual(self.node.route_index,old_index)
        self.assertEqual(self.speeds(),(0,0))

    def test_bad_configuration_rejected(self):
        for params in ({"~avoidance_enabled": True},
                       {"~avoidance_calibrated":"yes"},
                       {"~avoidance_speed":float("nan")},
                       {"~avoidance_speed":.2},
                       {"~avoidance_clear_seconds":0},
                       {"~duck_lower":[40,90,70],"~duck_upper":[15,255,255]}):
            self.params=params
            with patch.dict(self.mod.os.environ,VEHICLE_NAME="duck2"):
                with self.assertRaises(ValueError):
                    self.mod.LaneFollowerNode("invalid")


def load_tests(loader, tests, pattern):
    return unittest.TestSuite(DuckTests(n) for n in DuckTests.__dict__ if n.startswith("test_"))

if __name__ == "__main__":
    unittest.main()
