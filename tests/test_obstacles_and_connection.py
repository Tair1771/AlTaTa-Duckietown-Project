import json
import unittest
from types import SimpleNamespace as NS
import cv2
from test_lane_follower import LaneTests

class SafetyTests(LaneTests):
    def command(self,action,**kwargs):
        payload=dict(id=str(len(self.node._seen_commands)+1),issued_at=self.now,action=action)
        payload.update(kwargs)
        self.node.command_callback(NS(data=json.dumps(payload)))
        return self.node._last_command

    def heartbeat(self,client="laptop",issued=None):
        self.node.command_callback(NS(data=json.dumps(dict(
            id="hb",action="heartbeat",client_id=client,
            issued_at=self.now if issued is None else issued))))

    def obstacle(self,color=(0,255,255)):
        frame=self.image()
        cv2.rectangle(frame,(285,365),(360,435),color,-1)
        return frame

    def frame(self,frame):
        self.now+=.1
        self.deliver(frame)

    def claim(self):
        self.node.drive_enabled=True
        self.heartbeat()
        self.assertTrue(self.command("continue",client_id="laptop")["accepted"])

    def test_colored_and_bright_obstacle_candidates(self):
        for color in [(0,255,255),(255,0,0),(0,255,0),(180,180,180)]:
            frame=self.obstacle(color)
            self.node.detect_lane_bgr(frame)
            self.assertIsNotNone(self.node.detect_obstacle_bgr(frame),color)

    def test_lines_small_and_outside_objects_are_rejected(self):
        for frame in [self.image(),self.image(((260,370),(580,390)))]:
            self.node.detect_lane_bgr(frame)
            self.assertIsNone(self.node.detect_obstacle_bgr(frame))
        for box in [((285,365),(295,375)),((20,365),(100,435)),((285,160),(360,230))]:
            frame=self.image()
            cv2.rectangle(frame,box[0],box[1],(255,0,0),-1)
            self.node.detect_lane_bgr(frame)
            self.assertIsNone(self.node.detect_obstacle_bgr(frame),box)

    def test_dark_obstacle_is_a_documented_detection_gap(self):
        frame=self.obstacle((10,10,10))
        self.node.detect_lane_bgr(frame)
        self.assertIsNone(self.node.detect_obstacle_bgr(frame))

    def test_obstacle_stop_needs_clear_frames_and_continue(self):
        self.node.drive_enabled=True
        self.frame(self.image())
        self.assertGreater(max(self.speeds()),0)
        self.frame(self.obstacle())
        self.assertEqual(self.speeds(),(0,0))
        self.assertFalse(self.command("continue")["accepted"])
        self.frame(self.image())
        self.assertFalse(self.command("continue")["accepted"])
        for _ in range(6):
            self.frame(self.image())
        self.assertEqual(self.speeds(),(0,0))
        self.assertTrue(self.command("continue")["accepted"])
        self.frame(self.image())
        self.assertGreater(max(self.speeds()),0)

    def test_clear_interval_resets_on_camera_loss(self):
        self.node.drive_enabled=True
        self.frame(self.obstacle())
        self.frame(self.image())
        self.now+=1.
        self.node.check_camera_timeout(None)
        self.frame(self.image())
        self.assertFalse(self.command("continue")["accepted"])

    def test_obstacle_mid_junction_faults_without_advancing(self):
        self.node.drive_enabled=True
        self.node.navigation_state="crossing"
        old_index=self.node.route_index
        self.frame(self.obstacle())
        self.assertEqual(self.node.navigation_state,"fault")
        self.assertEqual(self.node.route_index,old_index)
        self.assertEqual(self.speeds(),(0,0))

    def test_heartbeat_is_required_in_supervised_mode(self):
        self.node.require_client_heartbeat=True
        self.node.drive_enabled=True
        self.assertFalse(self.command("continue")["accepted"])
        self.assertFalse(self.command("continue",client_id="laptop")["accepted"])
        self.heartbeat()
        self.assertTrue(self.command("continue",client_id="laptop")["accepted"])

    def test_client_timeout_stops_once_and_reconnect_does_not_resume(self):
        self.claim()
        self.frame(self.image())
        initial_epoch=self.node._control_epoch
        self.now+=2.1
        self.node._last_frame_time=self.now
        self.node.check_camera_timeout(None)
        self.assertEqual(self.speeds(),(0,0))
        self.assertTrue(self.node.manual_stop)
        self.assertEqual(self.node._control_epoch,initial_epoch+1)
        self.node.check_camera_timeout(None)
        self.assertEqual(self.node._control_epoch,initial_epoch+1)
        self.heartbeat()
        self.frame(self.image())
        self.assertEqual(self.speeds(),(0,0))
        self.assertTrue(self.command("continue",client_id="laptop")["accepted"])
        self.frame(self.image())
        self.assertGreater(max(self.speeds()),0)

    def test_stale_or_replayed_heartbeat_cannot_extend_lease(self):
        self.claim()
        original=self.now
        self.now+=1.0
        self.heartbeat(issued=original)
        self.now+=1.1
        self.node._last_frame_time=self.now
        self.node.check_camera_timeout(None)
        self.assertTrue(self.node.client_expired())
        self.assertEqual(self.speeds(),(0,0))

    def test_timeout_during_junction_invalidates_position(self):
        self.claim()
        self.node.navigation_state="crossing"
        self.now+=2.1
        self.node._last_frame_time=self.now
        self.node.check_camera_timeout(None)
        self.assertEqual(self.node.navigation_state,"fault")
        self.heartbeat()
        self.assertFalse(self.command("continue",client_id="laptop")["accepted"])

    def test_other_client_cannot_take_over_without_stop(self):
        self.claim()
        self.heartbeat(client="other")
        self.assertFalse(self.command("speed_up",client_id="other")["accepted"])
        self.assertTrue(self.command("stop",client_id="other")["accepted"])
        self.heartbeat(client="other")
        self.assertTrue(self.command("continue",client_id="other")["accepted"])

    def test_heartbeat_preserves_user_ack(self):
        self.command("stop")
        ack=self.node._last_command.copy()
        self.heartbeat()
        self.assertEqual(self.node._last_command,ack)

def load_tests(loader,tests,pattern):
    names=[n for n in SafetyTests.__dict__ if n.startswith("test_")]
    return unittest.TestSuite(SafetyTests(n) for n in names)

if __name__=="__main__":
    unittest.main()
