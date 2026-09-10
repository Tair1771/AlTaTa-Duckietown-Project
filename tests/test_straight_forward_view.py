"""Distant exit observations aim a crossing but cannot finish it."""
import unittest
import numpy as np
import cv2
import test_lane_follower as fixtures

class StraightForwardViewTests(unittest.TestCase):
    setUp = fixtures.LaneTests.setUp
    subscribe = fixtures.LaneTests.subscribe
    message = fixtures.LaneTests.message

    def image(self, shift=0, reverse=False, missing=None, short=False):
        image=np.zeros((480,640,3),np.uint8)
        for row in range(145,153 if short else 190):
            t=row-145
            a,b=int(285-1.2*t+shift),int(355+1.2*t+shift)
            if reverse:a,b=b,a
            if missing!='yellow':cv2.line(image,(a-3,row),(a+3,row),(0,255,255),1)
            if missing!='white':cv2.line(image,(b-4,row),(b+4,row),(255,255,255),1)
        return image

    def configure(self):
        n=self.node
        n.navigation_state='reacquiring';n._active_turn='straight'
        n.require_client_heartbeat=True;n._camera_valid=True;n._last_frame_time=self.now
        n.flip_steering=True;n.max_speed=.20;n.min_active_wheel_speed=.03
        return n

    def aim(self, image):
        n=self.configure();n._junction_forward_geometry=n.straight_forward_geometry(image)
        n._junction_lane_geometry={'valid':False}
        result=None
        for _ in range(4):
            self.now+=.1;n._last_frame_time=self.now
            result=n.straight_visual_wheels(.15)
        return result

    def test_exit_above_normal_roi_is_seen_but_not_reacquired(self):
        n=self.configure();image=self.image()
        n.detect_lane_bgr(image)
        self.assertFalse(n._junction_lane_geometry['steering_valid'])
        self.assertTrue(n._junction_forward_geometry['usable'])
        self.assertFalse(n._junction_exit_geometry['valid'])
        self.assertFalse(n._junction_exit_geometry['near_support'])
        self.assertEqual(self.aim(image),(.15,.15,0.))

    def test_correct_direction_and_small_authority(self):
        for shift in [-60,60]:
            n=self.node;n.reset_steering()
            left,right,steering=self.aim(self.image(shift=shift))
            self.assertLessEqual(abs(steering),.025)
            self.assertGreaterEqual(min(left,right),.125)
            self.assertGreater(right,left) if shift<0 else self.assertGreater(left,right)

    def test_partial_reversed_and_transverse_markings_do_not_authorize_aiming(self):
        images=[self.image(reverse=True),self.image(missing='yellow'),
                self.image(missing='white'),self.image(short=True)]
        transverse=np.zeros((480,640,3),np.uint8)
        cv2.rectangle(transverse,(180,160),(305,165),(0,255,255),-1)
        cv2.rectangle(transverse,(355,160),(470,165),(255,255,255),-1)
        images.append(transverse)
        for image in images:
            self.assertFalse(self.node.straight_forward_geometry(image)['usable'])
            self.assertIsNone(self.aim(image))

    def test_left_right_junctions_and_ordinary_roads_are_unchanged(self):
        n=self.configure();g=n.straight_forward_geometry(self.image())
        for state,turn in [('following',None),('red_stop','straight'),('reacquiring','left'),('crossing','right')]:
            n.navigation_state=state;n._active_turn=turn;n._junction_forward_geometry=g
            self.assertIsNone(n.straight_forward_wheels(.15))
            n.detect_lane_bgr(self.image())
            self.assertIsNone(n._junction_forward_geometry)

    def test_freshness_confirmation_and_near_lane_takeover(self):
        n=self.configure();n._junction_forward_geometry=n.straight_forward_geometry(self.image())
        self.assertIsNone(n.straight_forward_wheels(.15))
        self.aim(self.image());self.assertTrue(n._junction_forward_used)
        self.now+=.6;self.assertIsNone(n.straight_forward_wheels(.15))
        self.assertFalse(n._junction_forward_used)
        self.aim(self.image());n.junction_straight_lateral_gain=.25;n.junction_straight_heading_gain=.3
        n._junction_lane_geometry=dict(valid=True,steering_valid=True,lateral_error=.08,heading_error=0.)
        left,right,_=n.straight_visual_wheels(.15)
        self.assertGreater(left,right)
        self.assertFalse(n._junction_forward_used)
        self.assertIsNone(n._junction_forward_since)

    def test_far_evidence_does_not_bypass_stop_output_gate(self):
        n=self.configure();wheels=self.aim(self.image())
        n.manual_stop=True;n.publish_wheels(*wheels[:2])
        self.assertEqual(n._last_wheel_speeds,(0.,0.))

    def points_image(self, points):
        """Render recorded row pairs, not a claimed reconstruction of the scene."""
        image=np.zeros((480,640,3),np.uint8)
        for row,yellow,white in points:
            for x,radius,color in ((yellow,3,(0,255,255)),(white,4,(255,255,255))):
                x=int(round(x))
                cv2.rectangle(image,(x-radius,row-1),(x+radius,row+1),color,-1)
        return image

    def test_logged_far_end_fragments_do_not_discard_the_remaining_exit(self):
        # Failed straight-exit telemetry had 22/16 ordered pairs, but one/two
        # far-end fragments invalidated the linear fit of the entire corridor.
        recordings=[
            [(159,162,276),(162,178.5,284),(165,178.5,288),(168,176.5,289.5),
             (171,173.5,291.5),(174,169,292.5),(177,163.5,293.5),(180,160,295),
             (183,156,296),(186,150,297.5),(189,144.5,298.5),(192,141.5,300),
             (195,134.5,301.5),(198,130.5,302),(201,126.5,303),(204,121.5,304.5),
             (207,119.5,306),(213,105,308.5),(216,103.5,310),(219,97.5,311),
             (222,94.5,312.5),(225,91,313.5)],
            [(168,111.5,235.5),(171,116,241),(174,119.5,246),(177,120,249),
             (180,119.5,249.5),(183,112.5,250),(186,112,251),(189,107,251.5),
             (192,101.5,252),(195,101.5,252),(198,91,252.5),(201,88,253),
             (204,83,253),(207,78.5,253.5),(210,79,254),(216,65,255)],
        ]
        for points,expected_trim in zip(recordings,(1,2)):
            with self.subTest(expected_trim=expected_trim):
                image=self.points_image(points)
                g=self.node.straight_forward_geometry(image)
                self.assertTrue(g['usable'])
                self.assertEqual(g['trimmed_far_pairs'],expected_trim)
                self.assertGreaterEqual(len(g['pairs']),len(points)-2)
                left,right,steering=self.aim(image)
                self.assertGreater(right,left)
                self.assertLessEqual(abs(steering),.025)
                self.assertFalse(self.node.straight_reacquisition_good())

    def test_only_small_far_prefix_can_be_discarded(self):
        clean=[(147+3*i,270-3*i,350+3*i) for i in range(22)]
        g=self.node.straight_forward_geometry(self.points_image(clean))
        self.assertTrue(g['usable']);self.assertEqual(g['trimmed_far_pairs'],0)
        for indices in [set(range(3)),set(range(0,22,2)),set(range(8,12))]:
            damaged=[(row,yellow-(25 if i in indices else 0),white)
                     for i,(row,yellow,white) in enumerate(clean)]
            self.assertFalse(self.node.straight_forward_geometry(
                self.points_image(damaged))['usable'])

    def test_fresh_slower_camera_cadence_keeps_confirmed_aiming(self):
        n=self.configure()
        for cadence in (.25,.30):
            with self.subTest(cadence=cadence):
                n.reset_steering()
                n._junction_forward_geometry=n.straight_forward_geometry(self.image(shift=-60))
                n._last_frame_time=self.now
                self.assertIsNone(n.straight_forward_wheels(.15))
                for _ in range(5):
                    self.now+=cadence;n._last_frame_time=self.now
                    left,right,steering=n.straight_forward_wheels(.15)
                    self.assertGreater(right,left)
                    self.assertTrue(n._junction_forward_used)
                    self.assertLessEqual(abs(steering),.025)
                self.now+=.51
                self.assertIsNone(n.straight_forward_wheels(.15))
                self.assertFalse(n._junction_forward_used)
                n._last_frame_time=self.now
                self.assertIsNone(n.straight_forward_wheels(.15))

    def test_geometry_loss_still_requires_new_confirmation(self):
        n=self.configure();self.aim(self.image(shift=-60))
        n._junction_forward_geometry={'usable':False}
        self.assertIsNone(n.straight_forward_wheels(.15))
        self.assertIsNone(n._junction_forward_since)
        n._junction_forward_geometry=n.straight_forward_geometry(self.image(shift=-60))
        self.assertIsNone(n.straight_forward_wheels(.15))

if __name__=='__main__':unittest.main()
