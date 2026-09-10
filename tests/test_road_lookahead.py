"""Offline road entry checks; no robot connection or camera is required."""
import unittest
import cv2
import numpy as np
import test_lane_follower as fixtures


class RoadLookaheadTests(unittest.TestCase):
    setUp = fixtures.LaneTests.setUp
    subscribe = fixtures.LaneTests.subscribe
    message = fixtures.LaneTests.message

    def setup_road(self):
        n = self.node
        n.road_left_lookahead = True
        n.navigation_state = 'following'
        n._lane_both_visible = True
        n.k_p = .75
        n.junction_straight_lane_target_fraction = .49
        n._road_curve_geometry = dict(image_width=640, pairs=[
            dict(row_fraction=f, center_x=c, width=w)
            for f,c,w in [(.04,301,288),(.12,313,320),(.20,321,356),
                          (.40,331,426),(.55,321,484)]])
        return n

    def test_recorded_entry_requests_left_before_centroid_does(self):
        n=self.setup_road()
        self.assertLess(n.anticipate_left_bend(.012), -.02)
        self.assertTrue(n._road_left_lookahead_used)
        self.assertEqual(n.anticipate_left_bend(-.1), -.1)

    def test_heading_guard_corrects_conflicting_sign_even_if_raw_is_smaller(self):
        n=self.setup_road();n.road_heading_guard=True
        n._junction_lane_geometry=dict(valid=True,near_support=True,
                                      lateral_error=.02880,heading_error=.06433)
        n.junction_straight_lateral_gain=.25;n.junction_straight_heading_gain=.3
        self.assertLess(n.guard_road_heading(.011),0)

    def test_straight_and_right_bends_keep_their_existing_request(self):
        n=self.setup_road()
        for far_center in (326,340):
            for p in n._road_curve_geometry['pairs']:
                if p['row_fraction']<=.20:p['center_x']=far_center
            self.assertEqual(n.anticipate_left_bend(.025),.025)

    def test_missing_partial_reversed_and_left_offset_corridors_are_rejected(self):
        for kind in ('missing','single_far','single_near','reversed_width','left_offset'):
            n=self.setup_road();g=n._road_curve_geometry
            if kind=='missing':n._lane_both_visible=False
            elif kind=='single_far':g['pairs']=g['pairs'][2:]
            elif kind=='single_near':g['pairs']=g['pairs'][:-1]
            elif kind=='reversed_width':
                for p in g['pairs']:p['width']=600-p['width']
            else:
                for p in g['pairs']:p['center_x']+=80
            with self.subTest(kind=kind):self.assertEqual(n.anticipate_left_bend(.02),.02)

    def test_junctions_red_stops_and_sharp_pivots_do_not_use_road_assist(self):
        for field,value in [('road_left_lookahead',False),('navigation_state','crossing'),
                            ('navigation_state','reacquiring'),('_junction_phase','aligning'),
                            ('_junction_approach_active',True),('_red_line_visible',True),
                            ('_sharp_corner_state','turning')]:
            n=self.setup_road();setattr(n,field,value)
            with self.subTest(field=field):self.assertEqual(n.anticipate_left_bend(.02),.02)

    def scene(self):
        im=np.zeros((480,640,3),np.uint8)
        # Far centre 270, near centre 320: continuous leftward lane ahead.
        cv2.line(im,(150,240),(65,455),(0,255,255),18)
        cv2.line(im,(390,240),(575,455),(220,220,220),18)
        return im

    def test_live_callback_copies_evidence_and_obeys_stop_and_steering_rate(self):
        n=self.setup_road();n.drive_enabled=True;n.manual_stop=False;n.flip_steering=True
        n.max_speed=.20;n.max_steering=.11;n.base_speed=.09
        for _ in range(20):
            self.now+=1/30
            previous=n.prev_steering
            n.callback(self.message(self.scene()))
            self.assertLessEqual(abs(n.prev_steering-previous),n.max_steering_change+1e-6)
        self.assertIsNotNone(n._road_curve_geometry)
        self.assertTrue(n._road_left_lookahead_used)
        self.assertGreater(self.messages[-1].vel_right,self.messages[-1].vel_left)
        n.manual_stop=True;self.now+=.04;n.callback(self.message(self.scene()))
        self.assertEqual((self.messages[-1].vel_left,self.messages[-1].vel_right),(0,0))

    def test_missing_images_never_reuse_bend_to_move(self):
        n=self.setup_road();n.lost_speed=0
        self.assertEqual(n.compute_wheel_speeds(None)[:2],(0,0))
        self.assertFalse(n._road_left_lookahead_used)

    def test_finished_junction_restores_same_road_reference(self):
        n=self.setup_road();n.road_white_reference_value=170
        n.white_lower=np.array([0,0,150],np.uint8)
        initial=n.detect_lane_bgr(self.scene())[0]
        n._junction_phase='complete'
        self.assertEqual(n.detect_lane_bgr(self.scene())[0],initial)
        self.assertTrue(n._road_white_reference_used)

    def far_scene(self):
        im=np.zeros((480,640,3),np.uint8)
        cv2.polylines(im,[np.array([(65,432),(170,240),(120,192)])],False,(0,255,255),16)
        cv2.polylines(im,[np.array([(575,432),(470,240),(290,192)])],False,(220,220,220),16)
        return im

    def test_far_curve_ignores_detached_background_white(self):
        n=self.setup_road();im=self.far_scene()
        cv2.rectangle(im,(550,180),(638,233),(220,220,220),-1)
        n.detect_lane_bgr(im)
        self.assertEqual(n._road_curve_geometry['source'],'tracked_forward_borders')
        far=[p for p in n._road_curve_geometry['pairs'] if p['row_fraction']<=.1]
        self.assertGreaterEqual(len(far),2)
        self.assertTrue(all(p['white_x']<480 for p in far))
        self.assertLess(n.anticipate_left_bend(.02),-.03)

    def test_disconnected_far_fragments_do_not_form_a_road(self):
        n=self.setup_road();im=np.zeros((480,640,3),np.uint8)
        cv2.line(im,(120,192),(170,235),(0,255,255),16)
        cv2.line(im,(290,192),(470,235),(220,220,220),16)
        self.assertEqual(n.road_forward_geometry(im)['pairs'],[])

    def confirmed_curve(self):
        n=self.setup_road();n.manual_stop=False;n.drive_enabled=True;n.flip_steering=True
        n.base_speed=.09;n.max_speed=.20;n.max_steering=.11
        n.temporal_lane_width_fallback=True;n.temporal_lane_width_timeout=.3
        for _ in range(20):
            self.now+=.05;n.callback(self.message(self.scene()))
        self.assertIsNotNone(n._left_curve_confirmed_at)
        return n

    def test_confirmed_curve_keeps_left_request_briefly_then_expires(self):
        n=self.confirmed_curve();im=self.scene();im[:,:220]=0
        confirmed=n._left_curve_confirmed_at
        for _ in range(7):
            self.now+=.1;n.callback(self.message(im))
            self.assertTrue(n._road_left_gap_active)
            self.assertGreater(self.messages[-1].vel_right,self.messages[-1].vel_left)
            self.assertEqual(n._left_curve_confirmed_at,confirmed)
        self.now+=.11;n.callback(self.message(im))
        self.assertEqual((self.messages[-1].vel_left,self.messages[-1].vel_right),(0,0))
        self.assertFalse(n._road_left_gap_active)

    def test_yellow_loss_without_confirmed_curve_does_not_arm_turn(self):
        n=self.setup_road();n.manual_stop=False;n.temporal_lane_width_fallback=True
        n.temporal_lane_width_timeout=.3
        im=np.zeros((480,640,3),np.uint8)
        cv2.line(im,(170,240),(170,455),(0,255,255),18)
        cv2.line(im,(470,240),(470,455),(220,220,220),18)
        n.detect_lane_bgr(im);im[:,:220]=0;self.now+=.4
        error,_,_=n.detect_lane_bgr(im)
        self.assertIsNone(error);self.assertFalse(n._road_left_gap_active)

    def test_curve_memory_cannot_override_manual_stop_red_or_missing_white(self):
        for kind in ('manual','red','blank','jump'):
            n=self.confirmed_curve();im=self.scene();im[:,:220]=0
            if kind=='manual':n.manual_stop=True
            elif kind=='red':n._red_line_visible=True
            elif kind=='blank':im[:]=0
            else:im=np.roll(im,-130,axis=1)
            self.now+=.4
            error,_,_=n.detect_lane_bgr(im)
            self.assertIsNone(error,kind);self.assertFalse(n._road_left_gap_active,kind)

    def test_driver_feedback_is_read_only_and_distinguishes_zero_samples(self):
        from types import SimpleNamespace
        n=self.node;count=len(self.messages)
        for left,right in ((.03,.2),(0,0),(float('nan'),.2)):
            n.executed_wheels_callback(SimpleNamespace(vel_left=left,vel_right=right))
        evidence=n.status()['wheel_evidence']
        self.assertEqual(evidence['executed_samples'],2)
        self.assertEqual(evidence['executed_zero_samples'],1)
        self.assertEqual(evidence['executed'],(0,0))
        self.assertEqual(len(self.messages),count)


if __name__=='__main__':unittest.main()
