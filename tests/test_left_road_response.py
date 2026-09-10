"""Real image geometry and wheel mixer regressions for the opt-in curve boost."""
import unittest
import cv2
import numpy as np
import test_lane_follower as fixtures

class LeftRoadResponseTests(unittest.TestCase):
    subscribe = fixtures.LaneTests.subscribe
    message = fixtures.LaneTests.message

    def setUp(self):
        fixtures.LaneTests.setUp(self)
        n = self.node
        n.base_speed, n.max_speed, n.max_steering = .09, .20, .11
        n.k_p, n.near_center_k_p, n.full_gain_error = .75, .35, .09
        n.deadband, n.steering_bias, n.alpha = .05, .0075, .20
        n.smooth_steering_deadband = True
        n.flip_steering = n.drive_enabled = True
        n.min_active_wheel_speed = .03
        n.road_left_curve_boost = True
        n.navigation_state, n._junction_phase = "following", "complete"
        n._active_turn = None
        n._junction_approach_active = n._red_line_visible = n.manual_stop = False
        n._sharp_corner_state = "idle"

    def road(self, bend=.065, offset=0, yaw=0, dashed=False, missing=None):
        image = np.zeros((480,640,3),np.uint8)
        for y in range(192,440):
            t = (y-192)/216.
            bow = bend*640*4*t*(1-t)
            shift = offset + yaw*(t-.5)
            yellow = int(65+70*t+bow+shift)
            white = int(270+210*t+bow+shift)
            if missing != "yellow" and (not dashed or (y//22)%2==0):
                cv2.line(image,(yellow-7,y),(yellow+7,y),(0,255,255),1)
            if missing != "white":
                cv2.line(image,(white-10,y),(white+10,y),(255,255,255),1)
        return image

    def tick(self, image, error=0., dt=.1):
        self.now += dt
        n=self.node
        n.detect_lane_bgr(image)
        n._camera_valid=True
        n._last_frame_time=self.now
        return n.compute_wheel_speeds(error)

    def settle(self, image, error=0.):
        for _ in range(15): result=self.tick(image,error)
        return result

    def test_actual_curved_borders_reach_requested_pair_after_confirmation(self):
        image=self.road(dashed=True)
        self.assertTrue(self.node.detect_left_road_curve(image)["candidate"])
        self.tick(image)
        self.assertFalse(self.node._road_left_boost_active)
        left,right,steer=self.settle(image)
        self.assertTrue(self.node._road_left_boost_active)
        self.assertAlmostEqual(left,.03)
        self.assertAlmostEqual(right,.20)
        self.assertAlmostEqual(steer,.085)

    def test_offset_and_yawed_straights_never_get_curve_power(self):
        for offset in [-35,0,35]:
            for yaw in [-70,0,70]:
                image=self.road(bend=0,offset=offset,yaw=yaw)
                self.assertFalse(self.node.detect_left_road_curve(image)["candidate"])
                self.node.reset_steering()
                boosted=self.settle(image,error=-.10)
                self.assertFalse(self.node._road_left_boost_active)
                self.node.road_left_curve_boost=False
                self.node.reset_steering()
                baseline=self.settle(image,error=-.10)
                self.assertEqual(boosted,baseline)
                self.node.road_left_curve_boost=True

    def test_right_curve_missing_borders_and_distant_fragments_rejected(self):
        images=[self.road(bend=-.065),self.road(missing="yellow"),self.road(missing="white")]
        distant=self.road();distant[300:]=0;images.append(distant)
        for image in images:
            self.assertFalse(self.node.detect_left_road_curve(image)["candidate"])
            self.settle(image)
            self.assertFalse(self.node._road_left_boost_active)

    def test_straight_exit_clears_boost_and_restores_baseline(self):
        self.settle(self.road())
        self.assertTrue(self.node._road_left_boost_active)
        self.tick(self.road(bend=0))
        self.assertFalse(self.node._road_left_boost_active)
        left,right,_=self.settle(self.road(bend=0))
        self.assertAlmostEqual(left,.0975)
        self.assertAlmostEqual(right,.0825)

    def test_junction_red_pause_and_stop_never_receive_curve_override(self):
        for key,value in [("_junction_approach_active",True),("_red_line_visible",True),
                          ("_active_turn","straight"),("navigation_state","crossing"),
                          ("navigation_state","reacquiring"),("_sharp_corner_state","turning"),
                          ("manual_stop",True),("red_stop_latched",True)]:
            n=self.node;old=getattr(n,key)
            self.settle(self.road());setattr(n,key,value)
            self.tick(self.road())
            self.assertFalse(n._road_left_boost_active,key)
            setattr(n,key,old)
        n.live.enabled=n.live.active=True;n.live.paused_at=self.now
        self.tick(self.road())
        self.assertFalse(n._road_left_boost_active)
        n.publish_wheels(.03,.20)
        self.assertEqual(n._last_wheel_speeds,(0.,0.))

    def test_stale_camera_loss_and_right_correction_clear_boost(self):
        n=self.node
        self.settle(self.road());self.now+=1.
        n.compute_wheel_speeds(0.)
        self.assertFalse(n._road_left_boost_active)
        self.settle(self.road())
        # Recover right only for an observed large left-of-lane offset, not
        # an injected centroid error contradicting the unchanged image.
        self.now += .1
        n.callback(self.message(self.road(offset=100)))
        self.assertFalse(n._road_left_boost_active)
        self.settle(self.road());wheels=n.compute_wheel_speeds(None)
        self.assertEqual(wheels[:2],(0.,0.))
        self.assertFalse(n._road_left_boost_active)

    def test_parameter_disabled_retains_normal_correction_on_curves(self):
        n=self.node;n.road_left_curve_boost=False
        left,right,_=self.settle(self.road(),error=0.)
        self.assertFalse(n._road_left_boost_active)
        self.assertAlmostEqual(left,.0975);self.assertAlmostEqual(right,.0825)

    def test_frame_callback_copies_new_geometry_to_control_state(self):
        self.now += .1
        self.node.callback(self.message(self.road()))
        self.assertTrue(self.node._road_left_shape["candidate"])

if __name__ == "__main__":
    unittest.main()
