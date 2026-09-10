"""Short connector and road-corner route transitions, without physical motion."""
import unittest
import test_navigation as fixtures


class SecondRedRouteTests(unittest.TestCase):
    setUp = fixtures.NavigationTests.setUp
    subscribe = fixtures.NavigationTests.subscribe
    message = fixtures.NavigationTests.message
    command = fixtures.NavigationTests.command
    configure_route = fixtures.NavigationTests.configure_route
    step = fixtures.NavigationTests.step
    reach_stop = fixtures.NavigationTests.reach_stop
    enter_crossing = fixtures.NavigationTests.enter_crossing
    prepare_guarded_left_search = fixtures.NavigationTests.prepare_guarded_left_search

    def near(self):
        return dict(valid=True, steering_valid=True, near_support=True,
                    lateral_error=-.145, heading_error=-.07)

    def left_exit(self):
        self.prepare_guarded_left_search()
        n=self.node
        n.route=['A','E','B','C']
        n.junction_left_visual_latch=True
        n._junction_left_visual_entry=True
        n._junction_lane_geometry=self.near()
        n.junction_straight_lateral_gain=.25
        n.junction_straight_heading_gain=.30
        return n

    def arm_next_red(self):
        n=self.left_exit()
        for _ in range(5): self.step(error=-.145)
        self.assertTrue(n._junction_left_red_rearmed)
        self.assertEqual(n.route_index,1)
        return n

    def test_second_red_advances_left_once_even_when_borders_end(self):
        n=self.arm_next_red()
        n._junction_lane_geometry=None
        n._white_boundary_visible=False
        self.assertEqual(self.step(error=None,red=True,both=False),(0,0,0))
        self.assertEqual(n.navigation_state,'red_stop')
        self.assertEqual(n.route_index,2)
        self.assertEqual(n.upcoming_junction_turn(),'right')
        self.assertEqual(n._last_junction_result['turn'],'left')
        for _ in range(10): self.step(red=True)
        self.assertEqual(n.route_index,2)

    def test_second_red_dwell_then_right_pivot_without_yellow(self):
        n=self.arm_next_red()
        n._white_boundary_visible=False
        n._yellow_boundary_visible=False
        n._junction_lane_geometry=None
        n.auto_continue=True
        n.max_speed=.20
        n.junction_right_speed=n.junction_right_bias=.10
        self.step(error=None,red=True,both=False)
        for _ in range(10):
            self.step(error=None,red=True,both=False)
            self.assertEqual(n.navigation_state,'red_stop')
        for _ in range(15):
            self.step(error=None,red=True,both=False)
            if n.navigation_state=='crossing':break
        self.assertEqual(n._active_turn,'right')
        self.assertTrue(n._junction_right_white_seen)
        for _ in range(10):
            wheels=self.step(error=None,both=False)
            if n._junction_right_turn_origin is not None:break
        self.assertAlmostEqual(wheels[0],.20)
        self.assertEqual(wheels[1],0)
        self.assertEqual(n.route_index,2)
        self.assertFalse(n._junction_left_red_rearmed)

    def test_left_red_rearm_rejects_departure_and_flickering_corridors(self):
        n=self.left_exit()
        for _ in range(5):self.step(red=True)
        self.assertFalse(n._junction_left_red_rearmed)
        for _ in range(4):
            n._junction_lane_geometry=self.near();self.step()
            n._junction_lane_geometry=dict(self.near(),valid=False);self.step()
        self.assertFalse(n._junction_left_red_rearmed)
        self.assertEqual(n.route_index,1)

    def test_left_next_red_stops_at_destination_without_departure(self):
        n=self.arm_next_red();n.route=['A','E','B'];n.auto_continue=True
        self.step(red=True)
        self.assertEqual(n.navigation_state,'route_complete')
        for _ in range(30):
            n.publish_wheels(*self.step(red=True)[:2])
            self.assertEqual((self.messages[-1].vel_left,self.messages[-1].vel_right),(0,0))
        self.assertIsNone(n._active_turn)

    def road_corner(self):
        self.configure_route(['A','E','B','C'])
        n=self.node;n.sharp_corner_enabled=True;n.base_speed=.09;n.max_speed=.20
        n.sharp_corner_confirm_seconds=.2;n.sharp_corner_recent_lane_seconds=1
        n.sharp_corner_approach_seconds=1;n.sharp_corner_turn_speed=.20
        n.sharp_corner_relief_inner_speed=0
        n._lane_both_visible=n._white_boundary_visible=n._yellow_boundary_visible=True
        self.assertIsNone(n.sharp_corner_wheels(0,False))
        n._lane_both_visible=n._white_boundary_visible=False
        self.now+=.01;self.assertIsNone(n.sharp_corner_wheels(.2,False))
        self.now+=.21
        self.assertEqual(n.sharp_corner_wheels(.2,False),(.09,.09,0))
        self.now+=1.01
        self.assertEqual(n.sharp_corner_wheels(.2,False),(.20,0,-.10))
        self.assertEqual(n.route_index,1)
        return n

    def test_road_corner_works_in_route_without_red_or_route_advance(self):
        n=self.road_corner()
        self.assertEqual(n.navigation_state,'following')
        self.assertEqual(n._sharp_corner_state,'turning')

    def test_road_corner_cannot_override_authorized_junction(self):
        self.configure_route(['A','E','B','C']);n=self.node;n.sharp_corner_enabled=True
        n._sharp_corner_last_both_time=self.now
        n._yellow_boundary_visible=True;n._white_boundary_visible=n._lane_both_visible=False
        for state in ('red_stop','crossing','reacquiring','route_complete'):
            n.navigation_state=state
            self.assertIsNone(n.sharp_corner_wheels(.3,False))
            self.assertEqual(n._sharp_corner_state,'idle')

    def test_road_corner_red_stop_respects_destination_C(self):
        n=self.road_corner();n.route_index=3;n.auto_continue=True
        self.assertEqual(n.sharp_corner_wheels(.2,True),(0,0,0))
        self.assertEqual(n.navigation_state,'route_complete')
        self.now+=3
        n.publish_wheels(*n.navigation_wheels(0,True)[:2])
        self.assertEqual((self.messages[-1].vel_left,self.messages[-1].vel_right),(0,0))


if __name__=='__main__':unittest.main()
