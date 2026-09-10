"""Initial straight centering and one-junction red-stop scenario, mocked ROS."""
import unittest
import cv2
import numpy as np
import test_fresh_chat_scenario as fixtures
from live_chat import LiveChatSession, interpret_live
from route_planner import plan_route
from route_control import start_route
import test_live_chat as queues


class InitialStraightCheckTests(unittest.TestCase):
    setUp = fixtures.FreshChatScenarioTests.setUp
    subscribe = fixtures.FreshChatScenarioTests.subscribe
    message = fixtures.FreshChatScenarioTests.message
    deliver = fixtures.FreshChatScenarioTests.deliver
    image = fixtures.FreshChatScenarioTests.image
    speeds = fixtures.FreshChatScenarioTests.speeds
    command = fixtures.FreshChatScenarioTests.command
    managed = fixtures.FreshChatScenarioTests.managed
    tick = fixtures.FreshChatScenarioTests.tick

    def configure(self):
        n = self.managed(['A', 'E'])
        self.command('stop')
        ack = self.command('set_route', route=['A', 'E'], managed_session=True,
                           position_confirmed=True, center_initial_straight=True,
                           finish_after_junction_red=True)
        self.assertTrue(ack['accepted'], ack)
        n.junction_straight_visual_approach = True
        n.junction_straight_lane_target_fraction = .49
        n.junction_straight_lateral_gain = .25
        n.junction_straight_heading_gain = .30
        n.junction_straight_approach_max_steering = .03
        n.base_speed, n.max_speed, n.alpha = .09, .20, .20
        n.steering_bias = .0075
        n._red_line_visible = False
        n._junction_lane_geometry = self.geometry()
        self.assertTrue(self.command('continue')['accepted'])
        return n

    @staticmethod
    def geometry(lateral=0., heading=0.):
        return dict(valid=True, steering_valid=True, near_support=True,
                    lateral_error=lateral, heading_error=heading)

    def test_centered_straight_ignores_centroid_jitter_and_trim_before_red(self):
        n = self.configure()
        for error in [.20, -.20] * 15:
            left, right, steering = self.tick(error=error)
            self.assertAlmostEqual(left, right)
            self.assertAlmostEqual(steering, 0.)
        self.assertTrue(n._junction_approach_active)
        self.assertFalse(n._red_line_visible)

    def test_lateral_and_heading_corrections_have_right_sign_and_decay(self):
        n = self.configure()
        for lateral, heading, direction in ((.08, 0., -1), (-.08, 0., 1),
                                             (0., .08, 1), (0., -.08, -1)):
            n._junction_approach_steering = 0.
            n._junction_lane_geometry = self.geometry(lateral, heading)
            for _ in range(12):
                wheels = self.tick()
            self.assertGreater(wheels[2] * direction, 0)
            self.assertLessEqual(abs(wheels[2]), .03)
        n._junction_lane_geometry = self.geometry()
        for _ in range(20):
            wheels = self.tick()
        self.assertLess(abs(wheels[2]), .0001)

    def test_row_noise_is_smoothed_and_missing_lane_stays_stopped(self):
        n = self.configure()
        output = []
        for lateral in [.04, -.04] * 25:
            n._junction_lane_geometry = self.geometry(lateral)
            output.append(self.tick(seconds=1/30)[2])
        self.assertLess(max(abs(a-b) for a,b in zip(output, output[1:])), .004)
        self.assertLess(max(abs(s) for s in output), .01)
        n._junction_lane_geometry = None
        self.assertEqual(self.tick(error=None), (0., 0., 0.))

    def test_dense_rows_keep_dashes_between_fixed_bands_usable(self):
        n = self.configure()
        image = np.zeros((480,640,3), dtype=np.uint8)
        cv2.rectangle(image, (470,265), (488,479), (255,255,255), -1)
        for a,b in ((.06,.12),(.25,.29),(.43,.47),(.61,.65),(.81,.84),(.92,.96)):
            cv2.rectangle(image, (140,240+int(a*216)), (158,240+int(b*216)), (0,255,255), -1)
        n.detect_lane_bgr(image)
        self.assertTrue(n.junction_geometry_steering_valid(n._junction_lane_geometry))
        self.assertGreaterEqual(n._junction_lane_geometry['pair_count'], 6)

    def test_after_first_junction_returns_exact_existing_approach_behavior(self):
        n = self.configure()
        n.route = ['A','E','C']
        n.route_index = 2
        n._junction_approach_active = False
        before = (.03, .20, .085)
        self.assertFalse(n.initial_straight_approach_active())
        self.assertEqual(n.straight_approach_wheels(*before), before)
        n.live.center_initial_straight = False
        self.assertEqual(n.straight_approach_wheels(*before), before)

    def test_override_completes_crossing_then_ends_at_c_red_not_lane_entry(self):
        n = self.configure()
        plan = plan_route('A->E', 'E->B')
        self.assertEqual(plan.turns, ('left',))
        live = LiveChatSession('A->E', plan.turns, run_id='test',
                               finish_after_junction_red=True, center_initial_straight=True)
        case = self
        class Link:
            def send(self, action, command_id=None, **kwargs):
                if command_id is not None:
                    kwargs['id'] = command_id
                return case.command(action, **kwargs)
        link = Link()
        live.execute(interpret_live('stop for 7s'), link, n.status())
        self.assertEqual(self.speeds(), (0.,0.))
        live.execute(interpret_live('go straight at the next junction'), link, n.status())
        self.assertEqual(live.queue, ['straight'])
        self.tick(seconds=7.1)
        self.tick(red=True)
        self.tick(red=True, seconds=2.1)
        live.service(link, n.status())
        self.assertEqual(n._active_turn, 'straight')
        for _ in range(100):
            self.tick()
            live.service(link, n.status())
            if n.route_index == 2:
                break
        self.assertEqual(n.route_index, 2)
        self.assertTrue(n.live.active)
        self.assertFalse(n.initial_straight_approach_active())
        self.assertEqual(live.approach, 'E->C')
        self.tick(red=True)
        self.tick(red=True)
        self.assertEqual(n.navigation_state, 'route_complete')
        self.assertEqual(self.speeds(), (0.,0.))
        self.assertIn('E->C', n.live.end_reason)
        self.assertFalse(self.command('continue')['accepted'])

    def test_normal_left_choice_ends_at_b_red_and_new_start_clears_options(self):
        from duckie_lane_follower.live_session import LiveSession
        live = LiveSession()
        live.start('start', finish_after_junction_red=True, center_initial_straight=True)
        live.tick(1., 'red_stop', 1, 0., True, 'A->E')
        self.assertTrue(live.active)
        live.tick(2., 'following', 2, None, True, 'E->B')
        self.assertTrue(live.active)
        live.tick(3., 'red_stop', 2, 3., True, 'E->B')
        self.assertFalse(live.active)
        self.assertIn('E->B', live.end_reason)
        live.start('new')
        self.assertFalse(live.center_initial_straight)
        self.assertFalse(live.finish_after_junction_red)
        for options in ({'center_initial_straight': 1},
                        {'finish_after_junction_red': True, 'stop_after_junction': True}):
            with self.assertRaises(ValueError):
                live.start('invalid', **options)

    def test_old_controller_rejected_before_start_or_route_commands(self):
        plan = plan_route('A->E', 'E->B')
        live = LiveChatSession('A->E', plan.turns, center_initial_straight=True,
                               finish_after_junction_red=True)
        value = queues.status(live)
        link = queues.Transport(value)
        with self.assertRaisesRegex(RuntimeError, 'initial-straight'):
            start_route(link, plan, live_session=live)
        self.assertEqual(link.calls, [])
        value['live_session']['supports_initial_straight_check'] = True
        link = queues.Transport(value)
        start_route(link, plan, live_session=live)
        self.assertTrue(link.calls[0][1]['center_initial_straight'])
        self.assertTrue(link.calls[0][1]['finish_after_junction_red'])


if __name__ == '__main__':
    unittest.main()
