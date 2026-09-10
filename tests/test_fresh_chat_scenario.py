"""Exact app/chat/controller sequence with a fake ROS clock; no robot network."""
import unittest
from unittest.mock import patch
import socket
import cv2
import numpy as np
import test_live_navigation as fixtures
from live_chat import LiveChatSession, interpret_live
from route_planner import plan_route
from route_control import start_route
import test_live_chat as queue_fixtures


class FreshChatScenarioTests(unittest.TestCase):
    setUp = fixtures.LiveNavigationTests.setUp
    subscribe = fixtures.LiveNavigationTests.subscribe
    message = fixtures.LiveNavigationTests.message
    deliver = fixtures.LiveNavigationTests.deliver
    image = fixtures.LiveNavigationTests.image
    speeds = fixtures.LiveNavigationTests.speeds
    command = fixtures.LiveNavigationTests.command
    managed = fixtures.LiveNavigationTests.managed

    def test_single_straight_crossing_pauses_and_stops_on_robot_without_laptop_polling(self):
        with patch.object(socket, 'socket', side_effect=AssertionError('Robot networking forbidden')):
            n = self.managed(['A', 'E'])
            self.command('stop')
            ack = self.command('set_route', route=['A', 'E'], position_confirmed=True,
                               managed_session=True, stop_after_junction=True)
            self.assertTrue(ack['accepted'], ack)
            self.assertTrue(self.command('continue')['accepted'])
            laptop = LiveChatSession('A->E', ['straight'], run_id='test', stop_after_junction=True)
            case = self
            class Link:
                def send(self, action, command_id=None, **kwargs):
                    if command_id is not None:
                        kwargs['id'] = command_id
                    return case.command(action, **kwargs)
            link = Link()
            self.assertGreater(sum(self.tick()[:2]), 0)
            laptop.execute(interpret_live('stop for 7s'), link, n.status())
            paused = self.now
            for _ in range(69):
                self.assertEqual(self.tick(), (0., 0., 0.))
            self.now = paused + 7.
            self.assertGreater(sum(self.tick()[:2]), 0)
            self.tick(red=True)
            self.tick(red=True, seconds=2.1)
            self.assertTrue(n.live.active)
            rejected = self.command('junction_instruction', value='left', approach='A->E',
                                    expected_route_index=1)
            self.assertFalse(rejected['accepted'])
            laptop.service(link, n.status())
            self.assertEqual(n._active_turn, 'straight')
            # No laptop service calls after departure: the robot owns completion.
            for _ in range(20):
                self.tick(error=None)
                self.assertTrue(n.live.active)
                self.assertEqual(n.route_index, 1)
            for _ in range(100):
                self.tick()
                if not n.live.active:
                    break
            self.assertFalse(n.live.active)
            self.assertEqual(n.route_index, 2)
            self.assertEqual(n.navigation_state, 'route_complete')
            self.assertEqual(self.speeds(), (0., 0.))
            self.assertTrue(n.manual_stop)
            for _ in range(10):
                self.assertEqual(self.tick(), (0., 0., 0.))
            self.assertFalse(self.command('continue')['accepted'])
            self.assertFalse(self.command('resume')['accepted'])
            laptop.service(link, n.status())
            self.assertFalse(laptop.active)
            self.assertEqual(laptop.approach, 'E->C')

    def test_single_crossing_capability_and_queue_are_enforced(self):
        plan = plan_route('A->E', 'E->C')
        live = LiveChatSession('A->E', ['straight'], stop_after_junction=True)
        for turns in (['left'], ['right'], ['straight', 'left']):
            with self.assertRaises(ValueError):
                live.replace_turns(turns)
        value = queue_fixtures.status(live)
        link = queue_fixtures.Transport(value)
        with self.assertRaisesRegex(RuntimeError, 'single-crossing'):
            start_route(link, plan, live_session=live)
        self.assertEqual(link.calls, [])
        value['live_session']['supports_stop_after_junction'] = True
        link = queue_fixtures.Transport(value)
        start_route(link, plan, live_session=live)
        self.assertTrue(link.calls[0][1]['stop_after_junction'])

    def test_single_crossing_options_validate_and_fresh_start_resets(self):
        n = self.managed(['A', 'E'])
        self.command('stop')
        for options in ({'stop_after_junction': 'true'},
                        {'stop_after_junction': True, 'stop_at_next_red': True},
                        {'stop_after_junction': True, 'finish_approach': 'C->B'}):
            self.assertFalse(self.command('set_route', route=['A', 'E'],
                position_confirmed=True, managed_session=True, **options)['accepted'])
        self.assertTrue(self.command('set_route', route=['A', 'E'],
            position_confirmed=True, managed_session=True, stop_after_junction=True)['accepted'])
        self.assertTrue(n.live.stop_after_junction)
        self.assertTrue(self.command('set_route', route=['A', 'E'],
            position_confirmed=True, managed_session=True)['accepted'])
        self.assertFalse(n.live.stop_after_junction)

    def tick(self, error=0., red=False, seconds=.1):
        self.now += seconds
        n = self.node
        n._last_frame_time = self.now
        n._last_lane_error = error
        n._lane_both_visible = error is not None
        wheels = n.navigation_wheels(error, red)
        n.publish_wheels(*wheels[:2])
        return wheels

    def test_exact_pause_override_curve_wait_left_and_final_red(self):
        with patch.object(socket, 'socket', side_effect=AssertionError('Robot networking forbidden')):
            n = self.managed(['A', 'E'])
            n.live.finish_approach = 'C->B'
            n.base_speed = .09
            n.max_speed = .20
            n.junction_straight_visual_approach = True
            n.junction_straight_lane_target_fraction = .49
            n.junction_straight_lateral_gain = .25
            n.junction_straight_heading_gain = .30
            n.junction_reacquire_timeout = 9.
            n.junction_left_visual_latch = True
            plan = plan_route('A->E', 'B->C')
            laptop = LiveChatSession('A->E', plan.turns, run_id='test', finish_approach='C->B')
            case = self
            class Link:
                calls = []
                def send(self, action, command_id=None, **kwargs):
                    self.calls.append(action)
                    if command_id is not None:
                        kwargs['id'] = command_id
                    return case.command(action, **kwargs)
            link = Link()
            # User's pause is immediate; a queued straight replaces both old turns.
            self.tick()
            laptop.execute(interpret_live('stop for 7s'), link, n.status())
            paused = self.now
            self.assertEqual(self.speeds(), (0., 0.))
            laptop.execute(interpret_live('go straight at the next junction'), link, n.status())
            self.assertEqual(laptop.queue, ['straight'])
            for _ in range(69):
                self.assertEqual(self.tick(), (0., 0., 0.))
            self.now = paused + 7.
            n._last_frame_time = self.now
            n.live_tick()
            self.assertIsNone(n.live.paused_at)
            self.assertEqual(n.live.profile, 'normal')
            self.assertGreater(sum(self.tick()[:2]), 0)
            self.tick(red=True)
            self.tick(red=True, seconds=2.1)
            laptop.service(link, n.status())
            self.assertEqual(n.route, ['A', 'E', 'C'])
            self.assertEqual(n._active_turn, 'straight')
            # Missing markings during the crossing do not stop its profile.
            while n.navigation_state == 'crossing':
                self.assertGreater(sum(self.tick(error=None)[:2]), 0)
            # Real OpenCV row masks: fixed bands miss a dash; no perfectly
            # straight heading or additional near-alignment gate is required.
            yellow = np.zeros((216, 640), np.uint8)
            white = np.zeros_like(yellow)
            for y in range(10, 128):
                x = int(155 + 55*(y/216.)**2)
                if not 68 <= y < 81:
                    cv2.line(yellow, (x, y), (x+9, y), 255)
                cv2.line(white, (x+310, y), (x+319, y), 255)
            original = n.junction_lane_geometry(yellow, white, 640)
            n._junction_exit_geometry = n.straight_exit_geometry(yellow, white, 640, original)
            n._junction_lane_geometry = original
            self.assertFalse(n._junction_exit_geometry['near_support'])
            for _ in range(6):
                self.tick(error=-.15)
                laptop.service(link, n.status())
            self.assertEqual(n.navigation_state, 'following')
            self.assertEqual(laptop.approach, 'E->C')
            self.assertEqual(laptop.queue, [])
            self.assertIsNone(n._active_turn)
            self.assertIsNone(n._reacquire_started)
            self.assertIsNone(n._junction_deadline_at)
            # The left curve belongs to the road controller beyond the old timer.
            for _ in range(110):
                wheels = self.tick(error=-.20)
                self.assertGreater(wheels[1], wheels[0])
            self.assertEqual(n.route_index, 2)
            self.assertEqual(n.navigation_state, 'following')
            # C waits for the user, without playing the old B right instruction.
            self.tick(red=True)
            arrival = self.now
            for elapsed in (2.1, 30., 59.):
                self.tick(red=True, seconds=arrival+elapsed-self.now)
                laptop.service(link, n.status())
                self.assertTrue(n.live.active)
                self.assertEqual(self.speeds(), (0., 0.))
            self.assertTrue(any('60 seconds' in x for x in laptop.take_notices()))
            laptop.execute(interpret_live('left'), link, n.status())
            laptop.service(link, n.status())
            self.assertEqual(n._active_turn, 'left')
            self.assertEqual(n.route, ['A', 'E', 'C', 'B'])
            # Reuse the existing left-junction entry profile and near exit gate.
            n._junction_lane_geometry = dict(valid=True, steering_valid=True,
                near_support=True, lateral_error=0., heading_error=0.)
            for _ in range(100):
                self.tick()
                laptop.service(link, n.status())
                if n.navigation_state == 'following':
                    break
            self.assertEqual(laptop.approach, 'C->B')
            self.assertEqual(n.route_index, 3)
            self.tick(red=True)
            self.tick(red=True)
            laptop.service(link, n.status())
            self.assertEqual(n.navigation_state, 'route_complete')
            self.assertFalse(n.live.active)
            self.assertFalse(laptop.active)
            self.assertEqual(self.speeds(), (0., 0.))
            self.assertIn('C->B', n.live.end_reason)
            self.assertEqual(link.calls.count('junction_instruction'), 2)
            self.assertFalse(self.command('resume')['accepted'])

    def test_new_start_clears_old_crossing_and_pause_state(self):
        n = self.managed(['A', 'E'])
        self.command('pause', seconds=7)
        self.command('stop')
        n._stop_started = n._reacquire_started = self.now-100
        n._crossing_started = self.now-110
        n._junction_straight_visual_entry = True
        n._lane_good_since = self.now-9
        n._junction_encoder_adjustment = .012
        result = self.command('set_route', route=['A', 'E'], managed_session=True,
                              run_id='fresh', position_confirmed=True, finish_approach='C->B')
        self.assertTrue(result['accepted'], result)
        self.assertEqual(n.live.run_id, 'fresh')
        self.assertIsNone(n.live.paused_at)
        self.assertIsNone(n.live.instruction_id)
        self.assertIsNone(n._reacquire_started)
        self.assertIsNone(n._crossing_started)
        self.assertIsNone(n._lane_good_since)
        self.assertFalse(n._junction_straight_visual_entry)
        self.assertEqual(n._junction_encoder_adjustment, 0.)
        self.assertTrue(n.manual_stop)
        self.assertEqual(self.speeds(), (0., 0.))
        self.assertFalse(self.command('resume', run_id='test')['accepted'])

    def test_final_target_sent_at_start_and_old_controller_rejected(self):
        plan = plan_route('A->E', 'B->C')
        live = LiveChatSession('A->E', plan.turns, finish_approach='C->B')
        value = queue_fixtures.status(live)
        value['live_session']['supports_finish_approach'] = True
        link = queue_fixtures.Transport(value)
        start_route(link, plan, live_session=live)
        self.assertEqual(link.calls[0][1]['finish_approach'], 'C->B')
        value['live_session']['supports_finish_approach'] = False
        link = queue_fixtures.Transport(value)
        with self.assertRaisesRegex(RuntimeError, 'final-red-line'):
            start_route(link, plan, live_session=live)
        self.assertEqual(link.calls, [])


if __name__ == '__main__':
    unittest.main()
