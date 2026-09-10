"""Live chat and preview integration. Run ONLY in a Docker --network none container."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import xmlrpc.client


def main():
    # Fail before constructing ROS clients if any non-loopback interface exists.
    if {p.name for p in Path('/sys/class/net').iterdir()} != {'lo'}:
        raise RuntimeError('This synthetic check requires Docker --network none')
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11311',
                      ROS_HOSTNAME='localhost', VEHICLE_NAME='duck2')
    import cv2
    import numpy as np
    import rospy
    from sensor_msgs.msg import CompressedImage
    from duckietown_msgs.msg import WheelsCmdStamped
    from chat_core import RobotTransport
    from companion_core import OfflineCompanionSession
    from live_chat import LiveChatSession, interpret_live
    from route_control import start_route
    from camera_client import CameraClient

    root = Path(__file__).resolve().parents[1]
    processes = []
    finished = threading.Event()
    log = open('/tmp/live-pause-ros.log', 'w')
    wheels = []
    scene = [None]
    transport = None

    def wait_for(predicate, seconds=8):
        deadline = time.monotonic() + seconds
        error = None
        while time.monotonic() < deadline:
            try:
                value = predicate()
                if value:
                    return value
            except (RuntimeError, OSError) as exc:
                error = exc
            time.sleep(.03)
        raise AssertionError('Condition timed out: %s' % error)

    def spawn(args):
        child = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
        processes.append(child)
        return child

    try:
        spawn(['roscore'])
        wait_for(lambda: xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getUri('/pause_check')[0] == 1)
        rospy.init_node('isolated_live_pause_check', disable_signals=True)
        camera = rospy.Publisher('/duck2/camera_node/image/compressed', CompressedImage, queue_size=1)
        rospy.Subscriber('/duck2/wheels_driver_node/wheels_cmd', WheelsCmdStamped,
                         lambda m: wheels.append((time.monotonic(), m.vel_left, m.vel_right)), queue_size=100)
        image = np.zeros((480, 640, 3), np.uint8)
        cv2.rectangle(image, (160, 240), (180, 475), (0, 255, 255), -1)
        cv2.rectangle(image, (470, 240), (490, 475), (255, 255, 255), -1)
        scene[0] = image

        def stream():
            while not finished.wait(.05):
                ok, data = cv2.imencode('.jpg', scene[0])
                if ok:
                    msg = CompressedImage()
                    msg.header.stamp = rospy.Time.now()
                    msg.format, msg.data = 'jpeg', data.tobytes()
                    camera.publish(msg)

        threading.Thread(target=stream, daemon=True).start()
        source = root / 'packages/duckie_lane_follower/src'
        spawn([sys.executable, str(source / 'lane_follower_node.py'),
               '_drive_enabled:=true', '_show_debug:=false', '_route_enabled:=true',
               '_junctions_calibrated:=true', '_require_client_heartbeat:=true',
               '_junction_straight_visual_approach:=true',
               '_junction_straight_lateral_gain:=0.25', '_junction_straight_heading_gain:=0.30',
               '_junction_straight_lane_target_fraction:=0.49',
               '_base_speed:=0.09', '_max_speed:=0.20'])
        spawn([sys.executable, str(source / 'command_gateway.py'), '_listen_host:=127.0.0.1'])
        spawn([sys.executable, str(source / 'camera_gateway.py'), '_listen_host:=127.0.0.1'])
        transport = RobotTransport()
        wait_for(transport.status)
        preview = CameraClient()
        for view in preview.VIEWS:
            frame = wait_for(lambda: preview.frame(view))
            assert frame.ppm.startswith(b'P6\n') and frame.age < 1.5
        print('PASS HTTP/ROS: normal, mask and overlay from real preview gateway', flush=True)

        def heartbeat():
            while not finished.wait(.25):
                try:
                    transport.heartbeat()
                except RuntimeError:
                    pass

        threading.Thread(target=heartbeat, daemon=True).start()
        plan = OfflineCompanionSession()
        plan.select_start('A->E')
        plan.select_destination('A->E')
        live = LiveChatSession('A->E', stop_at_next_red=True)
        start_route(transport, plan.plan, live_session=live)
        wait_for(lambda: transport.status()['live_session']['straight'])
        for profile in ('slow', 'normal', 'fast', 'normal'):
            live.execute(interpret_live(profile + ' speed'), transport, transport.poll_status())
            assert transport.status()['live_session']['profile'] == profile
        print('PASS HTTP/ROS: local chatbot selects all three straight profiles', flush=True)

        # Valid markings but no confirmed straight: pause must still be immediate.
        slanted = np.zeros_like(image)
        cv2.line(slanted, (260, 240), (160, 475), (0, 255, 255), 20)
        cv2.line(slanted, (570, 240), (470, 475), (255, 255, 255), 20)
        scene[0] = slanted
        wait_for(lambda: not transport.status()['live_session']['straight'])
        sent = time.monotonic()
        live.execute(interpret_live('stop for 7s'), transport, transport.poll_status())
        value = transport.status()
        assert value['live_session']['paused'] and value['wheel_speeds'] == [0., 0.], value
        wait_for(lambda: any(t >= sent and left == right == 0 for t, left, right in wheels))
        zero_at = next(t for t, left, right in wheels if t >= sent and left == right == 0)
        scene[0] = image
        time.sleep(6.6)
        assert transport.status()['live_session']['paused']
        assert all(left == right == 0 for t, left, right in wheels if t >= zero_at)
        wait_for(lambda: any(t > zero_at and max(left, right) > .01 for t, left, right in wheels), 3)
        resumed_at = next(t for t, left, right in wheels if t > zero_at and max(left, right) > .01)
        assert 6.9 <= resumed_at - zero_at <= 7.6, resumed_at - zero_at
        print('PASS HTTP/ROS: immediate zero; 7-second pause; healthy resume (%.3fs)' % (resumed_at-zero_at), flush=True)

        red = image.copy()
        cv2.rectangle(red, (260, 400), (580, 420), (0, 0, 255), -1)
        scene[0] = red
        wait_for(lambda: transport.status()['state'] == 'route_complete')
        value = transport.status()
        assert not value['live_session']['active']
        assert value['wheel_speeds'] == [0., 0.]
        assert value['live_session']['end_reason'] == 'Pause check complete at red line'
        assert not transport.send('resume', run_id=live.run_id,
                                  expected_control_epoch=value['control_epoch'])['accepted']
        time.sleep(.4)
        assert transport.status()['wheel_speeds'] == [0., 0.]
        print('PASS HTTP/ROS: next red line ends the check; no crossing or late resume', flush=True)

        # New session uses a directed final red line, independently of the
        # laptop staying alive to send a finishing Stop command.
        scene[0] = image
        wait_for(lambda: not transport.status().get('red_line_detection'))
        plan.select_start('C->B')
        plan.select_destination('C->B')
        live = LiveChatSession('C->B', finish_approach='C->B')
        value = start_route(transport, plan.plan, live_session=live)
        assert value['live_session']['finish_approach'] == 'C->B'
        wait_for(lambda: max(transport.status()['wheel_speeds']) > .01)
        scene[0] = red
        wait_for(lambda: transport.status()['state'] == 'route_complete')
        value = transport.status()
        assert not value['live_session']['active']
        assert value['wheel_speeds'] == [0., 0.]
        assert value['live_session']['end_reason'] == 'Destination reached at C->B red line'
        assert not transport.send('resume', run_id=live.run_id,
                                  expected_control_epoch=value['control_epoch'])['accepted']
        print('PASS HTTP/ROS: configured final red line ends run on robot; late resume rejected', flush=True)

        scene[0] = image
        wait_for(lambda: not transport.status().get('red_line_detection'))
        plan.select_start('A->E')
        plan.select_destination('E->C')
        live = LiveChatSession('A->E', ['straight'], stop_after_junction=True)
        start_route(transport, plan.plan, live_session=live)
        wait_for(lambda: max(transport.status()['wheel_speeds']) > .01)
        scene[0] = red
        wait_for(lambda: transport.status()['state'] == 'red_stop')
        wait_for(lambda: transport.status()['junction_instruction_ready'])
        live.service(transport, transport.poll_status())
        wait_for(lambda: transport.status()['state'] == 'crossing')
        scene[0] = np.zeros_like(image)
        time.sleep(.8)
        value = transport.status()
        assert value['live_session']['active'] and value['route_index'] == 1
        scene[0] = image
        # No further chat queue polling: completion is an onboard action.
        wait_for(lambda: transport.status()['state'] == 'route_complete', 15)
        value = transport.status()
        assert value['route_index'] == 2 and value['current_approach'] == 'E->C'
        assert value['manual_stop'] and value['wheel_speeds'] == [0., 0.]
        assert not value['live_session']['active']
        assert value['junction_last_result']['outcome'] == 'reacquired'
        assert not transport.send('continue')['accepted']
        print('PASS HTTP/ROS: one straight crossing ends at confirmed outgoing lane; no late Continue', flush=True)

        plan.select_start('A->E')
        plan.select_destination('E->B')
        live = LiveChatSession('A->E', plan.plan.turns, center_initial_straight=True,
                               finish_after_junction_red=True)
        start_route(transport, plan.plan, live_session=live)
        wait_for(lambda: transport.status()['junction_approach_active'])
        assert live.queue == ['left']
        live.execute(interpret_live('go straight at the next junction'), transport, transport.poll_status())
        scene[0] = red
        wait_for(lambda: transport.status()['junction_instruction_ready'])
        live.service(transport, transport.poll_status())
        wait_for(lambda: transport.status()['state'] == 'crossing')
        scene[0] = np.zeros_like(image)
        time.sleep(.8)
        scene[0] = image
        wait_for(lambda: transport.status()['route_index'] == 2, 15)
        value = transport.status()
        assert value['state'] == 'following' and value['live_session']['active']
        assert not value['junction_approach_active']
        scene[0] = red
        wait_for(lambda: transport.status()['state'] == 'route_complete')
        value = transport.status()
        assert value['wheel_speeds'] == [0., 0.] and not value['live_session']['active']
        assert value['live_session']['end_reason'] == 'Destination reached at E->C red line'
        print('PASS HTTP/ROS: initial straight centering, user left-to-straight override, road handoff and C red stop', flush=True)

        # Ordinary companion mode: no scenario finish flags or early-straight
        # assumption. A map queue can be overridden, then extended at a red stop.
        scene[0] = image
        wait_for(lambda: not transport.status().get('red_line_detection'))
        plan.select_start('A->E')
        plan.select_destination('B->C')
        live = LiveChatSession(plan.plan.start_approach, plan.plan.turns)
        value = start_route(transport, plan.plan, live_session=live)
        live.observe(value)
        assert live.queue == ['left', 'right']
        assert not value['live_session']['center_initial_straight']
        live.execute(interpret_live('go straight at the next junction'), transport, transport.poll_status())
        scene[0] = red
        wait_for(lambda: transport.status()['junction_instruction_ready'])
        live.service(transport, transport.poll_status())
        wait_for(lambda: transport.status()['state'] == 'crossing')
        scene[0] = np.zeros_like(image)
        time.sleep(.8)
        scene[0] = image
        wait_for(lambda: transport.status()['route_index'] == 2, 15)
        live.observe(transport.status())
        assert live.approach == 'E->C' and live.active and not live.queue
        scene[0] = red
        wait_for(lambda: transport.status()['state'] == 'red_stop')
        value = transport.status()
        live.observe(value)
        assert any('Where should I go next?' in notice for notice in live.take_notices())
        assert value['wheel_speeds'] == [0., 0.] and value['live_session']['red_wait_seconds'] == 60
        live.execute(interpret_live('go left'), transport, transport.poll_status())
        wait_for(lambda: transport.status()['junction_instruction_ready'])
        live.service(transport, transport.poll_status())
        wait_for(lambda: transport.status()['state'] == 'crossing')
        assert transport.status()['active_turn'] == 'left'
        transport.send('stop')
        value = wait_for(lambda: (lambda s: s if s['manual_stop'] else None)(transport.status()))
        live.observe(value)
        assert not live.active and value['wheel_speeds'] == [0., 0.]
        assert not transport.send('resume', run_id=live.run_id,
                                  expected_control_epoch=value['control_epoch'])['accepted']
        print('PASS HTTP/ROS: normal map queue, straight override, C prompt, left dispatch and terminal Stop', flush=True)

    finally:
        if sys.exc_info()[0] is not None:
            log.flush()
            print(Path('/tmp/live-pause-ros.log').read_text()[-6000:], flush=True)
        if transport is not None:
            try:
                transport.send('stop')
            except Exception:
                pass
        finished.set()
        for child in reversed(processes):
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
        log.close()


if __name__ == '__main__':
    main()
