# Commands and navigation

The existing compressed camera input and stamped wheel output topics are unchanged.
Additional std_msgs/String topics:
- /duck2/lane_follower/command: JSON commands.
- /duck2/lane_follower/status: live JSON state, route, next junction, speed,
  camera age, published wheel speeds and latest command acknowledgment.

All normal commands need a unique string id and issued_at (Unix seconds).
The node accepts normal commands between two seconds in the future and five
seconds old. The desktop timestamps requests; the gateway supplies a timestamp only if absent.
Stop is accepted regardless of age. Recent ids are deduplicated (100 retained).
The command response reports accepted/rejected, not measured physical success.

Actions:
- stop: stop immediately. A stop during crossing faults the route.
- continue: resume a manual pause. Never enables a debug launcher, clears a fault,
  skips the stop dwell or bypasses missing calibration.
- slow_down / speed_up: multiply the speed scale by .8 / 1.2.
- speed_scale with numeric value: absolute scale within .25 to 1.5.
- turn with value left/right/straight: change the next junction exit and reconnect
  to the remaining route. Impossible exits and U-turns are rejected.
- heartbeat with client_id: keep the controlling laptop connection live; does not change the last user acknowledgment.
- set_route with route (junction list), position_confirmed: true: while stopped
  in route mode, confirm placement on the directed road `route[0] -> route[1]`,
  outside a junction and facing the second junction. Optional `map_id`,
  `start_approach` and `destination_approach` fields must match the shared map
  and route endpoints. This sets route progress and leaves a manual stop active
  until continue.

The desktop sends expected_control_epoch with motion requests so a delayed model
response cannot undo a later Stop. Turn requests additionally carry
expected_route_index and expected_next_junction. Both are checked on the node.

## Route states
awaiting_route -> following -> red_stop -> crossing -> reacquiring -> following.
A final-junction stop becomes route_complete. A camera failure, manual stop
inside a junction, unexpected second red line, or reacquisition timeout becomes
fault. A fault requires physical placement confirmation and a route reset.

The provisional outer loop is A-D-C-E-A. It has straight junction choices with
curves between junctions. A right override at D gives A-D-B-C-E-A.
Map details are in PROJECT_REQUIREMENTS.md.

The lane-route launcher enables route mode but leaves junctions_calibrated false.
It is not a fully calibrated road-running preset. After measuring the entry,
turn and reacquisition behavior with the actual bot, supply the calibrated
private ROS parameters and set junctions_calibrated true.
Default settings remain a two-second stop, .5-second entry, 1.6-second turn
(1 second for straight), .05 junction base speed and .025 wheel differential
bias. Direction-specific private parameters now inherit those values so left,
right and straight can be calibrated separately without changing this default.
Speed reductions scale both wheels and slow the traversal timing.
Reacquisition requires the original red line to have disappeared, both lane
boundaries in the expected order, and bounded lane error for .3 seconds.
None of this proves the correct physical exit without track testing.

The intersection interior is expected to be unmarked. After an authorized
red-line departure, missing yellow and white borders do not request zero:
`crossing` and `reacquiring` continue the selected bounded maneuver. Once both
outgoing right-lane borders are visible, visual centring takes over. The route
index changes only after the stable .3-second confirmation. The maneuver faults
at its deadline if the outgoing lane is never established. This exception does
not apply during ordinary `following`, where lane loss still stops output.

Status retains all existing fields and adds `junction_phase`, elapsed/progress
values, remaining deadline, direction-specific settings, the last junction
result, and red-line geometry. `red_line_visible` can be true before the
proximity trigger; `red_stop` becomes true only after the configured line-bottom
threshold is reached.

## HTTP gateway
command_gateway.py bridges GET /status and POST /command to those ROS topics.
It requires fresh node status and waits for a matching command acknowledgment.
A timeout reports an unknown outcome; the client does not retry movement blindly.
Default listener: 127.0.0.1:8765. Set DUCK2_CONTROL_TOKEN to a shared random value
and put the same value in the Windows client. A non-loopback listener requires
a token. Browser-origin requests are rejected.

To expose a Docker gateway to Windows, bind the host port to 127.0.0.1 only and
run the gateway with _listen_host:=0.0.0.0 inside that container. The gateway must
share ROS connectivity with the lane follower. Do not publish a robot-control
port to the public internet.

Camera loss stops the robot. The desktop sends a heartbeat every .5 seconds
while polling status. Once a client takes control, losing its heartbeat for
two seconds stops the robot and invalidates older motion requests. Reconnection
does not resume motion: a fresh explicit Continue is required. An interruption
inside a junction faults the route and requires placement confirmation.
The lane-route and lane-chat launchers require a fresh desktop heartbeat before
control. Unsupervised lane-follow mode can run without a desktop until a client
takes control. Stop remains available to any connected operator.

The combined lane-chat launcher starts the route node and HTTP gateway together.
It requires DUCK2_CONTROL_TOKEN and coordinates Ctrl+C shutdown of both processes.
It retains _junctions_calibrated:=false until physical calibration is supplied.
The current camera recorder is available separately as lane-record.

## Read-only camera service

`camera_gateway.py` subscribes to the existing compressed-camera topic and
serves `normal`, `mask` and `overlay` PPM images through `GET /camera?view=`.
It imports the lane follower's exact detector method, owns no ROS publisher and
has no command endpoint. `GET /health` returns frame age and diagnostic text.
The service defaults to loopback port 8766; the Windows app accepts only a
loopback URL intended for an SSH tunnel. `lane-camera-view` starts this service
alone and does not start a controller.

Obstacle candidates trigger a persistent stop. The view must remain clear for
.5 seconds before explicit Continue can release it. Detection is a provisional
bright/colored-region heuristic, not recognition of every possible obstacle.
