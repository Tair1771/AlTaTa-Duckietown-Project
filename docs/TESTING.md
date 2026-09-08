# Testing status

## Offline results — 2026-09-08

The project was rebuilt against the same Duckietown ROS base release observed
on duck2: `dt-ros-commons:v4.3.0`, ROS Noetic, Python 3.8, OpenCV 4.2 and
NumPy 1.17. The package built successfully for both local AMD64 and matching
ARM64 images. The base image references are digest-pinned in
`config/duck2.json`.

The offline regression suite ran 132 tests: 131 passed and one platform-specific
test was skipped. The suites use synthetic images, a local isolated ROS master, mock ROS
interfaces or local Tk windows. They do not contact duck2.

The ROS transport test also checks these safety behaviours using synthetic
compressed images:

- camera-debug produces only zero wheel messages;
- missing, stale, future, repeated and out-of-order stamps stop output;
- a fresh camera frame restores normal output;
- Ctrl+C delivers a final zero wheel message;
- lane loss and configured red stops inhibit movement.

The Noetic transport check observes a short recovery window after an invalid
timestamp. It now requires that a positive output occurred after fresh frames
returned, rather than treating the final sample as the result: the watchdog may
legitimately publish a later zero in that window.

## Re-run the local checks

First build the local AMD64 image as described in
[ROBOT_SETUP.md](ROBOT_SETUP.md). From a shell with Docker access, run:

```bash
docker run --rm --network none -v "$PWD:/project:ro" \
  --entrypoint bash altata-duck2:noetic-amd64 \
  -lc 'source /environment.sh >/dev/null 2>&1; cd /project/tests && python3 -m unittest \
  test_runtime_setup test_lane_follower test_navigation \
  test_obstacles_and_connection test_first_test_readiness test_bounded_ground_supervisor test_duck_avoidance \
  test_chat test_offline_interpreter test_command_preview test_obstacle_language'

docker run --rm --network none -v "$PWD:/project:ro" \
  --entrypoint bash altata-duck2:noetic-amd64 \
  -lc 'source /environment.sh >/dev/null 2>&1; python3 /project/tests/test_ros_transport.py'
```

Both commands use Docker network isolation. The second command starts an
isolated local ROS master; it does not use the robot’s ROS master. Rebuild after
changing package source, launchers or the Dockerfile.

The offline chatbot tests can also be run on Windows:

```powershell
py -3 -X utf8 tests/test_offline_interpreter.py
py -3 -X utf8 tests/test_offline_chat_ui.py
py -3 -X utf8 tests/test_command_preview.py
py -3 -X utf8 tests/test_chat.py
```

## Live compatibility and remaining physical checks

duck2's live camera has been passively verified: twelve compressed frames
decoded successfully at 640x480, timestamps advanced, and the latest frame was
well inside the node's 0.5-second freshness limit. The exact lane-follower
source processed those frames with driving disabled and a diagnostic wheel
topic; its logs reported only zero values and a clean shutdown.

The project controller has now completed one bounded lifted-wheel check, as
recorded below. Wheel direction, ground-contact response, braking distance,
junction timing, obstacle passing and route behaviour remain provisional.

## Upside-down bench results — 2026-09-08

duck2 was secured upside down before the installed wheel-driver diagnostics
were used. Windows SSH access through the hotspot worked with the robot's
previously verified SSH host key. The ROS master, camera, IMU and front range
sensor were available. Camera header spacing was approximately 30 Hz. That
spacing confirms stream cadence, not the age of a frame; the follower's
freshness timeout still requires a live track-session check.

Before testing, a five-second passive check found no messages on
`/duck2/wheels_driver_node/wheels_cmd_executed`. The left and right encoder
topics remained respectively at `0` and `1`, so no motion was being reported.
`/duck2/kinematics_node` remained the sole publisher of the normal
`wheels_cmd` topic.

The installed `std_srvs/Trigger` left- and right-wheel test services were
each called once. Both returned success and reported their own bounded
parameters: a normalized velocity of `0.5` for `3 s`. The left encoder then
settled at `844`, and the right encoder settled at `849`; the opposite
encoder stayed unchanged during each corresponding test. After a further
five-second passive check, both values were stable and no executed-wheel
messages appeared.

The driver did not publish `wheels_cmd_executed` feedback during these
built-in tests. The user observed each wheel spinning individually for about
three seconds and believes its apparent upright direction was forward. Treat
direction as tentative until it is confirmed on a later controlled ground test.
The service result and encoder change establish driver/encoder interface
evidence but do not measure ground speed or stopping distance.

The project lane-follower source was also run temporarily on duck2 with
`drive_enabled:=false`, display disabled and its output remapped to
`/duck2/lane_follower/diagnostic_wheels_cmd`. It processed live desk-view
frames, published five observed `0.0/0.0` diagnostic messages, and exited
cleanly. Its temporary source and process were removed. It did not publish to
the real wheel topic.

A later project-controller check was performed with duck2 securely upside
down and both wheels clear. The camera was covered, so the test deliberately
fed the controller a previously captured straight-track frame on a temporary
private camera topic. Emergency stop and explicit zero commands were verified
before motion. The installed `car-interface` container was then paused to
remove `/duck2/kinematics_node` from the wheel topic and give the test exclusive
control. The controller released one approximately one-second pulse within the
stand limits. Executed output remained at `0.07` left and `0.03` right; encoder
deltas were `126` left and `96` right. Ctrl+C produced a zero request and the
final executed output was `0.0/0.0`. An independent timed stop was also armed.

The user observed both wheels turn together for about one second and then
stop. The user could not determine their direction, so forward polarity is
still unverified. After the test, emergency stop and explicit zero were
asserted again, `car-interface` was restored healthy, `/duck2/kinematics_node`
was restored as the sole normal wheel publisher, and all temporary robot-side
test files were removed.

At the end of the lifted-wheel stage, ground direction and trim, stopping
distance, turns, routes, red-line driving behaviour, obstacle avoidance and
every autonomous movement check remained deferred. The first upright result is
recorded next.

## First upright straight check — 2026-09-08

With duck2 stationary in the right lane, a current 640x480 camera frame decoded
at an observed age of 0.016 seconds. The first diagnostic run reported a
white-only lane fallback because disabled obstacle processing still classified
two yellow dash sections as duck candidates and removed them from the lane
mask. The callback now skips duck detection entirely when obstacle handling is
disabled. The focused 28-test lane/readiness run passed, and the repeated live
diagnostic then reported both boundaries, no duck candidates, a lane error near
`-0.078`, and zero diagnostic wheel output. Existing HSV defaults were not
changed.

The first upright movement attempt used base speed `0.04`, wheel cap `0.05`,
steering cap `0.01`, disabled routes/obstacles/avoidance, and a planned
0.8-second window. The user observed duck2 move forward a short distance for
about one second. This establishes forward wheel polarity at those positive
commands. It does not establish steering polarity, lane-following stability or
stopping distance.

The movement orchestration itself failed its timing criterion. A foreground
ROS helper released emergency stop quickly but took several additional seconds
to exit, delaying the wrapper's next instruction. A later zero-only timing
probe measured 1.878 seconds to finish publishing and 4.931 seconds for the
process to return. The independent robot-side watchdog still asserted emergency
stop and explicit zero, which limited the observed motion to about one second.
The recorded feedback excerpt contained only the final zero samples, so it
cannot characterize the complete motion window. The project node was removed,
`car-interface` returned healthy, and `/duck2/kinematics_node` returned as the
sole normal wheel publisher.

`tools/bounded_ground_supervisor.py` replaces the failed orchestration for
bounded ground checks. One ROS process releases and reasserts emergency stop
around an absolute monotonic deadline. A second watchdog asserts stop before
release, has an independent deadline and stops if its parent pipe disappears.
The current test-only limits are duration up to `8.0` seconds, base/wheel speed
up to `0.05`, and steering up to `0.02`. It rejects missing camera/lane
readiness, competing publishers or missing stopped feedback. It is not a
deployment launcher.

## Supervised ground results — 2026-09-08

All ground runs used the temporary supervisor, paused `car-interface` only for
exclusive test ownership, and restored it healthy after explicit zero and
emergency-stop messages. The final driver feedback was zero for each completed
run. The user remained beside duck2 and supplied the visual observations below.

| Scene and setup | Command window | ROS/encoder evidence | User observation |
| --- | --- | --- | --- |
| Straight, unplugged | 0.5 s at equal `0.05` | left/right encoder deltas 18/20 | Moved a few centimetres forward and stopped promptly. |
| Straight, charger connected | 0.5 s at equal `0.05` | deltas 11/13 | Same visible behaviour and prompt stop. |
| Straight, centred | 3.0 s at `0.05` | deltas 100/101 | Appeared to travel straight. |
| Left curve, first trial | 3.0 s, left `0.04`, right `0.05` | deltas 67/73; white-only fallback | Initially curved correctly, then went straight toward the white border. |
| Left curve, longer trial | 6.0 s, left `0.04`, right `0.05` | deltas 135/141; white-only fallback | Same limitation was investigated further. |
| Left curve, stronger steering | 8.0 s, left `0.03`, right `0.05` | deltas 153/163; white-only fallback | Improved early curve response, then later approached the white border. |

The camera diagnostics detected both lane boundaries on straight scenes. On the
left curve, the yellow divider sometimes disappeared from the mask and the node
used the white-only fallback. The logs still showed a sustained left request;
the later failure therefore cannot be attributed to colour loss alone.

The installed wheel driver maps requested normalized values to PWM after adding
a minimum. Its verified source maps `0.03` to PWM 65 and `0.05` to PWM 69.
This explains why a small requested wheel difference can yield nearly equal
encoder rotation. It is a driver-interface fact, not a measured physical speed
model.

These results establish controlled start/stop behaviour and limited straight
movement. They do not establish reliable curve following, colour calibration,
braking distance, red-line handling, obstacle avoidance, route turns or
reverse. No automatic follow-up movement should occur from these records.

The current full offline suite passed 131 of 132 tests with one platform-specific
skip. The focused lane/readiness/supervisor suite passed 34 tests. The isolated
ROS transport suite also passed all camera, timestamp, stop, heartbeat,
obstacle, recorder, route, launcher and experimental-avoidance checks. The
watchdog's isolated parent-loss path reached
`WATCHDOG_STOPPED_PARENT_LOST` with robot networking disabled.
