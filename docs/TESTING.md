# Testing status

## Offline results — 2026-09-08

The project was rebuilt against the same Duckietown ROS base release observed
on duck2: `dt-ros-commons:v4.3.0`, ROS Noetic, Python 3.8, OpenCV 4.2 and
NumPy 1.17. The package built successfully for both local AMD64 and matching
ARM64 images. The base image references are digest-pinned in
`config/duck2.json`.

The offline regression suite ran 118 tests: 117 passed and one platform-specific
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
  test_obstacles_and_connection test_first_test_readiness test_duck_avoidance \
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

## Still unverified physically

duck2's live camera has been passively verified: twelve compressed frames
decoded successfully at 640x480, timestamps advanced, and the latest frame was
well inside the node's 0.5-second freshness limit. The exact lane-follower
source processed those frames with driving disabled and a diagnostic wheel
topic; its logs reported only zero values and a clean shutdown.

No project wheel command, including a real-topic zero command, has been sent. The installed driver tests are recorded below. Colour
thresholds, steering direction, motor response, braking distance, junction
timing, obstacle passing and route behaviour remain provisional. Follow
[FIRST_TEST_READINESS.md](FIRST_TEST_READINESS.md) only after duck2 is secured
with both wheels clear and the team explicitly begins the wheel-test stage.

## Upside-down bench results — 2026-09-08

duck2 was secured upside down before the installed wheel-driver diagnostics
were used. Windows SSH access through the hotspot worked with the robot's
previously verified SSH host key. The ROS master, camera, IMU and front range
sensor were available. Camera headers advanced at approximately 30 Hz, well
within the follower's 0.5-second freshness limit.

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
built-in tests. Therefore the service result and encoder change establish the
driver/encoder interface evidence, but do not replace a person's visual
confirmation that each wheel rotated in the intended apparent direction and
stopped. Record that visual observation before treating either wheel direction
as confirmed for track driving.

The project lane-follower source was also run temporarily on duck2 with
`drive_enabled:=false`, display disabled and its output remapped to
`/duck2/lane_follower/diagnostic_wheels_cmd`. It processed live desk-view
frames, published five observed `0.0/0.0` diagnostic messages, and exited
cleanly. Its temporary source and process were removed. It did not publish to
the real wheel topic.

Track-only work remains deferred: stationary track-camera inspection, steering
direction and trim, ground-contact stopping distance, turns, routes, red-line
behaviour, obstacle avoidance and every autonomous movement check.
