# Local verification

## Experimental duck avoidance update (2026-09-07)

Added 18 synthetic controller/perception tests and four obstacle-language tests.
The combined regression command ran 109 tests: 108 passed and one Windows-only
test was skipped in Linux. Native Windows runs then passed all ten preview tests
(including that lifecycle test) and the separate interpreter-window lifecycle
test with sockets, HTTP, child processes and connected-code imports blocked.

The isolated ROS suite passed against the current source mounted into the image:
existing lane, route, gateway, heartbeat, recorder and launcher checks, plus a
JPEG-driven pass/return sequence retaining a configured route, driving-disabled
zero output and SIGINT zero-message delivery during passing.

A read-only probe of the supplied 1200x1600 tabletop photo found two compact
yellow candidate regions. Both boxes were clipped at the detector's 45%-height
ROI boundary; this was a colour/body-candidate check, not a complete silhouette
or road test. No three-boundary road geometry was found, as expected.

Passing remains disabled in all existing launchers. No robot was contacted.
See [DUCK_AVOIDANCE.md](DUCK_AVOIDANCE.md) for provisional parameters, limits and
physical checks. Rebuild before future deployment; these checks mounted current
source and did not update the stored image.


## First-test readiness update (2026-09-07)

The node now validates settings before subscribing, rejects invalid/stale/repeated
camera timestamps, checks freshness again after processing, resets steering on
stops and lane loss, and exposes camera/lane/stop diagnostics. The existing colour
bounds remain unchanged; they can now be supplied as validated ROS parameters.
See [FIRST_TEST_READINESS.md](FIRST_TEST_READINESS.md) for parameter names and the
physical test sequence. These are independently implemented changes to our node;
no external team's vision or movement implementation was used.

Run the additional synthetic regression suite in the same isolated image:

~~~bash
docker run --rm --network none -v "$PWD:/project:ro" --entrypoint python3 duckietown/template-ros:v3-amd64 /project/tests/test_first_test_readiness.py
~~~

For integration checks against the current Python source without rebuilding the
unchanged ROS dependencies, mount the node source into the existing image as well:

~~~bash
docker run --rm --network none -v "$PWD:/project:ro" -v "$PWD/packages/duckie_lane_follower/src:/code/catkin_ws/src/duckiebot-ros/packages/duckie_lane_follower/src:ro" --entrypoint bash duckietown/template-ros:v3-amd64 -lc 'source /environment.sh >/dev/null 2>&1; python3 /project/tests/test_ros_transport.py'
~~~

The new integration cases establish that zero, stale, future, duplicate and
out-of-order image timestamps stop wheel output, and fresh frames allow ordinary
lane-following recovery. Camera debug, Ctrl+C zero-message delivery, persistent
red stops and the pre-existing route/gateway/recorder checks also passed against
the updated mounted source. No robot was contacted. All 77 regression tests passed: 53 robot tests (including
14 new readiness tests), 11 existing companion tests and 13 offline-interpreter
tests. Rebuild before deployment;
the previously built image alone does not include this source update.

Run from the existing repository in Ubuntu-22.04:

~~~bash
dts devel build -f
docker run --rm --network none -v "$PWD:/project:ro" --entrypoint bash duckietown/template-ros:v3-amd64 -lc 'python3 /project/tests/test_lane_follower.py'
docker run --rm --network none -v "$PWD:/project:ro" --entrypoint bash duckietown/template-ros:v3-amd64 -lc 'python3 /project/tests/test_navigation.py'
docker run --rm --network none -v "$PWD:/project:ro" --entrypoint bash duckietown/template-ros:v3-amd64 -lc 'python3 /project/tests/test_obstacles_and_connection.py'
python3 tests/test_chat.py
docker run --rm --network none -v "$PWD:/project:ro" --entrypoint bash duckietown/template-ros:v3-amd64 -lc 'source /environment.sh >/dev/null 2>&1; python3 /project/tests/test_ros_transport.py'
~~~

The regression suite uses actual OpenCV with synthetic images and mocked ROS.
The transport suite starts an isolated ROS master and the actual built node.
Its simulated camera publishes JPEG messages; its wheel listener checks actual
received messages. Docker network isolation prevents connection to the robot.
The built image must match current source before running the transport suite.

Verified 2026-09-07:
- 50 regression tests: 11 lane/safety, 16 route/command, 12 obstacle/heartbeat, 11 chat-engine.
- Real ROS debug zero output, camera timeout/recovery, SIGINT zero message delivery,
  and persistent red-line stopping passed.
- Real ROS route placement, speed commands, turn override, stop dwell, visual
  reacquisition, route advancement and remote Stop passed with synthetic images.
- The actual HTTP gateway and desktop transport passed token checks, status
  reading and acknowledged speed/stop commands in the isolated container.
- Native Tkinter window construction passed on Windows. API responses were mocked
  in chat-engine tests. A small local model was also evaluated live and rejected
  after failing important context cases; see LOCAL_MODEL_EXPERIMENT.md.
- A Windows Python client reached a driving-disabled ROS sandbox through a Docker
  port bound to localhost. Status, speed acknowledgment, heartbeat loss, explicit
  restart requirements and debug Continue rejection passed.
- Actual ROS obstacle stop/clear/release and laptop-timeout stopping passed.
- Camera-only recording produced five readable PNGs with valid JSONL metadata.
- The combined lane-chat launcher started both processes and delivered zero wheel
  output on Ctrl+C.
- No physical bot link, camera calibration, turn calibration or full lap is proven.
- All six ROS packages built successfully and the Docker image was exported.
  The installed dts reports an unrecognized v3 distro and returned exit status 1
  despite reporting successful packaging. The resulting image ran the tests.
- These are laptop amd64 checks; no ARM build or physical track test is proven.

## Physical validation still needed
Start with camera debug and inspect masks at actual track lighting, including
red lines and red objects outside the lane. The red stop ROI begins at 65% of
image height and covers x=35%-95%. This is a heuristic corridor, not calibrated
distance; do not assume the bot will stop before crossing until measured.

With wheels lifted, verify steering direction, ramped speed and Ctrl+C stopping.
Stand mode uses base .05, wheel cap .07, steering cap .02.
Road mode uses base .08, wheel cap .18; speeds are normalized commands, not m/s.
Only then measure lane following and stopping on the physical track.

Default lane-follow mode keeps a persistent red stop. Route mode can continue
after its stop dwell when placement is confirmed and calibrated junction control
is explicitly enabled. Route and chat behavior are described in COMMAND_INTERFACE.md.
The turn profile is provisional; do not present software state tests as physical
route-following evidence. Obstacle recognition remains provisional, particularly for dark objects.
