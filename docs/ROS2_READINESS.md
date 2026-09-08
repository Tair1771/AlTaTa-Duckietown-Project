# ROS 2 readiness and hardware handoff

Branch: `feat/ros2-autonomy-readiness`, based on teammate commit `ea999ce`.
This is a **diagnostic migration candidate**, not approval to replace duck2's drivers.
ROS 1 remains the physical baseline. Neither branch nor test success guarantees a grade.

## What changed

- `duck2_core.py`: shared vision, immutable timestamped observations, command validation,
  navigation state machine, steering filtering, slew/acceleration limits and fault stops.
  Transport and clocks are injected. The engine is stateful, not a collection of entirely
  pure functions. Perception computes on a private snapshot; committing results still
  requires a fresh source timestamp under the controller's lock.
- `lane_follower_node.py`: thin ROS 1 adapter, retaining the old topics and parameters.
- `ros2/`: ament package, parameter file, launch, adapter, independent diagnostic supervisor
  and the same authenticated HTTP command gateway. No `duckietown_msgs` wheel publisher
  is created by this package. Wheel values remain normalized requests, **not m/s**.
- `tools/replay.py`: reads camera-recorder manifests or labelled frame manifests and
  writes per-frame CSV, summary JSON and an SVG lane-error trace without ROS or movement.
- Bounded ROS 1 test supervisor: watches parent-pipe loss after GO, checks child death,
  rechecks preflight before release, and requires fresh post-stop zero feedback.
- Read-only inspection: requires all expected containers, nodes and topic types; individual
  ROS command errors fail the check. Fingerprints exclude uptime; cache names include microseconds.
- `lane-obstacle-debug`: detects lanes and obstacles with driving disabled and diagnostic
  wheel output. `lane-follow` is explicitly labelled lane-only calibration, not the final demo.

## Architecture and safety boundary

```text
Camera acquisition stamp + compressed image
    -> ROS 1 adapter OR ROS 2 adapter
    -> shared OpenCV perception -> Observation
    -> shared command/navigation/controller state
    -> ROS 1 wheels (existing supervised test path)
       OR ROS 2 request -> separate lease supervisor -> diagnostic JSON wheels ONLY
```

ROS 2 camera subscription: best-effort, volatile, depth 1; it can consume a best-effort
camera publisher without retaining a queue of old images. Commands and wheel requests
use reliable, volatile delivery with bounded depth; status from the core is transient-local.
The adapter defines separate callback groups, but perception uses a single-threaded
executor: the first multithreaded candidate showed intermittent camera/status starvation
in repeated tests. OpenCV is limited to one worker thread. Safety expiry and direct Stop
run in a **different process**, not another callback in the perception node. Thus a slow
image can delay controller acknowledgements, but cannot block the supervisor's stop path.
See [ROS QoS documentation](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
and [callback groups](https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html).

The diagnostic safety contract uses a receiver-generated session and epoch, increasing
sequence numbers, a 250 ms request lease and a maximum 8-second arm window. Requests
must lie in `[0, 0.05]` for each wheel. Stop/expiry disarms; new frames or queued requests
cannot rearm. The supervisor also needs fresh camera-valid status. Invalid/expired requests
stop output; old session/epoch/sequence requests are ignored without extending a lease.
The normal desktop Stop is enforced by both the shared controller and an independent
supervisor subscription. Stop invalidates the supervisor epoch. Continue clears the
controller stop, but a separate safety rearm is still required for diagnostic output.
The independent safety Stop service also disarms without waiting for perception.

`issued_monotonic` is meaningful **only on one host, in the same monotonic clock domain**.
Do not distribute the autonomy/supervisor pair between laptop and robot, or use a separate
time namespace. Docker containers on the same Linux kernel normally share that clock.
DDS is not authentication: keep this diagnostic stack loopback-only. Exposing HTTP beyond
loopback requires `DUCK2_CONTROL_TOKEN`; use an SSH tunnel rather than cleartext LAN control.

Acquisition timestamps are preserved, never replaced on receipt. This cannot recover a
timestamp already replaced by an upstream bridge. Therefore `source_timestamp_verified`
defaults false and refuses arming. Setting it true in the integration test is valid only
because that test owns the synthetic image source and clock.

**Supervisor death cannot stop a real motor by itself.** A verified downstream driver
timeout/estop must enforce zero if the supervisor, DDS, DTPS bridge, router or host dies.
That is a hardware acceptance gate, not something proved by a Python unit test. The
diagnostic package intentionally has no hardware output adapter until it is satisfied.

## Build and verify on a laptop

Use a local Docker context, never a robot Docker daemon. Linux/WSL shell, source-repository root:

```bash
python3 tools/build_local.py --pull
docker build -f ros2/Dockerfile -t duck2-ros2:dev .

# ROS 1: real isolated ROS transport, core, desktop, replay and safety tests.
docker run --rm --network none -e DT_ENTRYPOINT_SOURCED=1 \
  -v "$PWD:/code/catkin_ws/src/duckiebot-ros:ro" \
  -w /code/catkin_ws/src/duckiebot-ros --entrypoint bash altata-duck2:noetic-amd64 \
  -lc 'source /code/catkin_ws/devel/setup.bash; python3 -m unittest discover -s tests -v'

# ROS 2: real DDS processes, camera source, gateway and injected failures.
docker run --rm --network none -v "$PWD:/work:ro" -w /work \
  -e PYTHONPATH=/work/packages/duckie_lane_follower/src:/work/ros2 \
  duck2-ros2:dev python3 tests/integration_ros2.py

# Installed package launch (no camera, no motion; stop with Ctrl+C).
docker run --rm -it --network none duck2-ros2:dev
```

`DT_ENTRYPOINT_SOURCED=1` above bypasses hardware-specific Duckietown initialization
that expects `/proc/1/cpuset`; the ROS workspace is explicitly sourced instead. This
is an offline-test accommodation, not a recommended robot startup change.
The ROS 2 base is digest-pinned. Apt package versions are not frozen; record image IDs
and installed versions for the submitted experiment. The new ROS 2 image was tested
on AMD64, not ARM64 hardware.

The ROS 1 node is now a wrapper: when staging source for a supervised test, copy the
whole `packages/duckie_lane_follower/src/` directory, including `duck2_core.py`.
Streaming only `lane_follower_node.py` over stdin no longer supplies its dependencies.
The built image already contains and installs the shared module.

## Replay and calibration workflow

Use the existing camera recorder (`docs/CAMERA_RECORDING.md`) to obtain `frames.jsonl`
and PNG files. Keep real captures outside Git. Replay accepts its `file` and
`camera_stamp` fields directly. Alternatively use one JSON object per line:

```json
{"t":0.0,"image":"frame-000.png","label_error":0.015625}
{"t":0.05,"image":"frame-001.png","command":{"id":"stop-1","action":"stop"}}
```

Times must increase. `label_error` is the manually labelled normalized lane-center error,
not pixels or metres. The summary reports MAE only for labelled frames with a detection;
always report lane detection coverage too, so missed lanes cannot make MAE look better.
The SVG joins detected samples and omits missing detections; use CSV for gaps/fault analysis.
Processing duration is host compute time, not camera-to-motor latency.

```bash
# /captures mounted read-only; /results writable. Choose a NEW output folder each run.
docker run --rm --network none -v "$PWD:/work:ro" -v "/path/to/captures:/captures:ro" \
  -v "/path/to/results:/results" -w /work \
  duck2-ros2:dev python3 tools/replay.py /captures/session/frames.jsonl /results/trial-01
```

Pass `--params /path/to/params.json` for a JSON object using parameter names without `~`.
Tune on a training subset; retain different sessions/lighting/curves as held-out data.
Replay is open-loop: output cannot change the recorded next frame. It cannot demonstrate
closed-loop stability, obstacle clearance or route completion.

## Required human session — do this next, in order

1. **Agree the scope and recovery plan.** Confirm rubric/deadline and whether ROS 2 is
   allowed. Preserve the original SD card/image, configuration and `main`. Do not flash
   Ubuntu 24.04 onto duck2 just because Jazzy uses it. Its confirmed host is ARM64 Ubuntu
   18.04; the Ente/native bridge compatibility needs a separate decision.
2. **Stationary camera data.** Confirm current topics/types with read-only inspection.
   Capture straights, both curve directions, dashed gaps, glare/shadows, red lines,
   ducks, non-yellow obstacles, clear lanes and blocked adjacent lanes. Label lane
   centers and obstacle stop regions. Run lane-obstacle-debug without wheel output.
3. **ROS 1 safety regression, wheels lifted.** A person owns the physical kill switch.
   Verify exclusive wheel publisher ownership, wheel polarity, fresh executed feedback
   and the fixed watchdog. Kill the parent after GO; interrupt camera and feedback.
   A missing fresh zero must report failure, not a successful stop. Restore the normal
   `car-interface` afterward using the already verified procedure in ROBOT_SETUP.md.
4. **Low-speed ground calibration.** Use the bounded supervisor, starting with the
   existing sub-second trial, a clear track and a spotter. Measure steering sign,
   actual PWM deadband, curve behavior, stopping latency and stopping distance.
   The teammate observed commands .03 and .05 mapping to PWM 65 and 69: more aggressive
   software gain alone is not evidence of a fix. Do not raise speed to hide the problem.
5. **ROS 2 native feasibility, stationary first.** Pin Ente/DTPS/bridge revisions compatible
   with this hardware. Identify real message types and preserve acquisition-time meaning.
   Measure bridge age and prove no old nonzero request is replayed after reconnect.
   Verify driver-side timeout when the supervisor itself is killed. Only then implement
   the small native wheel-output adapter, repeat lifted-wheel checks, and decide go/no-go.
   If compatibility or safety fails, keep ROS 1 driving and ROS 2 as a diagnostic demo.
6. **Obstacle stop-only before passing.** Enable calibrated obstacle detection; test slow
   approach, stop before contact, staying stopped and explicit Continue after clear.
   Test non-duck obstacles too. Current vision is heuristic candidate detection, not
   a guaranteed generic object detector. Steering around is optional under the stated
   project requirement; keep passing disabled until independently measured and safe.
7. **High-level commands and junctions.** Test stop, continue and speed changes first.
   Then calibrate red-stop placement, left/right/straight trajectories and outgoing-lane
   reacquisition. Turning is a queued next-junction maneuver, not an instantaneous yaw
   request. Position/route confirmation and junction calibration gates must remain.
   Test Stop during interpretation, crossing, camera loss and client-heartbeat loss.
8. **Repeatable final evidence.** Agree numerical acceptance thresholds with the rubric.
   Record every trial, including failures, using the template below. Repeat on held-out
   conditions; rehearse cold start, the full route and emergency recovery. Freeze features
   before the final demo; retain the ROS 1 fallback.

Record per trial: source commit, image ID, architecture, parameters, scene, battery state,
operator, initial placement, duration, lane departures, intervention count, obstacle
contacts, stopping distance, command acknowledgement latency, command completion observation,
and links to video/frames/CSV. An acknowledgement proves acceptance, not physical completion.

## Parallel team ownership

| Track | Own these files/work | Integrate using |
| --- | --- | --- |
| Teammate with robot | Captures, HSV/ROI and motor/curve calibration, physical observations | Labelled manifests + parameter JSON, no ROS 2 dependency |
| Migration work | `ros2/`, native feasibility notes, timestamp/timeout measurements | Existing shared-core inputs and request contract; never fork vision code |
| Commands/report | `laptop/`, held-out command cases, demo/report evidence | Unchanged HTTP/status/ack schema; no raw wheel commands from language code |

Keep physical parameter changes in a separate reviewed commit from transport changes.
Merge the shared-core and supervisor fixes only after the lifted-wheel regression; do not
merge ROS 2 as the default driving stack merely because diagnostic software tests pass.
