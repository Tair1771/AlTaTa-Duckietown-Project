# Software verification — 2026-09-08

Scope: `feat/ros2-autonomy-readiness`, based on `ea999ce`. No robot connection,
actuation, calibration change or platform flash was performed in this session.

## Results

| Check | Result |
| --- | --- |
| Pinned ROS 1 AMD64 application / catkin build | Passed |
| Full test discovery in isolated Noetic container | 147 tests, 145 passed, 2 Windows-specific skips |
| Actual ROS 1 transport script, executed by discovery | Passed camera freshness/loss, zero shutdown, red stops, obstacle stops, routes, HTTP acknowledgements, heartbeat loss, recorder and combined launcher |
| ROS 2 Jazzy AMD64 / colcon build | Passed |
| Installed ROS 2 package launch / Ctrl+C | All three processes started and shut down cleanly; `timeout` exit 124 is expected for the bounded smoke test |
| Repeated ROS 2 diagnostic fault trials | Three consecutive passes, followed by two passing expanded tests |
| Frozen-autonomy diagnostic stop observation | 0.211 s, 0.175 s, 0.246 s in the repeated trials; 0.246 s in the expanded trial |
| Expanded ROS 2 integration | Passed invalid source-stamp preservation, supervisor restart/new session, no automatic rearm and independent Stop |
| Replay | Deterministic simulated wheel/error traces, labelled metrics, recorder manifest input and non-overwriting reports tested |
| Read-only inspection | Missing node, wrong type, wrong wheel publisher, uptime-independent identity and cache tests passed |

The ROS transport script runs at import before unittest's reported timer, so the
reported roughly four-second unittest duration is **not** the full end-to-end run time.
The stop observations are wall-clock polling measurements of diagnostic output on
this laptop, not certified maximum latency or physical stopping distance. No ARM64
ROS 2 build or native DTPS wheel actuation was verified.

## Failures found during development

- The pinned Duckietown image expects `/proc/1/cpuset`, absent in this laptop's
  container configuration. Offline tests bypassed its robot-specific entrypoint and
  explicitly sourced the catkin workspace. The application image itself built normally.
- The first multithreaded ROS 2 executor candidate intermittently delayed camera/status
  callbacks and failed safe. The final design serializes perception with one OpenCV
  worker; independent expiry and Stop remain in another process. The test camera also
  runs continuously rather than pausing while the test performs service calls.
- Default ROS 2 signal shutdown could invalidate publishers during cleanup, and launch
  could forward a signal twice. Explicit signal ownership keeps the context alive through
  cleanup and ignores repeated termination signals during that cleanup.
- Several test-harness issues (ROS setup path, installed launcher availability and
  parameter-client API spelling) were corrected; unsuccessful trials are not counted
  as passes above.

## Reproduction and limits

Commands, exact base-image digests, architecture, QoS, safety contract and ordered human
checks are in [ROS2_READINESS.md](ROS2_READINESS.md). CI runs network-isolated tests;
its remote result has not been claimed before the workflow actually runs.

No available labelled real-track dataset was fabricated. Replay metrics from synthetic
images establish regression behavior only. Stable curves, braking margin, non-duck
obstacles, real junctions, native source timestamps, downstream motor timeout and the
final route demonstration remain required physical work.
