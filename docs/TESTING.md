# Testing status

Latest 2026-09-09 update: [sharp-turn/equal-wheel diagnosis](TURNING_DIAGNOSIS.md).
The equal 0.15/0.15 retry travelled straight (216/215 encoder ticks), but unequal
rolling commands still produced nearly equal rotation. Restored only the bounded
corner pivot to the recorded successful 0.20/0.00 pair, retained white-return
handoff, fixed missed stall detection under bursty echoes, and added read-only
PWM-register recording. 167 focused local tests passed on Python 3.12/OpenCV 5;
the Docker/Noetic integration check was unavailable in this session. No movement
occurred during the diagnosis. Repeatable sharp-corner completion is unverified.

## Offline results — 2026-09-08

The project was rebuilt against the same Duckietown ROS base release observed
on duck2: `dt-ros-commons:v4.3.0`, ROS Noetic, Python 3.8, OpenCV 4.2 and
NumPy 1.17. The package built successfully for both local AMD64 and matching
ARM64 images. The base image references are digest-pinned in
`config/duck2.json`.

The latest offline regression suite ran 198 tests: 196 passed and two platform-specific
tests were skipped. The suites use synthetic images, a local isolated ROS master, mock ROS
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
  test_obstacles_and_connection test_first_test_readiness test_bounded_ground_supervisor \
  test_steering_transition test_duck_avoidance test_ground_test_session \
  test_chat test_offline_interpreter test_offline_chat_ui test_command_preview \
  test_obstacle_language'

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
The general test-only limits remain duration up to `8.0` seconds, base/wheel
speed up to `0.05`, and steering up to `0.02`. One explicit camera-guided curve
preset uses base `0.09`, wheel cap `0.15`, and steering cap `0.09`; arbitrary
values above the general limits remain rejected. Its curve-test proportional
gain is `0.59`, lane target is `0.441` of image width, filter alpha is `0.20`,
deadband width is `0.05`, steering bias is `0.0075`, and
`smooth_steering_deadband:=true` enables a continuous near-centre response.
These remain test-only
settings; the node defaults and normal launchers are unchanged. The supervisor rejects missing
camera/lane readiness, competing publishers or missing stopped feedback. It is
not a deployment launcher.

### Fixed-wheel turning diagnostic

For the next curve investigation, the supervisor also has a deliberately
restricted fixed-wheel mode. It does **not** start the lane follower or use
camera steering: it accepts only left `0.03`, right `0.15`, for at most eight
seconds. This makes the installed driver's minimum-PWM behaviour visible
without mixing it with lane perception. Before it can release emergency stop,
it requires a current camera frame, exclusive wheel-topic ownership and zero
executed-wheel feedback. It saves rate-limited raw compressed frames with a
timestamp manifest plus requested/executed wheel and encoder samples in a
temporary robot-side evidence directory. The watchdog now also stops if the
supervising parent process disappears during the active window.

This diagnostic remains a one-off, supervised physical test. It must be run
only after duck2 is safely positioned, the operator is watching it, and an
explicit live-test authorization is given. The evidence files are temporary
and are not source files to commit.

The camera-guided preset continuously checks camera freshness, lane status,
follower process health, executed-wheel feedback and wheel publisher ownership
during its bounded window. Any failure causes an early emergency stop and an
explicit zero request. The evidence telemetry includes every timestamped lane
status plus the observed camera-age and requested-steering ranges.

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
| Left curve, fixed-wheel calibration | 8.0 s, left `0.03`, right `0.15` | deltas 420/508; 42 current camera frames; final feedback zero | Stayed centered throughout the curve, continued turning after entering the straight as expected for fixed commands, and stopped promptly. |
| Left curve, first camera-guided trial | Stopped automatically after 7.28 s; base `0.09`, cap `0.15`, steering cap `0.06`, gain `0.20` | Both boundaries initially; white-only fallback after 2.85 s; maximum `0.03`/`0.15` from about 4 s; both boundaries lost at 7.25 s; final feedback zero | Understeered early, crossed partly over the curved white border and stopped while pointing away from the path. |
| Left curve, centred camera-guided trial | 8.0 s; base `0.09`, cap `0.15`, steering cap `0.06`, gain `0.30` | Both boundaries for the full window; maximum `0.03`/`0.15` from about 3 s; deltas 182/210; a fresh zero-feedback window followed one in-flight nonzero sample | Stayed mostly centered but slowed progressively and finished slightly right of the ideal centre. |
| Upside-down fixed-wheel load isolation | 4.11 s, left `0.03`, right `0.15` | commands stayed constant; deltas 373/619; fresh zero-feedback window confirmed | Both wheels appeared steady; the intentionally faster right wheel was visibly faster. |
| Left curve, stronger camera-guided profile | 8.0 s; base `0.09`, cap `0.15`, steering cap `0.09`, gain `0.30` | Both boundaries throughout; steering ranged from `-0.009` to `0.071`; executed wheels ranged left `0.008`–`0.107` and right `0.073`–`0.150`; deltas 457/540; final zero confirmed | Curve containment was excellent and speed appeared consistent. The inside wheel appeared to pause briefly during the strongest correction. After entering the straight, the robot continued correcting left longer than desired. |
| Isolated centered straight | 6.0 s with the stronger curve profile before target correction | Both boundaries throughout; steering ranged from `-0.027` to `0.027`; deltas 199/213; final zero confirmed | Drifted about 10 degrees left and had inconsistent loaded speed. |
| Calibrated centered straight | 6.0 s; target `0.456`, gain `0.40`, alpha `0.20` | Both boundaries throughout; encoder deltas 331/333; fresh zero-feedback confirmation after stopping | Began drifting left, then corrected before crossing the yellow divider. Finished looking straight but slightly left of lane centre. |

The camera diagnostics detected both lane boundaries on straight scenes. On the
left curve, the yellow divider sometimes disappeared from the mask and the node
used the white-only fallback. The logs still showed a sustained left request;
the later failure therefore cannot be attributed to colour loss alone.

The installed wheel driver maps requested normalized values to PWM after adding
a minimum. Its verified source maps `0.03` to PWM 65 and `0.05` to PWM 69.
This explains why a small requested wheel difference can yield nearly equal
encoder rotation. It is a driver-interface fact, not a measured physical speed
model.

The `0.03`/`0.15` result establishes sufficient fixed wheel differential for
this tested left curve. It is not a general lane-following calibration: a
camera-guided controller must reduce that differential as the road becomes
straight.

The first camera-guided trial isolated delayed steering as the next issue. Its
initial lane error was about `-0.20`, but gain `0.20` requested only about
`0.05`/`0.13`. The controller did not reach `0.03`/`0.15` until the accumulated
understeer had already moved the camera toward the outside of the road. The
test preset now uses gain `0.30`, which requests nearly the verified fixed-wheel
differential for that initial error while retaining the same speed caps and
automatic straightening.

The centred camera-guided trial showed that the controller did not reduce its
command when the robot appeared to slow: the requested average remained `0.09`
and both boundaries remained detected. The unloaded diagnostic then sustained
the same fixed wheel values. This isolates the inconsistent ground speed from
the lane controller's requested speed, while leaving the exact loaded cause
unresolved.

Recorded-image replay with the new test-only steering cap `0.09` preserved the
initial output, then progressively reduced the inside wheel below `0.03` as the
observed curve error grew, reaching `0.00`/`0.15` on the strongest frames. A
separate curve-to-straight recording reduced and reversed the correction after
the road straightened. This replay verifies calculation limits only; it does
not establish the future physical path.

The subsequent physical run confirmed the stronger profile's curve authority:
duck2 stayed contained and approximately centered through the curve. It also
confirmed that curve-to-straight settling still needs isolation. The raw lane
error moved steadily toward zero, and the controller reduced left steering and
eventually requested a small right correction, but the user observed continued
left correction after entering the straight. This is recorded as a partial
success rather than reliable combined curve-to-straight following.

The isolated straight run then exposed a stable detector offset at that pose. Across
the final 20 stationary preflight samples, a physically centered duck2 produced
a mean lane error of `-0.08852` with a range of only `-0.08974` to `-0.08796`.
This did not distinguish camera alignment from pose or perspective-weighted
lane detection; the target values below remain provisional. The controller
therefore began turning left from the user-reported centered start. The test-only
target was moved from image fraction `0.500` to `0.456`. Because this reduces
the apparent curve error too, test-only gain was raised to `0.40` to preserve
the successful curve command, and filter alpha was raised from `0.10` to `0.20`
to reduce curve-exit lag.

Offline replay with target `0.456`, gain `0.40`, alpha `0.20`, and steering cap
`0.09` requests equal `0.09`/`0.09` on the recorded centered straight start.
On the successful curve recording it begins near `0.027`/`0.150`, preserving
strong curve authority, and changes correction direction without the previous
filter sign lag. This is counterfactual software evidence; the calibrated
straight and combined curve-to-straight behavior still require one physical
verification each.

The first calibrated straight run improved containment but retained a small
left offset. Its preflight lane error averaged `-0.0293`, so the test-only
target is now refined to `0.441`. Replay shows this keeps the recorded centered
straight start within the `0.04` deadband while maintaining strong curve output.
The test-only gain was initially raised to `0.48`; a small loaded left drift
remained despite the camera estimate. The next test-only profile therefore
used deadband `0.05` and a bounded right-turn steering bias of `0.015`, with
gain `0.59` so the bias does not reduce left-curve authority. Offline replay
starts the recorded centered straight at `0.105`/`0.075` and the recorded curve
at `0.030`/`0.150`. The subsequent physical trial is documented below; this
trim is not established calibration.

These results establish controlled start/stop behaviour and limited straight
movement. They do not establish reliable curve following, colour calibration,
braking distance, red-line handling, obstacle avoidance, route turns or
reverse. No automatic follow-up movement should occur from these records.

The previously completed full offline suite passed 131 of 132 tests with one
platform-specific skip. The latest focused lane/readiness/supervisor suite
passed 50 tests, including 20 supervisor tests. The isolated
ROS transport suite also passed all camera, timestamp, stop, heartbeat,
obstacle, recorder, route, launcher and experimental-avoidance checks. The
watchdog's isolated parent-loss path reached
`WATCHDOG_STOPPED_PARENT_LOST` with robot networking disabled.

### 2026-09-08: continuous steering transition (offline fix)

The user observed slight right drift and mild steering oscillation in the latest
six-second straight trial, with smooth forward speed. They judged the right
offset comparable to the preceding left offset and were concerned that longer
travel might approach the white boundary. This is an observation, not a measured
heading or lateral displacement. Telemetry recorded approximately 6.032 seconds
and five requested steering phases: right, left, right, left, right. Encoder
deltas were 365/365; equal totals do not establish straight travel or centring.

The hard deadband and positive trim had a discontinuity at filtered error -0.05:
the pre-flip steering target jumped from +0.015 just inside to about -0.0145 just
outside. Slew limiting slowed that change but did not remove it. This is a
software mechanism consistent with the observed oscillation. Dashed markings
can affect independently weighted colour centroids; their exact contribution
has not been isolated. Stationary detector readings alone did not establish an
accurate camera-centre calibration, and earlier target/gain/trim changes were
not isolated experiments.

The bounded camera-guided preset now opts into `~smooth_steering_deadband`.
Within width d, it uses control error `e * r * (2 - r)`, where `r = abs(e)/d`.
Outside this region it uses e unchanged. Value and slope join continuously at
both boundaries. Zero width uses ordinary proportional control. The existing
filter, steering slew, wheel acceleration and all stopping gates remain in
place. The default is false, preserving ordinary launchers. Status adds the
mode flag, `filtered_lane_error`, and `steering_before_flip` (slew-limited, before
the configured sign flip); these are diagnostics, not evidence of motor motion.
The supervisor will not release its stop if the node does not acknowledge the
new mode, preventing a silently ignored parameter on an older node.

Target 0.441, gain 0.59, alpha 0.20, width 0.05, and trim 0.0075 are the next
provisional profile. The trim was reduced after the smooth six-second straight
run stayed centred for most of its length but ended slightly right by visual
observation. This isolates the smallest adjustment. Base 0.09, wheel cap
0.15, steering cap 0.09 and the eight-second duration ceiling remain unchanged.
At exactly zero error the provisional trim still requests 0.0975/0.0825; this
change does not claim to eliminate physical drift. Smaller trim values were
replayed separately and not adopted as measured calibration.

Three recordings were replayed with actual OpenCV and the controller, mocked
ROS transport, and Docker networking disabled. All 76 motion-window images
produced lane estimates. On the latest straight recording the sampled replay's
first request changed from 0.105/0.075 to approximately 0.0988/0.0812; steering
direction changes above a 0.003 threshold decreased from two to one. These
sampled replay counts differ from live telemetry because images were saved at
roughly 4 Hz, below the original callback rate. On the curve recording the
initial request remained approximately 0.0302/0.1498, and the response changed
direction on the later straight frames. Later right steering still reached
the cap, so this is not proof of ideal curve-exit behaviour. Replay keeps the
old camera trajectory; it does not predict a new physical trajectory.

To repeat the calculations, put recordings (JPEGs, frames.jsonl, telemetry.json)
outside the repository and run the offline script inside the existing image:

```bash
docker run --rm --network none -v "$PWD:/project:ro" \
  -v /absolute/evidence:/evidence:ro -v /absolute/reports:/reports \
  --entrypoint bash altata-duck2:noetic-amd64 \
  -lc 'source /environment.sh >/dev/null 2>&1; python3 /project/tests/replay_steering.py \
  /evidence/straight /evidence/curve --output /reports/steering-replay.json'
```

The script compares the legacy response with the smooth response at trim
0.015, 0.0075 and zero. Reports and images remain outside Git. The script tests
perception/control calculations; controller safety gates are checked by the
separate unit and ROS integration suites. No robot services or software were
changed for this offline fix. A new Go is required for any physical retry.

Verification of this change:

- The full selected offline regression suite ran 167 tests: 165 passed and two
  native Windows Tk lifecycle checks were skipped inside Linux Docker.
- After adding the old-node release guard, the final supervisor/steering check
  ran 39 tests and passed. This includes the additional release-guard test.
- The isolated ROS integration suite passed, using the current local package
  mounted over the image's package source. It checked smooth-mode parameter
  delivery, camera-debug zeros, stale-camera stop/recovery, SIGINT zero delivery,
  and the existing red-stop, heartbeat, obstacle, route and launcher regressions.
- The first integration attempt exposed retained private ROS parameters between
  test scenarios. The harness now clears its isolated node namespace before
  each start and checks that the default mode and zero bias are restored; the
  complete rerun passed. This changed the isolated test harness, not duck2.

### 2026-09-08: provisional straight baseline accepted

The next eight-second supervised straight run used the smooth controller with
target `0.441`, gain `0.59`, alpha `0.20`, deadband `0.05`, trim `0.0075`, base
`0.09`, wheel cap `0.15`, and steering cap `0.09`. The user observed that it was
near-perfect and accepted the straight-path motion for the current track setup.
This is a provisional baseline rather than universal calibration.

The supervisor's final console summary reported approximately 8.028 seconds,
camera age from 0.050 to 0.126 seconds, 80 lane-status samples, encoder deltas
515/509, and a fresh post-stop zero window. It reported 61 sampled evidence
frames. The detailed frames and telemetry were mistakenly deleted from both
temporary robot locations before the attempted laptop transfer, so those values
cannot be independently reconstructed from a saved bundle. The available
summary and the user's visual observation are retained without claiming that a
complete recording exists. This run must not be repeated merely to replace the
missing evidence.

The next physical milestone is one unchanged-profile left curve-to-straight
run, followed by one right bend-to-straight run if the first passes. Each is
limited to eight seconds and requires a fresh user `Go`.

### Evidence preservation and cleanup workflow

`tools/run_duck2_ground_test.py` now coordinates an explicitly authorized
camera-guided test from Windows. It creates a unique run ID, stages temporary
source, stops `car-interface`, invokes the existing bounded supervisor, and
then checks that all temporary ROS nodes have exited before restoring normal
control. Restoration is accepted only when `/duck2/kinematics_node` is the sole
publisher of the normal wheel-command topic.

The supervisor refuses a nonempty evidence directory and writes
`completion.json` beside `frames.jsonl`, `telemetry.json`, and the JPEG files.
The Windows wrapper streams a tar archive directly from the ROS container,
rejects unsafe archive paths, checks every manifest entry, validates the stop
flags and telemetry structure, and writes SHA-256 hashes to
`evidence_verified.json`. Laptop files are stored under
`%LOCALAPPDATA%\Duck2\evidence`, outside Git. The archive and incomplete files
are retained when validation fails, and the wrapper never deletes robot-side
evidence or staging directories.

The wrapper requires `--confirm-go`; this flag records that a fresh physical
authorization was received and does not replace the user's observation. It
must only be invoked while the user is watching the prepared robot:

```powershell
py -3 tools\run_duck2_ground_test.py `
  --label left-curve-to-straight --duration 15 --confirm-go
```

Focused offline verification after this change ran 67 tests successfully in
the network-isolated `altata-duck2:noetic-amd64` image. It covers the unchanged
camera-guided preset, evidence-directory reuse, completion records, missing or
unsafe frame entries, unsafe tar paths, failed preservation retaining files,
temporary-node cleanup, restoration ordering, and rejection of unexpected
wheel publishers. No robot connection or movement occurred during these tests.

### 2026-09-08: left curve-to-straight result and faster 15-second test support

Run `20260908T173029Z-left-curve-to-straight-992e2b44` completed its requested
eight-second window (8.026 seconds measured). The user observed satisfactory
curve following, but poor adjustment on the straight: duck2 ended facing too
far right with part of a wheel on the right white boundary. The user also
reported insufficient time on the straight to assess settling. Record this as
a failed combined transition despite successful curve traversal; it is not a
reason to advance to the right-bend milestone yet.

The supervisor reported camera ages of 0.039--0.140 seconds, 80 motion-window
status samples, and a fresh final zero-feedback window. `post_stop_all_zero`
was false, while `post_stop_zero_confirmed` was true; the two describe different
windows and must not be reported interchangeably. Normal control was restored
and `/duck2/kinematics_node` was verified as the sole wheel publisher.

The archive, JSON telemetry and all 62 manifest-listed JPEGs are preserved
outside the repository and on duck2. The original completion record says 61
frames, so strict validation remains failed for this historical bundle. Do not
alter its raw record to force a pass. A late recording callback produced the
extra frame after the summary count was sampled. The recorder now locks and
freezes before counting completion frames; its focused regression passes.

Exit telemetry shows alternating estimates when yellow detection drops out.
At approximately 6.05 seconds both boundaries gave error -0.09; at 6.15 seconds
white-only fallback gave +0.15. Similar sign changes recur toward the end.
Saved images confirm a straight road in view by approximately six seconds.
This identifies an inconsistent perception estimate at the transition; it does
not establish that removing smoothing or increasing duration will fix the
physical path. Steering and colour settings remain at the accepted straight
baseline pending focused perception diagnosis.

At the user's request, camera-guided tests now allow an explicitly requested
duration up to **15 seconds**, with the independent watchdog capped at 15.25
seconds. The wrapper still defaults to eight seconds unless `--duration 15`
is supplied. General low-speed and fixed-wheel diagnostics retain their
eight-second caps. Speed, gain, trim, smoothing, freshness checks and stopping
gates are unchanged. The outer process timeout runs inside the ROS container
and includes startup/cleanup time; it is separate from the motion deadline.

To reduce overhead, both source files are uploaded directly into a unique
container directory through one compressed transfer, replacing six upload/setup
connections. Container state and post-test publisher ownership are queried
together. Evidence is downloaded as a compressed archive and retains the same
strict local verification. Session JSON includes durations for upload, the
supervised run, restoration and evidence transfer. No faster wall-clock result
is claimed until this path has been exercised on an authorized physical run.

Do not repeat runtime-version inspections, search for Python, or rerun the
offline suite before every unchanged physical trial. Use the known Windows
Python executable and SSH alias. Let the bounded supervisor perform live
camera/status, ownership, feedback and stopping checks once per run. A changed
robot runtime or a specific failure still requires investigation. If final
zero feedback is unconfirmed after an interrupted test, the wrapper leaves
normal control stopped for review instead of automatically starting it. A
failed upload before handover does not change robot services.

Verification: 74 focused offline tests passed with Docker networking disabled,
including acceptance of 15 seconds, rejection above 15, the 15.25-second
watchdog deadline, unchanged steering preset, single-transfer staging,
compressed evidence extraction, interrupted sessions, failed downloads,
missing-stop-feedback gating and rejection of competing publishers. No robot
connection or movement occurred while making these changes. A new Go is
required for another physical run; duration support alone is not approval to
repeat the known failing curve exit.

### 2026-09-08: focused yellow-dash fallback fix (offline)

Replay of the preserved failed run isolated the discontinuity. On frame 49,
both boundaries placed the normalized lane error at `-0.102`. When the yellow
dash disappeared on frame 50, the old fixed-width white-only fallback changed
that error to `+0.133`. Frame 51 returned to `-0.111`; frame 52 changed to
`+0.070` under the old fallback. Those raw sign changes weakened or reversed
the needed left correction even though the visible white boundary moved
continuously.

The lane node now has an opt-in temporal lane-width fallback. While both
boundaries are valid it records their measured half-width; during a one-boundary
frame it uses that value instead of the fixed `0.27` image-width assumption.
The same replay produced errors `-0.101` and `-0.145` on frames 50 and 52. The
steering request therefore remained in the needed direction and retained the
existing `0.012` per-sampled-frame steering-change bound. Both-boundary results,
colour values, gain, smoothing, trim, speed and wheel caps are unchanged.

The option defaults to false and no ordinary launcher enables it. Only the
bounded camera-guided preset turns it on. The supervisor checks the exact
boolean status before releasing its stop, and lane loss, invalid images or an
implausible boundary pair clear the remembered width. A synthetic callback test
also confirms that the value survives the node's private perception snapshot
and is cleared after lane loss.

After rebuilding the local AMD64 image, the network-isolated offline suite
passed 172 tests with one platform-specific skip. The isolated ROS suite also
passed its compressed-image, timestamp, timeout, recovery, SIGINT zero,
red-stop, connection, obstacle, recording, route, launcher and shutdown checks,
including delivery and default-off verification of the new ROS parameter. The
replay report is stored outside Git at
`~/.cache/duck2-temporal-fallback-replay-20260908.json`.

This is software evidence for the recorded images, not proof of physical curve
following. Duck2 did not move during the change or verification. The next step
is one supervised left curve-to-straight run with the bounded preset and a fresh
`Go`; the right-bend and red-line milestones remain deferred.

### 2026-09-08: dim-light curve failure and bounded fallback correction

The first physical run after the temporal fallback change did not visibly move.
Its telemetry showed an extreme left request and only four left-encoder ticks;
this result did not justify reducing steering authority. A subsequent dim-light
run used the test-only yellow HSV range `[20, 70, 80]` to `[35, 255, 255]` and a
mistakenly reduced steering cap of `0.06`. The user observed that duck2 followed
the curve initially, then understeered across the white boundary and stopped.
Telemetry recorded 331/380 encoder ticks and showed that both boundaries were
available for only 15 status samples, followed by roughly 4.4 seconds of
white-only fallback before lane loss.

The bounded camera-guided preset now restores the proven `0.09` steering cap.
The temporal width estimate may bridge a single-boundary gap for at most `0.30`
seconds. This limit covers the longest `0.201`-second gap found in the accepted
recording with margin, while preventing several seconds of steering from stale
lane geometry. The test-only preset also stops when a lone white boundary moves
left of `0.43` of the image width, before the existing `0.35` detection cutoff,
and it will not release the emergency stop unless the latest status confirms
that both lane boundaries are visible. These guards are opt-in; ordinary
launchers and their colour defaults remain unchanged.

Exact-preset replay detected both boundaries in all 30 sampled motion frames of
the earlier accepted curve recording. On the failed dim-light recording, it
stopped producing a lane command at frame 32; 16 of 23 sampled motion frames
were rejected rather than continuing the old white-only estimate. Replay proves
only the software response to saved images, not the future physical path.

The rebuilt AMD64 ROS image passed 157 network-isolated regressions with one
platform-specific skip. The isolated ROS checks passed debug-zero output,
timestamp rejection, camera timeout and recovery, the new opt-in parameter
delivery, SIGINT zero delivery and persistent red-line stopping. Duck2 did not
move during this implementation or verification. A physical retry still needs
the familiar one-third left-curve placement and a fresh `Go`.

### 2026-09-08: 15-second bounded-fallback physical result

Run `20260908T184452Z-left-curve-bounded-fallback-911fffda` completed its full
15.023-second motion window. The user observed insufficient curve tracking,
the right wheel crossing the white boundary, subsequent automatic adjustment
on the straight, prompt stopping, and inconsistent forward speed near the end.
This is a failed curve-to-straight result.

The evidence bundle was verified with 88 readable frames and retained on both
the laptop and robot. The supervisor recorded fresh camera data, 150
motion-window lane-status samples, no early fault, a confirmed post-stop zero
window, and restoration of `/duck2/kinematics_node` as the sole normal wheel
publisher. Both boundaries were detected for 148 samples; the remaining two
were short white-only temporal fallbacks.

During approximately the first seven seconds, the average requested curve
commands were close to `0.03/0.145`. Their encoder ratio matched the earlier
successful fixed `0.03/0.15` and camera-guided curve evidence, so a reversed
wheel or lost yellow divider does not explain this run. On the straight, the
requested and executed commands remained stable near `0.127/0.053`, but both
encoder rates fell together from roughly 60--80 ticks/s to 17--30 ticks/s and
then recovered. This proves that the late speed change was not caused by ROS
command jitter. The right-wheel request was again near the driver's nonlinear
low-command region; track contact, load and power behaviour remain possible
physical contributors.

The bounded preset now uses an opt-in gain schedule. Clear curve errors reach
the full gain `0.75` by normalized error `0.09`; errors at or below the existing
`0.05` deadband use gain `0.35`, with a smooth transition between them. Replay
of this run increases the curve correction while changing its final straight
request from about `0.127/0.053` to `0.115/0.065`. The latter keeps the slower
wheel farther above the observed low-command region. The steering cap remains
`0.09`, and the existing acceleration and steering-change limits still apply.
Defaults preserve the original single-gain behavior, so ordinary launchers are
unchanged.

After rebuilding the AMD64 image, 158 network-isolated regressions passed with
one platform-specific skip. The complete isolated ROS suite also passed,
including delivery of the scheduled-gain parameters, timestamp and camera
watchdogs, zero-output shutdown, command transport, recording, route handling
and default-off experimental features. No physical retry occurred during this
change; another run requires a fresh `Go`.

### 2026-09-08: scheduled-gain run exposed a physical stall

Run `20260908T190226Z-left-curve-gain-schedule-bb43ea24` completed its original
15-second window. The user observed good curve tracking, a rightward trend and
minor right-boundary crossing on the straight, followed by decreasing speed and
a complete physical stop. Motor sound continued for approximately three to five
seconds after motion stopped. The charging cable may have added drag, although
it had visible slack.

The verified 89-frame recording confirms that this was not a changing software
speed request. At seconds 11--14 the requested and executed wheel values stayed
near `0.09/0.09`, both lane boundaries remained visible, and the camera view
stopped advancing. Encoder changes were `3/2` ticks during second 10 and then
`0/0` for the following seconds. A mat seam was visible near the stopping area;
cable drag, track contact and low-power behaviour remain possible causes.

The bounded supervisor now requires fresh left and right encoder streams before
release. During motion it stops if a wheel has been continuously commanded at
least `0.07` for `0.75` seconds without any encoder progress, or if that encoder
stream becomes stale. A deliberately slow inside wheel below that threshold is
not treated as stalled as long as the commanded outside wheel continues moving.
Replay against this run detects the right-wheel stall at approximately 11.38
seconds, rather than leaving both motors energized through the 15-second limit.
The network-isolated suite passed 161 tests with one platform-specific skip,
including startup grace, intentional low inside-wheel commands, independent
left/right stalls, stale encoders and propagation through the live fault gate.
No wheel command was sent while implementing or verifying this safeguard.

### 2026-09-08: left curve-to-straight transition accepted

Run `20260908T191439Z-left-curve-cable-held-stall-watchdog-cf963d12` used the
scheduled-gain preset while the user held the charging cable with maximum slack.
The user observed perfect curve tracking. At the beginning of the straight the
robot trended right, corrected before crossing the white boundary, moved
smoothly to the lane centre, and stopped promptly in the centre. There was no
oscillation, stall or visibly inconsistent speed. The transition is accepted
for the present track and lighting, with the early straight correction retained
as a limitation to watch on other placements.

The verified evidence contains 88 frames and 150 motion-window status samples.
Both lane boundaries were detected in every status sample, the camera remained
fresh, no safety fault occurred, and encoders advanced by 826/899 ticks. The
encoder rate stayed nonzero throughout the motion. A fresh zero-feedback window
confirmed stopping, and normal control was restored afterward. This passes the
left curve-to-straight milestone; it does not establish right-bend behaviour.

### 2026-09-08: sharp-right preflight rejection diagnosed offline

Runs `20260908T192300Z-sharp-right-bend-to-straight-69d6fb59` and
`20260908T193104Z-sharp-right-bend-to-straight-retry-1f67cab3` aborted before
emergency-stop release. Neither establishes right-bend driving performance.
The earlier explanation that a straight-road restriction rejected the bend was
incorrect: that error text covered red-stop or missing lane estimates, not
curvature. Changing the wording did not resolve perception.

Offline replay of all 44 saved retry frames, with the reviewed yellow bounds
and temporal fallback enabled, returned `Lane lost: boundary gap exceeded`.
The white mask contained zero pixels in every frame. The view shows the yellow
divider and a distant white boundary across the upper/left image; the required
right-hand white border is absent from the analysed lower-right region. Widening
colour bounds cannot recover a border outside that region. No colour, steering,
speed, lane-width or motion-limit changes were made for this diagnosis. The next
step is a stationary placement check with duck2 aligned along its lane and both
boundaries visible, preferably nearer the entrance of the sharp bend.

Failed-start cleanup previously stopped its processes but skipped telemetry and
completion output. Consequently the laptop could not confirm a fresh zero window
and withheld automatic controller restoration. Earlier manual recovery restored
car-interface after temporary publishers exited, but did not obtain a recorded
fresh zero-feedback confirmation; that is not a verified stopping result.

The supervisor now preserves failed-start telemetry, its specific perception
reason, and a completion record after temporary controllers exit. It issues zero
again and requires a fresh eight-sample zero window before reporting stopping as
confirmed. Missing feedback remains unconfirmed and does not authorize automatic
restoration. A null release time explicitly distinguishes an aborted preflight
from motion. The latest camera status is rechecked rather than trusting only the
snapshot which ended the preflight wait. No robot connection or command occurred
while implementing this correction.

Verification: 79 focused supervisor, session, lane and readiness checks passed
inside the local ROS image with networking disabled. Tests cover both fresh zero
confirmation and absent feedback, raw status retention, and a null motion start.
Saved-image replay was also network-isolated. Live retry and physical stopping
remain deferred to a new Go after stationary placement verification.

### 2026-09-08: straight approach into the sharp right bend

Run `20260908T193805Z-straight-into-sharp-right-bca96dfd` released from a
complete two-boundary view approximately 10 cm before the bend. The user saw
duck2 move straight to the corner, begin turning right, and stop shortly
afterward. Recorded motion lasted 1.328 seconds. Both encoders advanced
(`78/65` ticks), the camera stayed current, and fresh executed-wheel feedback
confirmed the final zero. Normal control was restored with kinematics as the
only wheel publisher.

The synchronized status explains the stop. Both boundaries remained visible
for the first 0.85 seconds while the requested right correction increased. The
white boundary then left the image, yellow-only temporal fallback operated for
approximately 0.3 seconds, and the existing timeout stopped motion with `Lane
lost: boundary gap exceeded`. The user observation and recorded event therefore
agree: entry and turn direction worked, while the missing white edge ended the
test. This run does not establish completion of the right bend.

The bounded camera-guided preset now permits yellow-only tracking for at most
5 seconds after a measured two-boundary lane. It continues using the last
measured lane width and the current yellow-divider position. White-only tracking
retains the shorter 0.3-second limit and the white-boundary risk guard. Complete
lane loss, stale camera/status, red detection, encoder stall, competing wheel
publishers and watchdog expiry still stop the run. The ordinary node default
remains symmetric at 0.3 seconds; ordinary launchers are unchanged.

Network-isolated replay of every recorded motion frame stayed finite and within
the existing wheel and steering caps. The final saved yellow-only frame requested
a strong right correction (`0.15/0.016` in the replay); replay cannot establish
the unseen remainder of the corner. The focused suite passed 80 checks. After
rebuilding the local AMD64 image, the full network-isolated suite passed 197
checks with two platform-specific skips. The isolated ROS transport suite also
passed, including delivery of the separate yellow-only timeout,
camera timestamp and timeout gates, shutdown zero delivery, red-line latching,
command/heartbeat stopping, evidence recording and experimental-feature
isolation. A new physical run still requires a fresh Go.

### 2026-09-08: sharp right bend completed, inside-boundary contact

Run `20260908T195736Z-sharp-right-yellow-only-1p5-f35b091d` ran for 7.478
seconds and stopped on the real red line visible in the retained recording.
The user reported that the route and stop were otherwise perfect, but the right
wheel drove over the inside white boundary while turning. The right-bend lane
containment criterion therefore remains unmet; red-line stopping passed for
this approach by both user observation and recorded status.

Camera age remained between 0.035 and 0.136 seconds, both encoders advanced
(`546/439` ticks), a fresh zero-feedback window confirmed stopping, and normal
kinematics ownership was restored. Both lane boundaries remained detected in
all 74 motion-window status samples. This run did not exercise the new
yellow-only timeout.

The synchronized control record shows repeated maximum right-turn saturation:
the left/right request reached `0.15/0.00` while both boundaries were visible.
That is a stronger pivot than the previously accepted fixed curve command of
`0.15/0.03` and is consistent with cutting the inside boundary. This evidence
supports a focused right-turn authority or boundary-margin adjustment; it does
not support changing colour thresholds or extending missing-boundary time
again. No automatic retry is authorized.

### 2026-09-08: limit inside-wheel pivoting in the bounded preset

The bounded camera-guided preset now applies a `0.03` minimum to either wheel
while a finite lane estimate is actively requesting motion. This matches the
inside-wheel value in duck2's successful fixed-turn calibration and changes a
saturated turn to `0.15/0.03` or `0.03/0.15`. The floor is opt-in and defaults
to zero, so ordinary launchers retain their previous behavior. It is never
applied when the lane estimate is absent or non-finite, driving is disabled, or
the final publication gates require a stop.

Network-isolated replay of the complete right-bend recording replaced the zero
inside-wheel samples with `0.03`, preserved the `0.15` outer cap, produced no
missing-lane frames, and retained the straight-exit correction. Replay of the
accepted left curve stayed finite, continuous and within the same caps. The
full offline suite passed 198 checks with two platform-specific skips after a
test fixture was corrected to explicitly select the legacy single-gain mode it
claims to test. After rebuilding the AMD64 image, the isolated ROS suite passed
and confirmed transport of the active-wheel floor plus all existing stopping,
recording and feature-isolation checks. No physical retry occurred during this
change.

### 2026-09-09: watchdog startup warning no longer causes a false abort

The unplugged sharp-right attempt recorded as
`20260909T090141Z-sharp-right-020-003-unplugged-9603cebe` did not release the
emergency stop and produced no physical movement. Its stationary camera
preflight was valid and requested a right turn, but ROS printed an inbound TCP
handshake warning before the watchdog's `WATCHDOG_READY` line. The supervisor
previously inspected only the first output line and therefore treated that
unrelated warning as a watchdog failure. Cleanup published zero, normal control
was restored, and kinematics again became the sole normal wheel publisher.

The readiness check now reads complete output lines until it receives the exact
`WATCHDOG_READY` sentinel within the original absolute timeout. Earlier ROS log
lines are retained in any failure report. A missing sentinel, a lookalike line,
a closed output pipe, a child-process exit, or an elapsed timeout still aborts
before movement. The pipe is read on a bounded background reader so Python's
text buffering cannot hide an already received second line from `select`.

Verification passed 35 focused supervisor tests and 17 ground-session tests.
A real subprocess-pipe probe also accepted a warning followed by the exact
sentinel. A subsequent read-only live check confirmed key-based SSH, the ROS
master, the camera/kinematics/wheel nodes, and `/duck2/kinematics_node` as the
sole normal wheel publisher. No physical retry occurred during this fix.

### 2026-09-09: latched sharp-right retry and rolling relief

One latched-corner run reacquired both boundaries and continued until a red
stop after 12.57 seconds. Its immediate repeat stopped after 4.09 seconds when
the loaded outside wheel produced no encoder progress for more than 0.8
seconds while still commanded at `0.20`. The user's observation agreed that
turning paused; duck2 remained within the lane. Fresh camera data and final
zero feedback were confirmed, and normal kinematics ownership was restored.

The bounded profile now alternates 0.45 seconds at `0.20/0.00` with 0.25
seconds of rolling relief requesting `0.20/0.09`, without unlatching the sharp
corner. The original stall guard and all perception, ownership and watchdog
stops remain active. The focused ARM64 network-disabled run passed 65 tests,
and saved-frame replay exercised the pivot and relief phases. Physical success
remains pending a fresh supervised run.

### 2026-09-09: release the fixed right turn when white returns

The later continuous-turn trial lasted 6.59 seconds. The user observed a good
approach and turn followed by an overshoot that left duck2 facing the white
border. Status evidence shows both boundaries returned about 2.30 seconds into
the pivot, but the old code continued `0.20/0.00` while waiting for an already
centred lane. The lane error worsened and yellow subsequently left the image,
causing the recorded stop. Final zero feedback and normal-control restoration
passed.

In the bounded test mode, two consecutive white observations spanning 0.10
seconds, after at least one second of turning, now end the fixed pivot. Normal
camera-guided centring begins immediately in a distinct `reacquiring` phase.
Both boundaries must still be stably centred before the corner state clears,
and the ten-second corner deadline still covers the complete turn and
reacquisition. Existing camera, red-line, encoder, ownership, client and
watchdog stops are unchanged. The focused network-disabled ARM64 suite passed
162 tests. No physical retry was performed for this revision.

### 2026-09-09: pre-corner stationary-wheel stall

The first white-handoff retry produced motor sound but no visible movement and
stopped on `Left wheel stalled while commanded`. Both lane boundaries remained
visible and the sharp-corner state never activated. The ordinary controller
had saturated at about `0.20/0.00`; encoders advanced only six left and two
right ticks. Camera freshness, final zero feedback, evidence preservation and
normal-controller restoration passed.

The bounded preset now retains the `0.03` inner-wheel floor during ordinary
lane following and reserves `0.20/0.00` for the confirmed sharp-corner state.
Replay of the failed frames requests `0.20/0.03`. The 53 host-side supervisor
and cleanup tests and 162 network-disabled ARM64 focused tests passed. The
white-return handoff and all stopping safeguards remain active. No physical
retry was performed for this change.

### 2026-09-09: continuous corner stall and repeatable startup

The rolling-precorner run `20260909T111150Z-sharp-right-rolling-precorner-d9dd0a8f`
advanced both encoders (163/110 ticks) and entered the corner pivot, then
stopped after 4.95 seconds on `Left wheel stalled while commanded`. The user
observed forward travel, turning, and stopping. White remained absent at the
stop; the handoff could not run. Camera freshness, final zero feedback,
normal-control restoration and the saved 57-frame evidence bundle passed.

The next reviewed preset uses the user-requested 0.15/0.00 continuous pivot,
with a one-second approach after confirmed white loss. Stable white return
now releases the pivot without an additional minimum-turn delay. The rolling
floor during ordinary following, camera-guided centring, stall guard and all
deadlines remain. See [the current preset and limitations](RIGHT_BEND_RESTART.md).
Both pivot powers have previously stalled under load; this revision has no
physical passing result yet and is not claimed as a motor-stall cure.

The 167-test focused ARM64 suite passed with networking disabled. After adding
bounded SSH-error handling, all 22 Windows session tests passed, covering boot
failures, competing controllers, wrong keys, unavailable login, timeouts,
WSL rejection and failure cleanup. The new Windows `.cmd` entry point passed
its read-only check on duck2 without a password prompt or camera/wheel access.
No Git operations or physical retry occurred during this revision.

### 2026-09-09: distinguish loaded stall from white-border expiry

Run `20260909T112801Z-sharp-right-white-return-eb354c7a` used the requested
0.15/0.00 pivot. The user saw straight movement, a one-second approach, a brief
right turn, then stopping. Fresh encoder counts stopped advancing before the
supervisor issued zero at 4.04 seconds. White was still absent, but its timeout
was not the reason for this stop. Final zero feedback, ownership restoration
and the preserved 54-frame evidence bundle passed.

A separate conflict was corrected: while a confirmed pivot retains current
yellow and fresh camera/status, expiry of the ordinary lane-width estimate
does not shorten the ten-second corner deadline. All other lane failures,
stalls, stale data, ownership conflicts, red stops and the independent watchdog
retain their checks. 94 focused network-disabled ROS/OpenCV tests passed,
including synthetic width expiry followed by boundary reacquisition. This is
not evidence of a cure for the physical pivot stall. Further movement requires
a fresh Go after reviewing mechanical freedom and power.

### 2026-09-09: isolated stationary-inner-wheel pivot

Run `20260909T114923Z-ground-right-pivot-8b63df4a` bypassed lane detection and
requested left `0.15`, right `0.00` for two seconds on the ground. The command
remained present for the full window and no software early-stop reason was
reported. The user observed an initial right turn followed by a quick physical
stop, with the charging cable fully slack and no obstruction. Encoder deltas
were only 6 left and 1 right. Final zero feedback, evidence verification and
restoration of `/duck2/kinematics_node` passed.

This demonstrates that the short sharp-right failures are not caused solely by
camera loss or the corner timeout. A two-second rolling-right diagnostic using
left `0.15`, right `0.03` is now available through `-GroundRollingRight`. Its
inner-wheel request is intended to reduce stationary-wheel scrub while keeping
the previously reviewed outer-wheel power.

Run `20260909T115656Z-ground-rolling-right-19f2530d` completed the full window
without an early stop and produced encoder deltas of 132 left and 121 right.
The user observed mostly straight travel with a slight right turn. Final zero,
evidence verification and normal-control restoration passed. This removes the
stall but is not strong enough for the sharp corner. A stronger isolated
`0.20/0.03` profile is now available as `-GroundStrongRollingRight`, still
capped at two seconds. The 67 focused tests pass. No production corner change
or physical result for the stronger profile is claimed yet.

### 2026-09-09: accepted sharp-right bend and intersection preparation

Run `20260909T123050Z-sharp-right-restored-020-272d6ab6` used the bounded
camera-guided preset with its restored `0.20/0.00` confirmed-corner command.
The user observed a perfect sharp-right bend. The controller subsequently
latched a red-line stop after 7.72 seconds; the robot stopped approximately
15–20 cm before the line by the user's estimate. Camera age stayed between
0.037 and 0.137 seconds, encoder deltas were 615 left and 514 right, the final
feedback window was zero, and `/duck2/kinematics_node` regained normal wheel
ownership. This is an accepted result for that bend, not junction calibration.

Red detection now reports the nearest valid line's box, image-bottom fraction,
width, area and trigger state. A separate
`~red_stop_trigger_bottom_fraction` parameter allows stationary calibration of
a closer stop while retaining the existing default. Line disappearance is
never used as permission to cross. Use `-InspectRedLine` to capture a
stationary sample without releasing the emergency stop; determine the
provisional 5–10 cm trigger from measured placements before using it in motion.

The route controller now has separate straight, left and right junction
durations, speeds and biases. During an authorized `crossing` or `reacquiring`
phase, fresh camera frames with no yellow or white borders continue the bounded
selected maneuver. When both correctly ordered outgoing borders appear, the
controller changes to low-speed visual alignment and requires 0.3 seconds of
stable lane evidence before advancing the route. Missing borders during normal
lane following still request zero. Camera rejection, timeouts, route faults,
manual Stop, wheel stalls, publisher conflicts and the independent watchdog
still stop the test.

The dedicated `-JunctionTurn straight|left|right` supervisor profile configures
one valid three-node test route through the existing command interface. It
treats the mandatory red-line dwell and unmarked crossing as expected states,
records junction phase/progress/deadline evidence, and stops one second after
confirmed outgoing-lane reacquisition. The initial maneuver values are
provisional references and have not moved duck2 through an intersection. The
focused lane, navigation, readiness, supervisor and session suite passed 150
tests after these changes. The broader offline suite passed 282 tests with four
environment-dependent skips. The isolated ROS transport suite then passed in a
disposable ARM64 container with `--network none` and no hardware devices. It
confirmed continuous nonzero requests through an authorized unmarked crossing,
stable 0.3-second outgoing-lane reacquisition, route advancement, camera and
heartbeat stopping, and shutdown zero delivery. Physical work must begin with
stationary red-line calibration and one straight crossing.
