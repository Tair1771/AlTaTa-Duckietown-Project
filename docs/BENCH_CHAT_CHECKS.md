# Track-free live-chat checks — handoff prepared 2026-09-10

## Preparation facts

Windows strict-key SSH authenticated after the laptop/robot reboot without a
password prompt. The agent is running with Automatic startup. The ARM64 base
image digest, Ubuntu 18.04.6, ROS Noetic and camera/wheel message fingerprints
match `config/duck2.json`. Camera and wheel-driver nodes are registered.
This is metadata evidence, not a new live-frame or physical-motion test.

`car-interface` and both previous companion containers were stopped. No normal
wheel-command publisher was registered. The dashboard was unhealthy, while the
ROS and hardware-interface containers were healthy. Do not automatically restore
driving services for these bench checks. Dashboard health is separate from the
SSH/ROS path used here.

Fixed two reboot issues locally: the quick check now distinguishes successful
SSH from missing normal ownership, and stationary preview setup recreates a
stopped preview from current source instead of querying/restarting its stale
container. Runtime cache comparison no longer changes simply because uptime
increases. Connection scripts retain strict SSH identity checks.

Docker Desktop's stale-socket startup failure recurred. Renaming only its runtime
socket directories and restarting Desktop restored local testing; images and
volumes were preserved. The onboard command below does not need Docker Desktop.

## Verified routine — execute with Terra Medium

Keep duck2 stationary on a stable table, camera uncovered, with power available.
No track, upside-down placement or physical Go is needed: these checks cannot
command the installed wheel driver. Do not open Driving Mode or start a real
route. If any result differs, stop and report it rather than retrying or changing
motion settings. Return to Sol High for substantive diagnosis.

In Windows PowerShell, from the repository root:

```powershell
$benchPython = "$env:USERPROFILE\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
& $benchPython tools/inspect_robot_setup.py --connect --save-summary
```

Exit 3 with missing `car-interface`/kinematics and no normal publisher is the
known reboot state; it means normal driving is not prepared. It does not block
the following isolated checks. Authentication/host identity failures or a running
driving controller require diagnosis first. Do not bypass identity checks.

### Stationary camera and app

```powershell
& $benchPython tools/connect_companion.py
& $benchPython tools/check_stationary_camera.py
.\laptop\Start-Duck2Companion.cmd
```

Preview runs with `drive_enabled:=false` and its wheel output redirected to a
diagnostic topic. It does not stop/restart the installed containers. It refuses
to coexist with the companion driving service. Its gateway uses loopback SSH
ports 8765/8766. The camera check decodes Normal, Mask and Overlay samples and
checks advancing capture timestamps and sampled frame ages below 0.5 seconds;
it stores metadata outside the repository, not images. This is a short sampled
check, not proof of sustained camera delivery.

In the app, select **Connect to duck2**, **Start viewing**, and check all three
views. Resize the window and check that the whole map and Stop remain accessible.
Do not press Start selected route: the live camera preview deliberately refuses
driving. Desk masking is only a rendering check; do not adjust course colours.
The chatbot's execution tests happen in the separate synthetic session below.

### Synthetic onboard chatbot/ROS tests

```powershell
& $benchPython tools/isolated_bench_check.py --execute --target robot
```

The default without `--execute` prints a plan and does not connect. Execution:

- Copies a small, hashed source/test archive and uses the already installed
  digest-pinned ARM64 ROS base. No download or installed software replacement.
- Creates a unique container with network `none`, no device mappings, no host
  mounts, no exposed ports, no restart policy, dropped capabilities and bounded
  CPU/memory/process usage. Inspects this configuration before starting.
- Builds the package inside that disposable container and runs a separate
  loopback ROS master. Synthetic camera and wheel topics never reach the real
  master or hardware drivers. The fixed timeout is 480 seconds plus ten seconds
  for termination even if laptop connectivity is interrupted.
- Runs map/queue, pause/timeout, speed-isolation and controller regression tests,
  then actual isolated ROS transport checks. Thirty-second session deadlines are
  covered with a controlled clock in controller tests, not 30-second physical waits.
- Saves source hashes, configuration, exit status and logs under
  `%LOCALAPPDATA%\Duck2\bench-checks`. Removes only its own temporary container
  and exact staging archive/directory; installed containers remain as found.

Require `PASS` and a successful `result.json`; report any failure without an
automatic retry. If SSH disappears during cleanup, the container has its own
deadline. Inspect the unique name recorded in the report before resuming cleanup;
never delete by broad pattern. Compare before/after installed container IDs.

After visual checks, close the app and stop only `duck2-companion-preview` using
`ssh duck2 docker stop -t 10 duck2-companion-preview`. Close only the SSH process
forwarding ports 8765/8766 if no other app needs it. Do not start normal driving
control as cleanup. Finish with a read-only wheel-topic ownership inspection.

## Preparation validation

The new runner passed locally in the pinned AMD64 base with the same isolation
and bootstrap: 60 session/controller tests and the complete ROS transport script,
including managed Start, straight profile, timed pause/resume, heartbeat loss and
Stop. It exited successfully and removed its temporary container. Local evidence:
`%LOCALAPPDATA%\Duck2\bench-checks\20260910T013237Z-b7eb259e`.
Focused connection/isolation/reboot, camera-check and native UI checks: 27 passed,
one skipped. Camera-check tests use local fake responses and real image decoding;
they do not subscribe to duck2.

## Execution result — 2026-09-10

Executed with duck2 stationary after the reboot. Strict-key Windows SSH reached
the robot and the read-only check again confirmed the expected camera/wheel
message types. Normal control remained intentionally unprepared: `car-interface`
and kinematics were stopped, and the real wheel topic had no publishers.

The temporary preview passed Normal, Mask and Overlay image decoding. Nine sampled
frames had advancing timestamps and frame age at or below 0.5 seconds. Metadata,
not images, is stored in
`%LOCALAPPDATA%\Duck2\bench-checks\20260910T014034Z-camera.json`.

The ARM64 isolated run passed on duck2. Its report is
`%LOCALAPPDATA%\Duck2\bench-checks\20260910T014036Z-b29afed9\result.json`:
exit code 0, not OOM-killed, no physical tests. The log includes
`BENCH_SYNTHETIC_PASS`, 60 session/controller tests, and the isolated ROS
transport checks. Before execution the container was inspected as network `none`,
dropped all capabilities, with no devices or mounts. It used a separate loopback
ROS master and synthetic camera/wheel topics. The test container removed itself
through the runner's exact-name cleanup.

The preview was stopped after the check, and its loopback SSH tunnel was closed.
The real wheel topic was rechecked afterward: no publishers. No normal control,
wheel command, installed service, calibration, Git state or physical movement was
changed during this session.

Track containment, the three physical straight speeds, curve entry, and junction
accuracy remain track tests; these bench checks cannot establish them.

## Interactive app simulation — verified 2026-09-10

The real Windows Tk companion was connected to the actual controller and HTTP
gateways in a separate onboard container. That container had network `none`,
no devices/mounts/host ports, all capabilities dropped and a 20-minute deadline.
A single strict-key SSH stdio relay carried only allowlisted HTTP requests into
its loopback services. Windows simulation ports are 18765/18766, separate from
the physical companion ports 8765/8766. No physical driving controller was started.

Eleven integrated checks passed through real Tk callbacks, the HTTP relay and
onboard ROS: camera rendering; typed queue replace/append and invalid T-junction
rejection; straight-only speed changes and curve-deferred timed pause; typed
stop/continue; one red-line instruction and reported lane advancement; quit;
empty red-stop expiry using an actual 30-second wait; indefinite-pause expiry
using an actual 30-second wait; camera-loss zero requests; heartbeat-loss stopping;
and application-close Stop. Results are in
`%LOCALAPPDATA%\Duck2\bench-checks\20260910T035651-interactive-ui.json`.
Isolation/source evidence is in
`%LOCALAPPDATA%\Duck2\bench-checks\20260910T015308Z-2e0c6f97`.
The initial report lists all eleven passing checks and physical_motion=false;
the runner exited 0. Future reports additionally include an explicit passed field.
Fifteen focused UI and bench-tool regression tests also passed.

This simulation provides deliberately simple synthetic road scenes. It is an
integration test of commands, state transitions and software wheel requests,
not a physics model or validation of turn timing, road geometry, real encoders
or track containment. It uses test-only node parameters; the production launcher
and controller turning logic were not edited.

### Repeat or interact manually

From Windows PowerShell in the repository root, run:

```powershell
$benchPython = "$env:USERPROFILE\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
& $benchPython tools/interactive_bench.py
```

Wait for BENCH READY. In another window, open
`laptop/Start-BenchCompanion.cmd`. The window title and banner must say SIMULATION.
The simulation connection fields are read-only. Connect, select A->D for both
start and destination, confirm the **simulated** placement, then Start. Choose
Camera & status / Live chat. The scene buttons provide Straight, Curve, Red line,
No lines and Camera loss. With red selected, the scene automatically becomes
unmarked during crossing and then presents a synthetic outgoing lane.

For the scripted UI checklist, run `tools/exercise_interactive_bench.py` with
the same Python instead. It opens and closes its own simulation window. Do not
operate a second app client during that checklist, since heartbeat ownership is
deliberately exclusive. To rerun after a fault, use Stop and confirm a new
simulated start; reconnecting alone does not resume a run.

The relay ends after approximately 20 minutes and removes its own container and
temporary archive. Closing the app requests Stop; no heartbeat stops simulated
motion independently. The deadline also terminates the container if SSH fails.
After this recorded session the relay exited successfully, ports 18765/18766
were closed, the temporary container was absent, and the real wheel topic had
no publishers. No Git operations or physical movement occurred.
