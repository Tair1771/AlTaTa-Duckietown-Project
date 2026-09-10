# Duck2 usage guide

This guide separates the apps that are safe to use anywhere from commands that
require duck2 to be observed on the course. None of the Windows offline apps
need an API key.

For stationary camera checks and synthetic onboard chatbot tests without a
track, use [BENCH_CHAT_CHECKS.md](BENCH_CHAT_CHECKS.md). This routine keeps its
synthetic ROS session isolated from hardware and does not prepare driving mode.

## What to start

| Goal | Windows command or file | Connects to duck2? | Can move duck2? |
| --- | --- | --- | --- |
| Understand plain-English requests | `laptop/Start-OfflineChat.cmd` | No | No |
| Preview basic high-level commands | `laptop/Start-CommandPreview.cmd` | No | No |
| Plan and run a continuous route, view the camera and Stop | `laptop/Start-Duck2Companion.cmd` | Yes, when connected | Yes |
| Check robot connection and normal ROS ownership | `tools/Start-Duck2-Check.cmd` | Yes, read-only | No |
| Inspect a red line while stationary | `tools/Start-Duck2-GroundTest.cmd -InspectRedLine` | Yes | No |
| Run a supervised test | `tools/Start-Duck2-GroundTest.cmd ... -Go` | Yes | Yes |

Run the `.cmd` files by double-clicking them, or open Windows PowerShell in the
repository root and run the equivalent command with `./`.

## Offline chatbot and command preview

The two offline chat windows use only Python's standard library and Tk. Start
them from the repository root:

```powershell
py -3 laptop/offline_chat.py
py -3 laptop/command_preview_chat.py
```

The first explains what it understood. The second can make local drafts for
`stop`, `speed_up`, `slow_down`, and the next `left`, `right`, or `straight`
turn. **Record preview** saves only an in-memory fake-receiver record. Neither
window makes a network request or sends a ROS command.

## Combined companion and continuous route

Start the companion with:

```powershell
py -3 laptop/duck2_companion.py
```

The **Map & route** tab asks for a directed starting lane and a red-line
destination. It calculates a right-lane-only A* route with the fewest junction
crossings. Local live chat is available in the Camera & status panel; see
[LIVE_CHAT.md](LIVE_CHAT.md) for queue edits, straight-only speed profiles and
pause behavior. The app can send a confirmed route
to the `lane-continuous` controller; it sends no wheel-level values.

The **Camera & status** tab keeps the large camera view beside robot status,
connection, route start and **STOP DUCK2**. Normal, Mask and Overlay remain
read-only views. Closing the app requests Stop; loss of its heartbeat also
stops control.

### Start a continuous session

Keep duck2 stationary on the selected right lane, outside an intersection.

1. Verify connectivity with `tools/Start-Duck2-Check.cmd`.
2. Run `tools/Start-Duck2-DrivingMode.cmd` from Windows. It uploads the current
   Python sources, uses the pinned installed ARM64 runtime, stops the stationary
   preview and normal `car-interface`, then starts a temporary driving container.
   It preserves all `lane-continuous.sh` parameters and verifies exclusive wheel
   ownership, `awaiting_route`, manual stop and zero output. A previous running
   driving container is replaced only after those stopped-state checks pass, so
   revised source cannot be hidden by an old container. It reuses or creates the
   SSH tunnel. It then releases the driver's prior test emergency-stop latch
   only while the controller is manually stopped awaiting a route, requested and
   executed wheel values are zero, and there is no new emergency-stop request or
   unknown stop publisher. The installed HTTP API emergency-stop control stays available.
   No route or Continue command is sent. Docker Desktop and
   `dts devel build` are not needed for this source-mounted session.
3. Open `laptop/Start-Duck2Companion.cmd`. On **Camera & status**, press
   **Connect to duck2** and **Start viewing**. Select the directed starting
   lane and destination red line on the map. Confirm the physical placement,
   then press **Start selected route** while watching duck2.

The app refuses to start if the lane follower is not the only wheel-command
publisher. Stop invalidates a queued or partly completed Start transaction, and
the controller's control epoch rejects a delayed Continue after Stop. It sends a
heartbeat every 0.5 seconds. Ordinary lane following
handles straights and smooth curves. A recently complete lane followed by loss
of the right white edge, retained yellow divider and a consistent right-turn
error activates the tested sharp-right state machine. Red lines select the next
A* route turn. In map-only mode the selected destination red line ends the run.
With **Live chat** enabled, the laptop sends one turn at each red stop and asks
for another when the queue ends; the robot holds there for up to 30 seconds.

Use **STOP DUCK2** before ending. Stop the temporary container before restoring
normal control, then close the SSH tunnel:

```powershell
ssh duck2 docker stop -t 10 duck2-companion-driving
ssh duck2 docker rm duck2-companion-driving
ssh duck2 docker start car-interface
tools\Start-Duck2-Check.cmd
```

The driving container has no automatic restart policy. A new preparation starts
stopped. If an earlier container is running, preparation first requires it to be
manually stopped, awaiting a route, at zero output, and under exclusive control;
it then replaces that container with the current local source. If preparation
fails after suspending normal control, it leaves `car-interface` stopped and
reports the failure rather than resuming control unexpectedly.

Safety stops remain active for stale/lost camera data, lane loss outside an
authorized maneuver, encoder stalls, loss of the desktop heartbeat, competing
wheel publishers and maneuver deadlines. An independent continuous-session
watchdog also watches requested and executed wheels, lane status, encoder
progress and publisher ownership. It asserts the driver's emergency stop and
ends the temporary process group on a fault. A fault requires physical placement
confirmation and a new route; it never resumes automatically.

The camera Normal, Mask and Overlay views use the same target, yellow/white HSV
limits, temporal lane-width fallback and boundary-risk settings as the continuous
controller. Preview diagnostics therefore describe the active detector preset;
they still do not prove physical steering or stopping.

## Read-only camera view

The camera app expects `http://127.0.0.1:8766`. Its service must be running on
duck2 first. From a prepared Duckietown development session, launch only the
read-only service:

```powershell
dts devel run -R duck2 -L lane-camera-view
```

In a second Windows PowerShell, create the local-only tunnel and leave that
window open while viewing:

```powershell
ssh -N -L 8766:127.0.0.1:8766 duck2
```

Then start the companion, open **Camera & status**, keep the default local
address, and select **Start viewing**. Choose **Normal**, **Mask**, or
**Overlay**. Select **Stop** before closing the tunnel. The service subscribes
to the compressed camera topic and exposes no command endpoint. Do not run a
driving launcher merely to use the camera.

If the app reports that the camera connection is unavailable, first run
`tools/Start-Duck2-Check.cmd`; confirm the robot has booted and the service is
still running. Never replace the loopback address with duck2's hotspot IP.

## Connection check

Turn on the laptop hotspot, then duck2, and allow two to five minutes for the
robot to boot and join the hotspot. Run:

```powershell
tools/Start-Duck2-Check.cmd
```

The check verifies the saved SSH identity and key login, ROS master, essential
containers, and normal wheel publisher. It does not view images, publish wheels
or call motor services. If the key agent was restarted, unlock the dedicated
key with `ssh-add $env:USERPROFILE\.ssh\duck2_ed25519` and enter only the key
passphrase.

## Track-test commands

Every movement test requires duck2 upright, on the track, observed by a person,
with the cable slack and clear of the wheels. The bot must be stationary before
you add `-Go`.

The following is the earlier calibration/test workflow, retained for deliberate
recalibration. Do not overwrite the accepted launcher preset with these example
values. Current individual crossings have been exercised; continuous routes
remain to be physically verified. First, collect stationary samples at 15 cm, 10 cm and 5 cm
front-to-line positions. This keeps the emergency stop latched and does not
move duck2:

```powershell
.\tools\Start-Duck2-GroundTest.cmd -Label red-line-10cm -InspectRedLine
```

After choosing the provisional red threshold from those samples, the next
example moving test is one straight four-way crossing:

```powershell
.\tools\Start-Duck2-GroundTest.cmd -Label junction-straight -Duration 15 `
  -JunctionTurn straight -RedStopTriggerBottomFraction 0.80 -Go
```

`0.80` is only an example until the stationary samples establish the correct
threshold. A junction test permits missing lane borders only during its bounded
authorized crossing and reacquisition phases. It still stops on stale camera
data, controller faults, wheel stalls, ownership conflicts, manual Stop or its
deadline.

Details of all status fields, command semantics and safety gates are in
[COMMAND_INTERFACE.md](COMMAND_INTERFACE.md). Measured history and remaining
physical work are in [TESTING.md](TESTING.md).
