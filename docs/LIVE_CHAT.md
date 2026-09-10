# Live chat in the companion

Open `laptop/Start-Duck2Companion.cmd`. Live chat runs locally with no API key.
It supports the existing lane following, junction choices, Stop and pause
controls; it does not enable reverse or obstacle passing.

## Starting

1. Prepare the updated source with `tools/Start-Duck2-DrivingMode.cmd` while
   duck2 is stationary. This uses the existing source-mounted runtime and does
   not require a new Docker Desktop build. It changes robot controller ownership,
   so follow the preparation/cleanup instructions in [USAGE.md](USAGE.md).
2. Open the companion, connect, and select the directed starting lane and a
   destination red line. The map supplies an initial queue of junction turns.
   To start with no planned turns, select the same directed lane for both.
3. Leave **Live chat (ask when queue ends)** checked. Confirm placement and
   press **Start selected route** while observing duck2.
4. Open **Camera & status → Live chat** beside the camera. The panel reports
   the lane, future queue, speed profile, pause state and red-stop countdown.

The app refuses a live-chat Start if the controller does not advertise the
new session protocol. Reopen the app and prepare the updated robot source;
an already running window/container does not reload changed Python files.

Unchecking Live chat before Start retains map-only operation, which ends at
the destination red line. In live-chat mode, that line instead prompts for
another instruction and ends the run after 30 seconds without departure.

## Supported messages

| Message | Result |
| --- | --- |
| `The next turn should be right` | Replace all future junction turns with right |
| `Left after that one` | Append left after the current future queue |
| `Next left then right` | Replace the queue with left, right |
| `Clear turns` | Empty the future queue; wait at the next red line |
| `Status` / `Where are we?` | Show the reported lane and queue |
| `Slow speed`, `normal speed`, `fast speed` | Choose one straight-road profile |
| `Speed up` / `slow down` | Move one profile higher/lower |
| `Pause for 5 seconds` | Pause on a confirmed straight; preserve queue; resume after five seconds |
| `Stop` / `pause` | Pause on a confirmed straight; end run if not continued within 30 seconds |
| `Continue` / `resume` | Resume or cancel a pending pause; do not bypass a red stop |
| `Quit`, `end run`, `stop the run` | End the run immediately through Stop |

Press Enter to send; Shift+Enter adds a line. Wording outside the supported
grammar asks for clarification without sending a movement command. Negated,
conditional, reverse and obstacle requests are not executed. Ambiguous stop
wording is treated as pause rather than ending the run. An invalid timed
request is clarified rather than silently converted to an indefinite pause.

## Queue and position rules

The queue lives on the laptop. Each proposed sequence is checked against the
directed map before replacing anything. Straight ahead at a T-junction without
a straight exit is rejected. A turn already accepted by the robot cannot be
edited midway through the crossing; edits affect subsequent junctions.

At a red line the robot holds its normal dwell, then accepts one validated
instruction for that exact run, lane and junction index. Empty queue prompts
**Where should I go next?** The 30-second limit starts when the robot arrives
at the red stop, including the dwell. Chatting or sending an invalid turn does
not restart this clock. No instruction causes an indefinite unmarked crossing;
all existing crossing deadlines remain in effect.

The robot reports outgoing-lane reacquisition and its turn/index. The laptop
advances its directed-lane estimate only after matching this report to the
instruction it sent. This is route-history tracking, not global localization:
physically moving the bot requires a new confirmed starting position. Lost
acknowledgments are reconciled from status or retried with the same identifier,
so a retry cannot consume a second turn. Unexpected run/position reports
suspend queue delivery. Stop remains available.

## Straight-only changes

The straight-road profiles request centre speeds of **0.09 / 0.10 / 0.11**
(slow / normal / fast). Normal is the default. These are normalized wheel
requests, not metres per second. The profile path caps either wheel at 0.12
and retains the controller's acceleration limits. They are conservative
software candidates, not a new measured guarantee of lane containment.

Profiles apply only after stable, ordered yellow/white boundaries with near
support, small image heading/lateral error and small steering demand indicate
a straight. Missing/curved/uncertain evidence immediately uses the existing
controller speed. Speed increases are rejected outside a confirmed straight.
Selecting a lower profile during a turn changes the future straight setting
only. Curve, sharp-bend, junction-entry, pivot and reacquisition profiles retain
their existing calibrated values and steering logic. Legacy global speed-scale
commands are rejected during a live-chat run.

A pause requested in a curve, bend, or intersection remains pending until a
confirmed straight. Its timer starts when zero output begins, not when the
message was sent. Timed pauses resume only with a healthy connection and fresh
camera. Indefinite pauses expire after 30 seconds without Continue. Future
turns survive either pause. Safety stops and **STOP DUCK2** are immediate in
every phase; they never wait for a straight. Stop or a fault cannot be undone
by a late Continue or timed resume. Restart requires placement confirmation.

## Verification and limits

`tests/test_live_chat.py` exercises parsing, directed-map validation, queue
replacement/append, lost acknowledgments, turn reporting and deadlines.
`tests/test_live_navigation.py` uses the actual controller with mocked ROS to
check the wheel gate, deferred pause/resume, red-stop waiting, profile isolation
and Stop priority. Native Tk tests exercise the chat panel and responsive map.

The final native suite passed 407 tests with one platform/dependency skip.
Both AMD64 and ARM64 images built, and the complete isolated Noetic transport
script passed with networking disabled, including a managed-session
Start/profile/pause/resume/Stop sequence. See [TESTING.md](TESTING.md) for the
dated results and the distinction between local ROS evidence and physical tests.

| Requirement | Verification |
| --- | --- |
| Replace/append future turns and reject unavailable exits | Parser and directed-map tests cover every approach; invalid edits leave the queue intact |
| Track reported completion without duplicate turns | Lost-acknowledgment, repeated-ID and mismatched-run/index tests; laptop/controller two-junction sequence |
| Wait at an empty red stop and end after 30 seconds from arrival | Controller tests cover dwell, late first tick and disabled automatic departure |
| Three straight profiles, normal default, unchanged turns | Synthetic straight/curve images, wheel cap and profile tests; all three junction choices checked across every speed selection |
| Deferred timed/indefinite pauses preserve turns | Curve/crossing deferral, actual-zero timer start, resume and 30-second expiry tests |
| Stop/quit priority and connection failures | Parser, stale-epoch command rejection, UI Stop during command-lock contention, timed-resume health checks and isolated ROS heartbeat/shutdown checks |
| Usable app with no model API | Native Tk interaction, map fitting at multiple window sizes/scalings, chat input and clean-close tests |

These offline checks do not prove the new straight classifier or speed profiles
under tomorrow's lighting. Initial live verification must remain supervised.
No live-chat movement was performed as part of this implementation.
