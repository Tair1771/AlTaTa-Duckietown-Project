# Live chatbot

Live chat is enabled by default in the normal companion. It uses a deterministic
local grammar, not a trained model or cloud API. Route state provides context
for the next junction, appended turns and relative speed changes. Unsupported
wording asks for clarification; arbitrary typos are not guessed.

| Message | Effect |
| --- | --- |
| `stop for 7s` / `pause for 7 seconds` | Request zero immediately, pause seven seconds, then resume if healthy |
| `stop` / `pause` | Pause now; Continue required within 30 seconds |
| `continue` / `resume` | Resume without bypassing a red stop |
| `go straight at the next junction` | Replace future turns with straight |
| `go left` / `go right` | Replace future turns with that legal exit |
| `next left then right` | Replace queue with left, right |
| `left after that` | Append left |
| `clear turns` | Empty queue; wait at the next red line |
| `slow speed` / `normal speed` / `fast speed` | Choose a confirmed-straight speed profile |
| `speed up` / `slow down` | Move one profile higher/lower |
| `status` / `where are we?` | Show reported lane and future turns |
| `quit` / `stop the run` / STOP DUCK2 | End session without automatic resume |

Capitalization, common punctuation and supported polite phrasing are normalized.
Timed pauses accept positive numeric durations up to 3600 seconds. Enter sends;
Shift+Enter adds a line. Acknowledgment confirms acceptance, not physical success.

## Queue and position

The map supplies the initial queue. Changes are validated against the directed
map before replacing it. Straight at a T-junction without that exit is rejected.
An accepted crossing cannot be edited midway; new requests affect later turns.
An override replaces the planned future queue rather than silently retaining
the original destination route.

Each red stop has a two-second dwell. One instruction is bound to the current
run, lane and junction index. With no queued turn, duck2 asks for a direction and
ends after 60 seconds at that red line. Invalid commands do not reset the clock.
Repeated acknowledgments cannot consume two turns. Reported position advances
only after the matching outgoing lane is established. Repositioning requires a
new confirmed start.

Live chat waits for instructions when its queue ends at the destination. Uncheck
Live chat before Start for map-only operation ending at that red line.

## Pause and speed behavior

Pause publishes zero in the robot's command callback before acknowledgment.
Its monotonic clock measures duration; network delivery and braking still take
time. The queue and current maneuver survive a pause, while camera, heartbeat
and watchdog health remain monitored. Paused time does not consume maneuver
time. Stop or a fault prevents automatic resume. Repeating pause cannot extend it.

Straight profiles request **0.09 / 0.10 / 0.11**, normal by default. These are
normalized wheel commands, not metres per second. They apply only to confirmed
straight roads; curve and junction settings retain their own profiles. Speed
increases are rejected without current straight-road evidence.
