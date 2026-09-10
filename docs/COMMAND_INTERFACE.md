# Runtime command interface

The app uses gateway `/status`, `/heartbeat` and `/command` endpoints over local
SSH. JSON commands and status use `/duck2/lane_follower/command` and
`/duck2/lane_follower/status`. Camera and stamped-wheel interfaces are unchanged.

Commands carry unique `id`, `client_id` and Unix `issued_at`. Normal requests
must be at most five seconds old or two seconds in the future; Stop ignores age.
Recent identifiers are deduplicated. Acceptance is not proof of physical motion.

## Start and Stop

`set_route` requires a supported map, legal directed route, matching start and
destination, `position_confirmed: true`, and current `expected_control_epoch`.
The app verifies sole wheel ownership, sets the route, then sends `continue`
with the same epoch. An intervening Stop invalidates Start.

`stop` latches zero, advances the epoch and ends a live session. Delayed requests
cannot undo it. Camera/heartbeat loss and faults also prevent unattended resume.

## Managed live chat

`set_route` carries `managed_session: true` and a unique `run_id`. Its initial
route contains the two starting-lane junctions; the laptop owns future turns.
Status advertises `live_session.version: 1`, `pause_mode: immediate`, run id,
profile, pause state and red-stop deadline. Live actions require that run id and
the current `expected_control_epoch`:

- `junction_instruction`: left/right/straight `value`, current `approach` and
  `expected_route_index`. The robot validates exit, dwell and phase. Its accepted
  id appears in `live_session.instruction_id`.
- `pause`: optional numeric `seconds` in `(0, 3600]`. It publishes zero before
  acknowledgment and advances the epoch. Without duration, Continue is required
  within 30 seconds.
- `resume`: resume a pause without bypassing red authorization or faults.
- `straight_profile`: slow/normal/fast, applied on confirmed straight roads.

`junction_last_result` reports outcome, turn and index; `current_approach` gives
the outgoing lane. Only matching completion advances the laptop position.
An authorized unmarked crossing still has camera freshness and maneuver limits.
Empty-queue red stops retain zero and allow 60 seconds for an instruction.

Managed mode rejects legacy global speed/turn commands. The chatbot word `stop`
means pause; `quit`, `stop the run` and STOP DUCK2 send terminal `stop`. Removed
route options from an old client are rejected before route setup.
