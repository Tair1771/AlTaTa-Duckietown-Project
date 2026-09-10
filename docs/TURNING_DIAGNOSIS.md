# Sharp-right diagnosis — 2026-09-09

Historical motor/turn investigation. Later trial outcomes are in [TESTING](TESTING.md).
Current operating modes are listed in [STARTUP](STARTUP.md).

The equal-command trial travelled straight by the user's observation, with
216/215 encoder ticks. Earlier statements that this proves equal motor power,
or that the weak turns prove a defective/weaker left motor, were too strong.
It establishes matched rotation in that trial. No damage, cable resistance or
obstruction is assumed in this diagnosis.

## Evidence compared

| Trial | Left/right command | Encoder evidence | Physical observation |
| --- | --- | --- | --- |
| Continuous corner today | sustained 0.20/0.00 after approach | roughly 20–24 left ticks per half-second, right usually zero during pivot | Sharp right turn, followed by overshoot before white-handoff fix |
| Later corner | 0.15/0.00 after approach | left progress dwindled to zero while command remained | Brief turn, then stopped |
| Isolated pivot | 0.15/0.00 | total 6/1 | Brief turn, then stopped |
| Rolling right | 0.15/0.03 | total 132/121 | Mostly straight, slight right |
| Strong rolling right | 0.20/0.03 | total 139/131 | Straight with roughly 2–3 degrees right |
| Equal wheels | 0.15/0.15 | total 216/215 | Straight as expected |

Source run identifiers: `duck2-ground-export-sharp-right-continuous-20260909`,
`20260909T112801Z-sharp-right-white-return-eb354c7a`,
`20260909T114923Z-ground-right-pivot-8b63df4a`,
`20260909T115656Z-ground-rolling-right-19f2530d`,
`20260909T120118Z-ground-strong-rolling-right-c4a8b670`, and
`20260909T121452Z-ground-equal-wheels-retry-da5224fc`.
Raw evidence stays outside the repository. The offline comparison can be rerun
with `tools/analyse_wheel_evidence.py <evidence-directory> [more-directories]`.

## Verified setup and limits of the evidence

- Installed configuration is DB21J/HATv3. Motor enable PWM channels are 8 and
  13. Left direction uses channels 10/9; right direction uses GPIO 33/31.
- Encoder channels are different GPIOs (18/19), both configured at resolution
  135 and publish frequency 30 Hz. Counts reflect sensor edges; sign is inferred
  from command direction. They are not independent measurements of direction.
- Installed kinematics has gain 1 and trim 0. Direct WheelsCmdStamped commands
  bypass kinematics, but no nonzero trim compensation was found to be lost.
- The driver passes the two requested values separately. For active positive
  commands it maps them to `floor(60 + 195 * command)`: 0.03 becomes PWM 65,
  0.15 becomes 89, and 0.20 becomes 99. Zero becomes zero. Thus 0.20/0.03 is
  only about 1.52:1 in PWM settings, not 6.67:1. These are not measured speeds.
- The executed-wheel topic is a software echo after the driver call, not a
  measurement of PWM voltage, current, torque or rotation.
- The installed I2C helper catches write errors and returns -1. Higher driver
  layers ignore those results and still publish the command echo. This is a
  confirmed diagnostic blind spot, not proof an I2C failure occurred today.
- Historical attempts at 0.20/0.00 also sometimes failed. Restoring the successful
  pair alone cannot establish repeatable turning or identify every cause.

## Changes made without movement

1. Restored **0.20/0.00** only for the bounded sharp-corner pivot. Kept the
   one-second approach, returning-white handoff after 0.10 seconds, normal
   camera-guided reacquisition, ten-second corner deadline and fifteen-second
   session cap. Normal launcher settings and installed software were preserved.
2. Fixed a stall-monitor sampling error. It previously required too much span
   between echoes inside each moving window, so bursty delivery could conceal
   an already-stalled wheel. It now carries the preceding command across the
   window, while respecting stops, refresh gaps and startup grace. Replay of the
   isolated failed pivot now detects the stall at +1.405 seconds. It does not
   falsely flag the successful continuous pivot or the rolling/equal trials.
3. Added optional motor-register evidence to every bounded session in an
   independent read-only child. It reads the verified controller registers at
   about 5 Hz; it never constructs a PWM/HAT/Motor object (those constructors
   change hardware). Two snapshots flag whether a driver write overlapped the
   read. Errors are recorded, not silently treated as successful reads. The
   child has a deadline and parent-pipe lifetime, and is stopped before transfer.
4. Added an advisory encoder-response assessment for fixed tests. Near-equal
   counts under unequal commands are labelled weak/wrong turn response, rather
   than successful turning. The 0.20 differential-fraction cutoff is diagnostic
   only; it is not a heading estimate or automatic speed adjustment.
5. Added boot identity and container start times to private session evidence so
   recordings across restarts are not mistaken for one continuous runtime.

## Verification and next step

167 focused Windows tests passed: lane/vision readiness, navigation, steering,
obstacles/connection, supervisor and session tests. A fresh isolated local venv
used Python 3.12, OpenCV 5.0 and NumPy 2.5.3. Docker Desktop was unavailable, so
the exact Noetic/OpenCV 4.2 integration suite was not rerun in this session.
The motor-register reader was also exercised read-only on stationary duck2;
both PWM enable channels read zero, with a stable snapshot and no read error.

The next physical run requires a fresh Go. Use the restored bounded corner
preset at the familiar approach to the right bend; record both the PWM evidence
and encoder response. If the registers disagree with sustained requested
commands, inspect the driver/write path before changing gains. If they agree,
the open-loop command-to-rotation response still needs a measured solution.
Neither one passing equal-wheel trial nor one passing pivot proves the entire
drive system or sharp-corner behavior reliable. No Git operations occurred.
