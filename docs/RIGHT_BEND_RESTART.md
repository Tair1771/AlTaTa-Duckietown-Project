# Right-bend restart diagnosis — 2026-09-09

## Current reviewed trial (supersedes earlier presets below)

Latest: see [the equal-wheel and sharp-turn diagnosis](TURNING_DIAGNOSIS.md).
The bounded corner pivot is restored to **left 0.20 / right 0.00**, which is
the pair used in today's sustained sharp turn. The 0.15/0.00 setting discussed
below is historical. White-return handoff remains enabled. Motor register
evidence and the corrected stall monitor will accompany the next fresh-Go run.
The equal-wheel result does not establish identical motor power; the rolling
profiles have not demonstrated sufficient sharp-turn response.

User follow-up after the 0.15/0.00 stall: reported battery level 93%, charger
connected with maximum cable slack, both wheels free to turn manually, and
rear support freely moving and unobstructed. Earlier successful runs also had
the charger connected. These observations weaken the obvious obstruction and
low-charge hypotheses; they do not measure motor supply voltage under load.

Comparison of saved executed commands and encoders found a sustained pivot
in the earlier continuous-turn recording lasting 3.35 seconds with 137 left
ticks. The latest 0.15/0.00 run held the pivot for 1.55 seconds but advanced
only 12 left ticks. A sustained right pivot has therefore worked previously;
the current loaded stall is intermittent, not proof that the chassis cannot
pivot. The comparison does not control exact placement, load or power.

Read-only inspection confirmed DB21J/HATv3: left direction uses PWM channels,
right direction uses GPIO. No camera-based stop or explicit rule stopping
both motors for a zero right request appears in the inspected driver path.
The command-executed topic echoes applied software requests; it does not
measure motor voltage, current or torque. Motor output under load remains
unresolved. No code, calibration or physical motion was changed for this
follow-up inspection.

Latest update: the ordinary five-second lane-width expiry no longer stops a
confirmed `turning/pivot` state if fresh camera/status reports current yellow,
no white, and specifically `Lane lost: boundary gap exceeded`. This exception
does not apply to ordinary tracking, approach, reacquisition, missing yellow,
out-of-image estimates, stale data or a node fault. The node's ten-second
corner deadline and the supervisor's motor-stall guard still apply.

Run `20260909T112801Z-sharp-right-white-return-eb354c7a` stopped after
4.04 seconds. The user observed forward travel, the approximately one-second
approach, a brief right turn and stopping. The telemetry places approach at
1.40 seconds and pivot at 2.50 seconds. Left ticks reached 3532 by 3.25 seconds
and stayed unchanged through 4.00 seconds while executed left remained 0.15
and right remained zero. Only after that did the supervisor request zero.
The encoders were still publishing; camera age was 0.064--0.165 seconds.
This establishes lack of encoder movement before the safety stop, not movement
being interrupted by a white timeout. White had not returned at the stop.
Motor power, traction and mechanical resistance remain unresolved; no software
cure of the loaded pivot stall is claimed.

The installed Dagu driver was reinspected read-only. It releases a motor at
zero and maps 0.15 to PWM 89 using its configured 60--255 bounds. It does not
implement a white-border timeout. The project session report now preserves the
specific early-stop reason instead of only reporting generic supervisor
failure, reducing repeated log inspection. All 94 focused controller,
supervisor and session tests passed in the ROS/OpenCV environment with network
access disabled. No physical movement or Git operation occurred in this fix.

The bounded camera profile retains a rolling 0.03 inner-wheel floor during
ordinary lane tracking. A recent two-boundary lane followed by confirmed
white loss, retained yellow and right-turn error starts a one-second equal-wheel
approach (after 0.20 seconds of confirmation). The turn then requests left
0.15 and right zero continuously. Returning white held for 0.10 seconds releases
the fixed pivot immediately; no additional minimum pivot duration is imposed.
The normal camera controller centres between the boundaries while the corner
remains in `reacquiring`. Both boundaries must be stably aligned for 0.30
seconds before that state clears. The ten-second corner deadline includes
reacquisition; the full test remains capped at fifteen seconds.

This is an intended right-angle maneuver completed by visual evidence. Neither
the one-second approach nor the white reappearance measures distance or a
90-degree heading, and missing white alone does not prove a perpendicular bend.
No colour threshold, driver calibration or stall timeout was changed.

The latest run, `20260909T111150Z-sharp-right-rolling-precorner-d9dd0a8f`,
entered approach and pivot but stopped after 4.95 seconds on the outside-wheel
stall guard. The user observed forward travel, turning and then stopping.
The camera stayed fresh (0.036--0.136 seconds); white remained absent at the
stop, so the returning-white handoff never ran. Final zero feedback, normal
ownership restoration and preservation of 57 frames passed. This distinguishes
the stall from a startup or camera timeout. Both 0.15/0.00 and 0.20/0.00 have
previously stalled under load; lowering the turn request at the user's request
is not evidence that the physical stall has been cured.

Verification: 167 focused ARM64 tests passed with networking disabled,
including the full approach/pivot/white-return/correction sequence and the
unchanged corner deadline. Windows startup checks passed against the real bot
without camera subscriptions or wheel commands. A physical retry remains
pending a fresh Go. No Git operations were performed.

For the repeatable Windows entry point and startup-only mode, see
[ROBOT_SETUP.md](ROBOT_SETUP.md#repeatable-windows-test-entry-point).

Two attempted runs failed before motion release after the overnight restart.

- `20260909T074937Z-right-bend-restart-0371cd52`: the temporary upload omitted
  `duckie_lane_follower/route_map.py` and its package initializer. The session
  uploader now includes both. Recorded requested/executed wheels were zero.
- `20260909T075739Z-right-bend-restart-fixed-e03c4f3f`: the follower started,
  but preflight could not establish both lane boundaries. Motion never started;
  final zero feedback and restored normal ownership were confirmed. The earlier
  conversational report that a physical window completed was incorrect.

Offline replay of all 44 saved stationary frames rejected the white boundary
at its original HSV value lower bound of 170. A lower bound of 160 succeeded
in only 9 frames; 150 succeeded in all 44. The supervised camera-guided preset
now uses white HSV `[0, 0, 150]` through `[180, 55, 255]`. Ordinary launchers and
default colour bounds are unchanged. Yellow bounds, steering, speed, temporal
fallback and preflight safeguards are unchanged. Status includes white bounds
so the supervisor verifies the intended configuration before releasing motion.

A subsequent read-only check detected both boundaries in 12/12 fresh camera
frames, with ROS timestamp ages between 0.029 and 0.047 seconds. This checked
perception only and published no wheel commands. The 32 supervisor tests and
14 session-tool tests passed. Camera evidence remains outside the repository.

This resolves the demonstrated startup blockers; it does not establish right
bend lane containment. The prior right-wheel border crossing still requires a
separate supervised movement evaluation. A fresh user Go is required per run.

The next supervised run released motion for 2.67 seconds, then stopped on lane
loss with final zero feedback. The user observed straight motion and no visible
right turn. Although the controller requested its strongest available right
turn, both encoders advanced by exactly 120 ticks. Comparison with the previous
near-successful sharp-right run showed that the newer `0.03` inner-wheel floor
prevented the earlier `0.15`/`0.00` maximum-turn command.

An interim supervised preset tapered that floor as steering approached its
maximum. It retained `0.09`/`0.09` near straight driving and permitted
`0.15`/`0.00` at maximum right steering. The ordinary lane-follower default was
unchanged. Its 24 first-readiness, 32 supervisor and 14 session tests passed,
and offline replay retained bounded output and lane-loss stopping. The next
live retry showed that this interim setting did not reproduce the selected
successful run, so it is no longer the active supervised profile.

After that retry, the robot made several right corrections but did not finish
the bend and stopped on lane loss after about four seconds. The selected
successful reference remains the 2026-09-08 `sharp-right-yellow-only-1p5` run:
it used a 15-second requested window, retained yellow-only tracking for up to
1.5 seconds, allowed the inside wheel to reach zero without an active wheel
floor, and stopped at the red line after 7.48 seconds. The supervised profile
has therefore been returned to a zero active-wheel floor with tapering disabled.
Future ground-test sessions now default to 15 seconds. The dim-light white
threshold remains test-only because the original threshold rejected the current
recorded scene. A new physical result is still required before calling the
restoration successful.

The restored-profile retry stopped after 3.58 seconds. Its final status sequence
showed a current yellow boundary throughout the preceding 1.5 seconds, while the
white boundary was outside the image; the stop occurred exactly when the
yellow-only timer expired. For the supervised sharp-bend profile, that timer is
now 5 seconds. It still uses the current yellow position and the most recently
measured lane width. Complete loss of yellow and white, stale camera/status,
red detection, encoder faults, competing publishers and watchdog expiry retain
their immediate stops. The ordinary node default and launchers remain unchanged.

Two subsequent ground runs stopped on the encoder-stall guard while holding the
maximum `0.15/0.00` right-turn command. The user confirmed the robot was
unobstructed. A lifted-wheel diagnostic commanded left `0.15` and right zero for
2.12 seconds: the left encoder advanced 289 ticks, the right remained at zero,
and the user observed continuous rotation and prompt stopping. This isolates
the failure to the loaded pivot rather than the left motor, encoder or command
path. The supervised camera profile now keeps the inside wheel rolling at
`0.03` and raises only the outside-wheel ceiling to `0.20`, with a `0.11`
steering cap. Near-straight output remains unchanged. This load-compensated
profile requires a new supervised ground result.

The exact replacement pair was then checked with duck2 secured upside down for
2.08 seconds. Executed feedback remained left `0.20`, right `0.03`; encoder
deltas were 324 and 201 ticks respectively, and final zero feedback was
confirmed. The user observed both wheels start together, rotate continuously
with the left visibly faster, stop together, and stop promptly. The pair is
therefore ready for one supervised loaded bend test; this lifted result does
not itself establish ground traction or lane containment.

The first loaded run of that pair, recorded as
`20260909T091152Z-sharp-right-watchdog-fix-e8e42d4a`, moved for 3.19 seconds.
The user observed insufficient right turning, a crossing of the yellow dashed
divider, and the subsequent stop. Synchronized status agrees: both boundaries
were visible initially, white left the view after about 0.8 seconds, and the
controller reached `0.20/0.03` at about 1.7 seconds. The yellow boundary then
moved across the image until both boundaries were lost. Fresh zero feedback
confirmed stopping, and kinematics was restored as the sole normal publisher.
Frames recorded after the stop, when the user picked up the robot, were excluded
from the motion diagnosis.

The load-compensated pair changed the balance between forward and differential
command, but these normalized requests are not measured wheel velocities and
cannot establish physical curvature. Steering was already saturated, so a
further gain increase would not change the requested pair or correct this
failure.

The supervised profile now keeps the `0.03` inner-wheel floor through the first
90 percent of steering authority and tapers it to zero only over the final 10
percent. Thus ordinary corrections retain the loaded-motion floor, while a
maximum sharp-right request becomes `0.20/0.00`. The ordinary node default and
launchers remain unchanged. The existing encoder guard still requires the
commanded outside wheel to advance and stops the run if it stalls.

Verification passed 25 image/controller readiness tests in an isolated
network-disabled ARM64 ROS/OpenCV container and 53 supervisor/session tests.
Replay of the failed motion frames retained the `0.03` floor during ordinary
corrections and reached `0.20/0.00` only at maximum steering. Replay establishes
the requested control transition, not physical lane containment. A new physical
run requires a fresh Go.

## Latest tapered trial and supervised corner mode

Run `20260909T092619Z-sharp-right-tapered-pivot-dc9cec36` stopped after
6.73 seconds on the left-wheel stall guard. The user observed alternating
right corrections and straight motion, followed by stopping. The camera still
reported both boundaries at the end. Fresh encoder messages in the final
partial second contained only one left tick and zero right ticks; normal
kinematics ownership and final zero output were confirmed. This is distinct
from a missing-lane timeout.

Read-only inspection of duck2's installed Dagu driver confirmed a speed
tolerance of 0.01: the floating-point residue near 6.7e-18 is treated as a
released motor with zero PWM. It does not activate minimum PWM. Also, wheel
commands are not measured velocities: the earlier ratio of command difference
to command sum is not a verified physical curvature calculation.

The bounded camera profile now separates ordinary lane tracking from an
explicitly latched sharp-right maneuver. Missing white alone is insufficient:
the node requires a complete lane within the preceding second, a current yellow
divider, a right-turn error of at least 0.10, and 0.20 seconds of confirmation.
It then uses a 0.15-second equal-wheel approach and a `0.20/0.00` right pivot.
The higher outside value was selected because loaded `0.15/0.00` attempts had
already stalled. The timing is a provisional test setting, not a distance or a
claimed 90-degree calibration.

The turn remains latched when the first white segment reappears. It returns to
normal lane following only after both boundaries remain visible with absolute
lane error no greater than 0.13 for 0.30 seconds. The pivot has a 3.5-second
deadline. Yellow loss, red detection, camera failure, client interruption,
encoder stall, competing publishers and the independent session watchdog still
stop it. Routes and passing cannot be combined with this mode. Ordinary
launchers keep it disabled.

Network-disabled replay entered the maneuver once in each recent failed run.
For `sharp-right-watchdog-fix`, it changed from following to approach at 1.41
seconds and pivoting at 1.68 seconds. For `sharp-right-tapered-pivot`, it began
approach at 1.60 seconds, pivoted at 1.87 seconds, ignored the premature white
reappearance, and returned to lane following at 5.34 seconds after stable
alignment. This occurs before the prior 6.73-second stall point. Replay proves
state and command behavior only; sustained loaded pivoting and lane containment
still require one supervised run.

Current verification passed 29 image/readiness tests, 48 obstacle, connection
and steering tests, and 53 supervisor/session tests. Navigation and parameter
contract checks also passed in the isolated ARM64 ROS/OpenCV environment.

## Loaded pivot retry and rolling-relief revision

The first latched-corner ground run lasted 12.57 seconds, entered `approach`,
held `turning` for about 2.6 seconds, reacquired both boundaries, and returned
to ordinary following before stopping on a detected red line. A repeated run
from the user's reset position entered the same maneuver but stopped after
4.09 seconds on `Left wheel stalled while commanded`. The user saw right
turning, a pause in turning, and another right correction; duck2 stayed within
the lane. Forward motion was uncertain by visual observation.

The second stop was not a camera timeout or an encoder sampling error. The
left wheel remained commanded at `0.20` and the right at zero, but neither
encoder changed for more than 0.8 seconds. Camera age stayed between 0.052 and
0.120 seconds, and the final zero-output window passed. Normal kinematics was
restored as the sole normal wheel publisher after both runs.

The bounded test profile now divides the latched turn into a 0.45-second
`0.20/0.00` pivot followed by 0.25 seconds of rolling relief with a requested
`0.20/0.09` pair, repeating until the existing stable-lane exit or the
3.5-second corner deadline. The published inside-wheel increase still obeys
the acceleration limit. The outside command remains at full turn power, and
the state does not return to ordinary lane corrections during relief. This
addresses the measured scrub-load stall without weakening the encoder guard.
Red, yellow loss, stale camera/status, publisher ownership changes and the
independent duration watchdog retain their existing stop behavior. Ordinary
launchers keep this mode disabled.

The focused network-disabled run passed 65 tests in duck2's ARM64 ROS/OpenCV
container. Saved-frame replay exercised `approach`, `pivot`, and
`rolling_relief` with bounded wheel values. Replay verifies the command state
machine only. One supervised ground run is still required to establish whether
the relief interval prevents the loaded stall while completing the bend.

## Continuous corner trial after the relief run

The relief run stopped at 5.21 seconds on the 3.5-second corner deadline.
No encoder-stall fault was reported during this run; that does not establish
that the stall problem is solved. The user observed interrupted forward/turning
motion and the right wheel over the white border near the outgoing straight.
Record this as a failed containment test. Both boundaries appeared near the
deadline, but the run did not demonstrate stable lane reacquisition before it.
The earlier description that another 0.3 seconds would complete reacquisition
was an estimate, not a verified outcome.

At the user's request, the next bounded preset uses a one-second equal-wheel
approach after corner confirmation, then a continuous left 0.20/right zero
request. Relief is disabled by setting its inner speed to zero. The corner
deadline is now 10 seconds, with the overall session still capped at 15 seconds.
The approach follows the existing 0.20-second confirmation; missing white by
itself is insufficient. Current yellow evidence remains required. The exit
behavior was revised after the trial below: a confirmed returning white
boundary now ends the fixed pivot, while both boundaries and acceptable centre
error are still required before the corner state is cleared. This is visual
reacquisition, not a measured 90-degree turn.

The longer approach is a provisional geometry change, not a demonstrated cure
for the loaded pivot stall. Stall detection, camera freshness, red stops,
boundary-risk detection, lane-loss stops and the independent watchdog remain
active and can stop before either duration limit. In particular, the existing
five-second yellow-only estimate expiry has not been extended to ten seconds.
No motion or Git operations occurred while preparing this revision.

Verification: 114 isolated ARM64 controller/readiness/supervisor/navigation/
connection/steering checks passed with networking blocked, and 53 host-side
supervisor/session checks passed (these suites overlap). New tests cover the
one-second approach, continuous output across former relief periods, stable
reacquisition after a flickering boundary, and expiry of the ten-second limit.

## White-boundary handoff after the continuous-turn trial

Run `duck2-ground-evidence-sharp-right-continuous-20260909` moved for 6.59
seconds and stopped on `Yellow boundary lost during sharp corner`. The user
reported that the approach and continuous right turn were good, but that the
turn continued after the white boundary should have returned and left duck2
facing the white border. Encoder deltas were 296 left and 153 right, final zero
feedback was confirmed, and normal wheel ownership was restored.

The synchronized status record explains the overshoot. The corner entered
`approach` at 2.12 seconds and `turning` at 3.13 seconds. Both boundaries first
returned at 5.43 seconds with lane error near -0.13 and remained visible while
the fixed `0.20/0.00` pivot continued. The centre error then worsened to about
-0.227. At 6.52 seconds the yellow divider left the image while white remained,
and the old logic stopped. Its exit required the lane to be already centred
for 0.30 seconds before releasing the pivot, so the pivot itself prevented the
exit condition from becoming true.

The bounded sharp-corner mode now separates ending the fixed pivot from
finishing lane reacquisition. After at least one second of turning, a visible
white boundary held for 0.10 seconds changes the state to `reacquiring` and
immediately uses the normal camera-guided wheel calculation. A single early or
one-frame white detection cannot release the pivot. The corner state clears
only after both boundaries remain within the existing error limit for 0.30
seconds. The original ten-second total corner deadline is not restarted at the
handoff, and complete lane loss, stale data, red detection, encoder stalls,
publisher conflicts, client loss and the independent session watchdog retain
their stop behavior.

The 162 focused lane, corner, navigation, obstacle, supervisor and cleanup
tests passed in the installed ARM64 ROS/OpenCV environment with networking
blocked. A targeted test reproduces the recorded ordering in which white
returns and yellow then leaves the view, and verifies that the fixed pivot is
released before that loss. The saved JPEG set was sampled at 4 Hz and did not
retain the short corner-trigger sequence, so replay of those images alone did
not re-enter the maneuver; the synchronized status evidence above is the
authoritative reconstruction. No physical retry or Git operation occurred
during this fix.

## Pre-corner loaded stall and rolling-floor restoration

The first ground attempt after the white-boundary handoff change did not move
visibly. The user heard the normal motor sound for about one second. The
supervisor stopped on `Left wheel stalled while commanded`, confirmed final
zero feedback, restored `/duck2/kinematics_node` as the sole normal wheel
publisher, and preserved 44 frames. Camera age remained between 0.049 and
0.120 seconds. The encoder deltas were only six left and two right ticks.

This attempt never entered the sharp-corner state, so it did not exercise the
new white handoff. Both boundaries were visible with lane error around
0.13--0.14. The ordinary controller saturated at approximately `0.20/0.00`;
the outside wheel was powered against a stationary inside wheel and failed the
existing loaded-stall guard.

The bounded preset now keeps its verified `0.03` active-wheel floor throughout
ordinary lane tracking, including saturated pre-corner corrections. The same
recorded frames therefore replay at `0.20/0.03`. A zero inside-wheel command is
reserved for the separately confirmed sharp-corner maneuver, whose encoder
stall guard remains authoritative. This changes only the bounded test preset;
ordinary launchers remain unchanged. The 53 host supervisor/session tests and
162 network-disabled ARM64 lane, corner, safety and cleanup tests passed. No
physical retry or Git operation occurred after this change.

## Isolated loaded-pivot diagnostic

The next physical check separates the intermittent loaded pivot from camera
and corner-state decisions. A dedicated Windows launcher option requests left
`0.15` and right `0.00` for at most two seconds with duck2 upright on a clear,
flat surface. It retains the encoder-stall stop, independent time limit,
exclusive wheel ownership, explicit final zero and normal-control restoration.
It does not start the lane follower or use camera detections to steer.

Run the read-only startup check without `-Go`:

```powershell
.\tools\Start-Duck2-GroundTest.cmd -Label ground-right-pivot -GroundRightPivot
```

Only after a fresh physical-test authorization, add `-Go`. The launcher uses a
two-second duration automatically unless an explicit shorter duration is
provided. The charger must be unplugged and clear of the chassis for this
comparison. A successful rotation would isolate the remaining problem to the
camera-guided sequence; another encoder-confirmed stall would point to the
loaded pivot itself and would not justify weakening perception safeguards.

Run `20260909T114923Z-ground-right-pivot-8b63df4a` delivered left `0.15` and
right `0.00` for the complete two-second window without using lane detection.
The user saw duck2 begin turning and then stop quickly despite full cable slack
and no physical obstruction. The left/right encoder deltas were only 6/1 ticks
while 48 executed-command samples retained the requested values. Final zero
feedback was confirmed, the evidence bundle was verified, and normal ownership
returned to `/duck2/kinematics_node`. This isolates the failure from white-line
expiry and corner-state timing: the stationary-inner-wheel pivot did not remain
physically effective under load.

The next diagnostic keeps the same outer-wheel request and adds the installed
driver's minimum active command to the inner wheel: left `0.15`, right `0.03`,
for at most two seconds. This is a rolling right turn intended to reduce scrub
load. Run `20260909T115656Z-ground-rolling-right-19f2530d` completed that
two-second window without an early stop. Encoder deltas were 132 left and 121
right, and the user saw mostly straight travel with only a small right turn.
The inner-wheel command therefore removed the loaded stall, but `0.15/0.03`
does not provide enough differential for the sharp bend.

The next isolated profile keeps the inner wheel at `0.03` and raises only the
outer wheel to the test preset's existing `0.20` limit. It is exposed as
`-GroundStrongRollingRight`, remains capped at two seconds, and passed 67
focused session/supervisor tests. The camera-guided corner preset remains
unchanged until this stronger profile is observed under load.
