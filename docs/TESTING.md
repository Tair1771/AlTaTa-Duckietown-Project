# Testing status

## 2026-09-10 — normal live-chat release and review

**Latest physical result: incomplete.** The user reports that duck2 did not
curve enough and stopped. This is not accepted full-route validation. No new
physical movement or calibration change was made during this release review.

The normal companion enables live chat by default: selected map turns seed
the queue, messages change future junction choices, timed pauses take effect
on robot acceptance, and straight-road speed profiles remain separate from
curve and junction profiles. When the queue ends, the bot stops at a red line,
asks for a direction and waits up to 60 seconds. Optional scenario restrictions
do not apply to the normal launcher. Initial-straight centering remains opt-in
for the explicitly placed straight-path check.

Review fixes: reconnect is blocked during Start; background connection errors
cannot unlock a second Start while the first remains pending; a queued Start
result cannot replace the UI's Stop outcome. Final lane reporting reconciles
only an acknowledged instruction with matching outgoing lane, index and turn.
Camera HTTP failures are distinguished from network timeouts, and invalid
capture timestamps are rejected. The preview's session-free detector helper
is included in this release, fixing its earlier frame-processing exception.

Verification: 562 native tests completed, with 559 passing and three platform
skips; 80 Python sources, documentation links and Windows launcher targets
passed inspection. Network-isolated HTTP/ROS checks passed real normal/mask/
overlay delivery, all speed profiles, immediate zero, a measured 7.093-second
synthetic pause/resume interval, final-red stopping, scenario isolation, and
the normal map queue → straight override → C red prompt → left instruction →
Stop sequence. These are synthetic wheel requests and software observations,
not measurements of track containment. Source and PDF include all three authors.
The broader disposable ROS suite also passed (71 prerequisite tests plus its
real ROS transport checks), including red dwell, route progress, heartbeat loss,
camera loss, pause/resume and Stop. Its local evidence is retained outside Git
under bench-checks run `20260910T185021Z-b9ce8a35`.

The earlier dated sections below retain diagnostic history; the current
entry points are [STARTUP](STARTUP.md), [USAGE](USAGE.md) and [LIVE_CHAT](LIVE_CHAT.md).

## 2026-09-10 — test workflow ended; companion app restored

At the user's request, removed the experimental paired road-path controller and
encoder-coast assistance, including their preview additions and focused tests.
Restored the continuous app profile used before these two experiments, retaining
the earlier red-stop and junction fixes. The test supervisor is not running.
Controller startup remains stopped; route selection and Start belong to the
user in the companion app. No new physical test was run during this restoration.
The experiment entries below are historical and do not describe active code.
This restores a prior profile; it does not establish reliable full-route driving.

## 2026-09-10 — fresh video diagnosis and paired road path

Run `20260910T125513-full-route-AEBC-encoder-left-assist` stopped after about
seven seconds with `Lane lost: boundary gap exceeded`. The user reports no
effective left turn; the supplied external video agrees with forward travel
toward the bend. Requested and executed zero output were verified afterward.
The encoder assistance never activated. Initial requested wheels were about
0.109/0.071 (a right correction); the lane estimate only turned negative shortly
before yellow vanished from the lower image region. This run does not establish
a motor-power fault. The primary demonstrated error is late/wrong-direction
visual steering, followed by expiry of the 0.3-second white-only fallback.

Continuous mode now enables `road_paired_path` in both controller and camera
preview. It traces visible yellow and white stripe runs at matching image rows,
rejects insufficient/disconnected/perspectively implausible pairs, and measures
near position and forward direction separately. Lookahead uses observed pairs
at image height 0.45--0.50 and near position up to 0.75, without extrapolating
an unseen boundary. The 0.49 image reference belongs to paired geometry; the
ordinary legacy centroid target remains 0.441. The paired controller uses no
centroid trim, applies a small noise band, and retains smoothing, steering slew,
acceleration and wheel caps. Current geometry replaces remembered curve steering
as the road straightens. Intersections, red stops and sharp-corner phases retain
their previous control. Missing-image and actual lane-loss stopping remain.
The unverified encoder coast experiment is disabled in this profile.

Offline results: 63 focused paired-path, camera, lane/reference and red-route
tests passed; syntax, documentation links and launcher targets passed. Sparse
recorded-frame replay, holding each saved frame until the next recorded frame
time, requests equal wheels on the first two straight views, then a left turn
before yellow loss (approximately 0.044/0.136, followed by 0.03/0.20). It still
stops on the final genuinely lost-lane view. These are replay requests, not new
executed motor measurements or a simulation of the corrected trajectory.
The earlier accepted 19-frame sequence was also inspected: path steering reduces
as the curve exits. Synthetic tests cover centering, heading, dashed markings,
straightening, false fragments, Stop, stale camera, and perception-to-controller
callback transfer. All raw evidence, video contact sheets and replay details
remain outside the repository under `Duck2/diagnostics/20260910-fresh-curve`.
Physical success of this revision is pending a fresh supervised Go.

## 2026-09-10 — provisional encoder-assisted left curve

The restored continuous profile now opts into `road_left_encoder_assist`.
Vision, red-stop/junction behavior, outer-wheel power and existing stopping
checks remain unchanged. During an ordinary-road strong left request only,
fresh camera, executed-command and encoder evidence must agree. If both wheels
are advancing but their normalized count difference is below 0.08, the inner
left wheel receives zero for a target 0.15-second coast, followed by a 0.35-second
cooldown and a new measurement window. Restoration occurs on a camera callback;
the independent camera watchdog still stops stale-image operation. This is not
a calibrated steering angle or a hard real-time 0.15-second motor pulse.

The threshold provisionally separates the recorded failed 72/82 differential
from the earlier accepted 420/508 differential. Different observation windows
and track poses limit that comparison; a physical retry is still required.
Straight/right requests, junctions, missing boundaries, stale feedback, encoder
resets and stalled wheels do not enable this assistance. Ordinary launchers
retain the default disabled setting.

Offline: nine focused assist tests and 32 lane/red-route/reference regressions
passed. Nineteen accepted frames reproduced the original perception and wheel
mixing with assistance disabled; these frames alone do not validate the new
encoder assistance. No physical run was performed during this change.

### Restored-profile observation and wheel evidence

The user reports no visible turning and stopping before the curve in
`20260910T124730-full-route-AEBC-restored-accepted-curve`. Red detection stayed
false. Executed-wheel feedback followed requested output, and the cumulative
zero-feedback count stayed at 676 through the moving samples, increasing at the
final lane-loss stop. There is no recorded intermittent-zero sequence explaining
the reported lack of turning.

The controller initially requested a slight right correction and only requested
strong left steering around 6.31 seconds (0.03/0.1745, rising to 0.03/0.20).
From 6.31 to 7.75 seconds, left/right encoder counts increased 72/82: the right
wheel rotated only modestly farther despite the large requested difference.
This establishes late steering and weak realized wheel differential, not a
calibrated turn radius or a diagnosed physical defect. A further perception
retune alone cannot be assumed to solve it. No settings or movement changed
during this read-only diagnosis; duck2 remains manually stopped.

## 2026-09-10 — restored accepted curve profile retry

After a fresh Go, ran A -> E -> B -> C with road heading guard and left
lookahead disabled, the accepted curve parameters restored, and the red/junction
fixes retained. Both boundaries and zero executed output were checked before
Start. The run again stopped with `Lane lost: boundary gap exceeded`, before
a junction. Stop and zero requested/executed output were verified. Evidence:
`Duck2/evidence/20260910T124730-full-route-AEBC-restored-accepted-curve`, outside
the repository. The user subsequently reported no visible turning and stopping
before the curve (see the wheel-evidence analysis above). No settings were changed
during or after the run, and no automatic retry was started.

## 2026-09-10 — restore the accepted curve profile

The user reports premature left turning across yellow followed by straightening
toward white, and requests restoring the normal successful curve behavior.
They covered red tape outside the road and explicitly requested no change that
ignores those reds. Red detection remains unchanged.

The continuous launcher now explicitly disables `road_heading_guard` and
`road_left_lookahead`. This also disables forward-border anticipation and the
extended confirmed-left-curve yellow-gap behavior. The original 0.30-second
white-only fallback, calibrated steering, speed limits, colour thresholds and
bright-white reference remain. Experimental helpers are inactive. The left-exit
handoff and second-red stop/dwell/route transition fixes are retained, as is
read-only wheel feedback. No motor calibration or red threshold was altered.

A comparison against the saved source/launcher preceding the full-route drift
changes confirms identical lane errors and wheel commands on all 19 frames from
`20260910T112401-continuous-left-reference-retry`, which the user called perfect.
All original launcher parameter values match that snapshot. The 32 focused lane,
white-reference and second-red tests passed. At the user's request, the full
regression and container build were not repeated for this configuration rollback.
Comparison evidence is outside Git at
`Duck2/diagnostics/20260910-restore-accepted-curve`. This replay does not guarantee
a new physical trajectory; a fresh Go is required. No Git operations occurred.

## 2026-09-10 — unchanged forward-curve repeat

The user explicitly requested the same test again and supplied a fresh Go.
No source, launcher or parameter changes were made. The camera showed the
starting approach with both borders; requested and executed output were zero.
The repeat again stopped with `Lane lost: boundary gap exceeded` before a
junction. Stop and zero requested/executed output were verified. Evidence is
retained outside Git at
`Duck2/evidence/20260910T124132-full-route-AEBC-forward-curve-continuation`.
Physical observations are pending; no automatic further run was started.

## 2026-09-10 — forward-curve continuation retry

After a fresh Go, the A -> E -> B -> C routine ran with current camera,
both boundaries initially visible, zero executed feedback and sole controller
ownership. It stopped on `Lane lost: boundary gap exceeded` before a junction.
Stop was sent and manual stop with zero requested output was confirmed.
Frames, status, executed feedback and encoder counters are retained outside
the repository at
`Duck2/evidence/20260910T123952-full-route-AEBC-forward-curve-continuation`.
Physical observations are pending. No automatic retry or logic change followed.

## 2026-09-10 — earlier forward-border tracking and confirmed left-curve gaps

The user reports that the curve-lookahead run advanced, slowed while partly
turning, then stopped. The two later mask/overlay screenshots were taken after
the user moved duck2 backward; they are excluded as evidence of the stopped
position. Use the run's synchronized records instead.

In `20260910T122244-full-route-AEBC-curve-lookahead`, requested output favored a
right correction initially and then reached 0.03 left / 0.20 right from about
5 seconds. The lane-loss stop was recorded at 9.75 seconds. No red or route
completion stop occurred. Sparse requested-wheel status does not establish the
cause of the perceived slowdown; this run did not record executed feedback or
encoder counters. It is not evidence for changing motor calibration or power.

The prior lookahead still started at half image height. The continuous-road
option now traces genuine white-stripe runs from the near road upward to 40%
image height, pairs them with actual yellow-stripe runs, and rejects excessive
jumps and broad fragments. Nearby stripe continuity prevents choosing detached
background white objects as the far border. Two far and two nearer observed
pairs are required; otherwise the existing nearer lookahead remains available.
The main lane detector, HSV thresholds and junction/right-pivot geometry retain
their existing regions and settings. Replay of the latest run requests left
steering by the 2.22-second saved frame and reaches the left steering limit on
the 2.95-second scene. Rate limits still govern actual command changes; sparse
replay cannot simulate the altered physical trajectory.

Missing yellow alone does not select left. With the continuous lookahead option,
a two-border left curve must first be confirmed for 0.2 seconds. A brief yellow
gap can then retain that same left request and the measured lane-width estimate
for at most 0.8 seconds from the last confirmation, provided the white border
remains consistent. White-only frames cannot renew confirmation. Straight-road
yellow loss, loss of both borders, jumping white detections, boundary risk,
red/junction phases, manual Stop and stale camera data retain stopping behavior.
This is a bounded continuation of observed curvature, not an unmarked left turn.

Status now includes read-only executed-wheel feedback, feedback age, cumulative
executed/zero sample counts and both encoder counts/ages. These diagnostics
provide evidence for future slowdown analysis; they do not command motors.
Fourteen focused lookahead/feedback tests and the native regression suite pass:
463 tests total, 460 passed and three skipped. Evidence, saved-frame replay and
pre-change source are outside the repository in
`Duck2/diagnostics/20260910-far-curve-entry`. No Git operations or physical run
are part of this change. Physical curve containment and consistent motion
remain unverified until a fresh Go and the user's observation.

The ARM64 build, 175 focused checks and complete isolated ROS integration suite
passed (`Duck2/bench-checks/20260910T103408Z-67d8baa1`). The real ROS test confirms
left output during the allowed white-only gap, zero after its expiry, and
unchanged camera-loss/red-stop handling. The updated source is deployed with a
matching onboard hash. A subsequent stopped check confirmed fresh valid camera,
zero requested and executed output, fresh encoder messages and sole controller
ownership. The controller is awaiting a route; no movement was started.

## 2026-09-10 — curve-lookahead physical retry

After a fresh Go, the A -> E -> B -> C routine ran with lookahead enabled,
both boundaries initially visible, and sole controller ownership. It stopped
early with `Lane lost: boundary gap exceeded`, still following the first route
edge. The routine sent Stop and verified manual stop and zero requested wheels.
Evidence is outside the repository at
`Duck2/evidence/20260910T122244-full-route-AEBC-curve-lookahead`.
The user observed straight motion followed by slowing, partial turning and
stopping. This run does not validate curve containment or
the later junction transitions. No automatic retry or further logic change was
made; diagnosis will incorporate the user's observations.

## 2026-09-10 — earlier steering at the first left bend

The user reports that `20260910T120355-full-route-AEBC-second-red-fix` went
straight and stopped before the familiar left curve. Status confirms an early
lane-loss stop at 5.34 seconds, before any junction transition. Initial commands
favored the left wheel (right steering); by the time a strong left command
appeared, the yellow boundary was leaving the view. The white-only fallback
expired. This was not a red stop, destination stop or detected encoder stall.

The near-weighted whole-mask centroid missed the approaching bend. In addition,
the ordinary-road heading guard could retain a smaller wrong-sign request when
matched-row evidence requested the opposite direction. It now corrects that
sign disagreement within its existing supported-centre gate.

The continuous launcher enables `road_left_lookahead`: two far and at least two
near yellow/white pairs within the existing image region can request earlier
left steering. Ordered borders and expanding perspective are required; large
left-of-lane offsets keep ordinary corrective authority. The contribution ramps
in with image-space heading, retains stronger existing left requests, and passes
through the existing steering/rate/wheel limits. It does not invent missing
borders, extend the white-only timeout, or participate in red approaches,
junction maneuvers or sharp-right pivots. Ordinary road perception also resumes
its bright-white reference after the junction phase becomes `complete`.

Saved-frame replay requests left steering at 3.14 seconds, while the previous
centroid controller still requests right steering. This is a request comparison:
the saved images are sparse, and replay does not simulate a changed trajectory
or prove containment. The accepted left-curve recording continues to request
left steering and then relax it; early steering is stronger, so physical
revalidation remains necessary. The saved sharp-right phase sequence remains
approach, pivot, reacquisition and cooldown. Second-red route regressions pass.

Eight new regressions cover recorded entry geometry, wrong-sign guarding,
straight/right scenes, partial and implausible corridors, junction exclusions,
real callback evidence transfer, rate limiting, missing-image stopping and
post-junction calibration. The native suite passes 457 tests (454 passed,
three skipped). Diagnosis, replay and pre-change source are kept outside the
repository in `Duck2/diagnostics/20260910-curve-entry`. No movement or Git
operation is part of this fix. A fresh Go is required for physical validation.

The isolated ARM64 build, all 169 focused checks and the complete ROS integration
suite passed (`Duck2/bench-checks/20260910T101533Z-9ff748e2`). The new real-ROS
synthetic-image check confirms left steering with the option enabled and zero
output for absent markings or stale images. The temporary test container had
no robot network or hardware access and was removed. The update is deployed to
the companion container with matching local/onboard source hashes, fresh camera,
sole controller ownership, manual stop and zero wheel output. No route was started.

## 2026-09-10 — second-red fix retry stopped before a junction

After a fresh Go, the selected A -> E -> B -> C run started with both boundaries
visible. It ended early with `Lane lost: boundary gap exceeded`, before reaching
a junction. The routine sent Stop and confirmed manual stop with zero requested
wheels. Evidence is outside the repository in
`Duck2/evidence/20260910T120355-full-route-AEBC-second-red-fix`. Physical observations
confirm straight motion followed by stopping before the curve; this attempt did
not verify the second-red transition or right turn.
No automatic retry or steering change followed.

## 2026-09-10 — second red after left exit and sharp-right route transitions

The user clarified the preceding run: the improved initial driving and left
turn were good, and duck2 stopped at the second red line, where it should turn
right toward C. Recorded images support this account. The next stop line reached
the camera's near region at about 27.44 seconds; the final recorded status had
red detection true and only one remaining boundary pair. The controller had
failed to advance its route after the left exit and reported corridor loss.

The short-connector next-red transition existed only for right exits. It now
also applies to an opted-in, visually acquired left exit: a valid near-field
outgoing corridor must persist for 0.3 seconds with the departure line clear
before a subsequent red line may commit exactly one route edge. It then latches
zero, observes the normal dwell, and selects the map's next instruction. Thus
A -> E -> B -> C stops at B and selects right rather than remaining in the left
alignment phase. A destination red line completes the route and cannot depart.
The observed outgoing white border is retained through that stop; the right
entry can therefore detect its disappearance after the dwell even if it has
already ended in the camera view. No yellow boundary is required for that
authorized intersection pivot. Unconfirmed white loss still cannot initiate it.

The separate ordinary-road sharp-right detector remains enabled only while
following a lane, outside junction phases. It requires a recently complete lane,
current yellow, confirmed white absence, and right-turn error; brief flicker,
complete lane loss and intersection instructions do not qualify. Saved successful
sharp-right footage `20260909T123050Z-sharp-right-restored-020-272d6ab6` replayed
through the current source progresses through approach, 0.20/0.00 pivot,
reacquisition and cooldown. A red line during an active road bend now also honors
the destination index instead of unconditionally setting an intermediate red stop.

Replay of the latest failed run's distinct saved frames and status samples arms
the left-exit next-red gate at 27.17 seconds and stops at the second line at
27.44 seconds with route index 2. This is a corrected state transition, not proof
of the subsequent physical right turn. Seven new regressions cover edge advance,
dwell, the no-yellow right pivot, destination stops, false/flickering outgoing
evidence, road bends in routes and separation from junction control. Native
verification passed 449 tests (446 passed, three skipped), plus syntax, links
and launcher checks. Evidence and pre-change source are outside the repository
under `Duck2/diagnostics/20260910-second-red-right`.

The native ARM64 build and all 161 focused checks passed, followed by the full
isolated ROS integration suite. Its new synthetic-camera sequence confirms a
left exit, the second red stop, route index 2, and a right pivot without yellow
or white markings after the dwell. Stop still terminates output. Evidence:
`Duck2/bench-checks/20260910T095720Z-3d2f5099`; no hardware or robot ROS network
was accessible to that temporary test container.

Deployment verified matching local/onboard source hashes, both preceding
alignment fixes and the road sharp-corner detector enabled, fresh camera,
exclusive controller ownership, and manual stop with zero output. The app is
connected in `awaiting_route`; no Start or route was sent. The full physical
A -> E -> B -> C route remains unverified with this update.

Initial driving and left-turn steering settings are preserved. No physical
movement or Git operation occurred during this fix; a fresh Go is required.

## 2026-09-10 — alignment-fix route retry, observations pending

A fresh Go authorized the unchanged A -> E -> B -> C route with both new
corrections enabled and a 180-second outer limit. Status recorded the first red
stop at 17.45 seconds, crossing entry at 19.92 seconds, left turning at 20.75
seconds and outgoing search at 22.25 seconds. At 25.03 seconds the new visual
entry latched and requested approximately 0.0830/0.0955 while aligning. At 27.84
seconds the controller stopped with "Outgoing corridor lost after left arc;
position must be reset". Final status confirmed manual stop and zero requested
wheels. Route index remained 1; the route did not complete.

Evidence is outside the repository in
`Duck2/evidence/20260910T114717-full-route-AEBC-alignment-fix`. Physical observations
are pending. This confirms that visual handoff activated, but does not establish
containment or a successful outgoing-lane entry. No automatic retry or logic
change followed this attempt.

## 2026-09-10 — full-route right drift and missed left-exit diagnosis

The user reported that one wheel crossed the right white boundary before the
first curve. The bot subsequently recovered, but continued turning left at the
first intersection past a near-perfect outgoing-lane entry position. This makes
the full-route attempt a failed containment and junction-exit test, despite the
previous isolated left-curve acceptance.

The initial right drift was commanded: at 1.39 seconds the recorded pre-flip
steering was +0.05086 while the matched-row corridor had lateral error +0.01230
and heading error -0.03093. The whole-mask centroid combines yellow dashes and
white tape at different depths; its lane error +0.07485 overstated the required
correction. The existing positive trim also contributes. These are image-space
errors, not calibrated heading angles. The continuous-only `road_heading_guard`
limits an excessive centroid correction to the existing matched-row steering
reference when both borders have a valid, near-field, nearly centred/aligned
corridor. It preserves the ordinary controller outside that narrow condition,
including established curves and incomplete boundary evidence. No global trim,
speed, colour threshold or turn-power retuning was made for this change.

The left search requested 0.03/0.18 repeatedly. It briefly aligned at 26.16
seconds, but the next heading estimate exceeded the strict completion limit,
so it resumed the full arc. The new continuous-only `junction_left_visual_latch`
separates ending the arc from completing the intersection. A stable ordered
multi-row outgoing corridor switches to visual alignment; near-field centering
and heading stability are still required before advancing the route. Partial
rows after entry cannot restart the arc. Complete loss of usable corridor
evidence stops the maneuver. The original deadlines, freshness checks, Stop
priority, encoder watchdog and direction-specific turning powers remain active.
The left visual handoff now preserves the requested wheel ratio while ramping,
as the right visual handoff already did.

Replay of the recorded status reduces the initial raw steering reference from
0.05086 to 0.01235, and the second sample from +0.02698 to -0.00590. Combined
distinct recorded image/status observations latch the outgoing left corridor at
approximately 25.77 seconds and request countersteering after the recorded
overshoot, rather than another fixed left arc. Status-only sparse replay enters
at 26.16 seconds; image-only sparse samples do not confirm the stability window.
The combined observations are useful diagnostic evidence, not a simulated new
trajectory or proof of physical road containment. Recordings and replay output
are outside the repository under `Duck2/diagnostics/20260910-drift-left-exit`.

Validation: 442 native tests completed (439 passed, three platform skips), plus
source, documentation-link and launcher checks. The new regressions cover the
road correction in both directions, unchanged curve authority, false corridors,
partial rows, confirmation flicker, manual Stop, deadlines, route-completion
stability and wheel-ramp ratios. None of the 19 retained steering samples from
the accepted left-curve retry changes under the new road guard.

The current source built in a disposable network-disabled ARM64 container on
duck2. All 154 focused tests and the full ROS integration suite passed, including
a new real-ROS synthetic-camera case that exits the left arc, countersteers into
an offset outgoing corridor, and stops after corridor loss. This container had
no hardware access. Evidence: `Duck2/bench-checks/20260910T094211Z-397c4d81`.

The updated app controller was deployed stopped. Its source SHA-256 matched the
local source; both new flags were enabled. Live status confirmed a valid camera,
`awaiting_route`, manual stop and zero requested wheels with sole project wheel
publisher. Stopped driver preparation also verified zero executed feedback. No
route or Continue command was sent during deployment.

Physical verification of both corrections is pending a fresh Go. No additional
movement or Git operation occurred during diagnosis and implementation.

## 2026-09-10 — full A -> E -> B -> C attempt stopped at first junction

After a new Go, the unchanged continuous controller ran the selected route with
a 180-second outer observation limit. Status reported a red-line stop at 17.94
seconds, departure into the left crossing at 20.23 seconds, and outgoing-lane
search at 21.77 seconds. At 31.27 seconds it faulted with "Outgoing lane was not
reacquired; position must be reset". Route index remained 1. The routine sent
Stop and verified manual stop with zero requested wheels. This is an incomplete
route, not a successful arrival at C.

Status and camera evidence are retained outside the repository in
`Duck2/evidence/20260910T112708-full-route-AEBC`. Physical observations are pending;
software phases alone do not establish road containment or the actual turn.
No automatic retry or controller adjustment followed this run.

## 2026-09-10 — continuous left-curve retry accepted

After a fresh Go, ran the corrected continuous app controller from the user's
position at the start of the familiar left curve, using the selected A -> E -> B
-> C route and a 15-second observation limit. No further steering or speed
changes were made. The user reported: "It was perfect."

Evidence `20260910T112401-continuous-left-reference-retry` under local
`Duck2/evidence` contains 19 timestamped status samples and 19 camera frames.
All sampled lane estimates reported both boundaries. Commands progressed from
approximately equal wheels to left-turn requests, including 0.03/0.20, with
subsequent visual steering corrections. The routine reached its 15-second limit;
the final controller status confirmed manual stop and zero requested wheels.
This record contains requested-wheel status, not an independent measurement of
executed wheel motion or stopping distance.

The user's physical observation accepts this left-curve retry and resolves the
reported failure to turn at that curve. It does not establish completion of the
entire A -> E -> B -> C route. No automatic second run or Git operation occurred.

## 2026-09-10 — continuous-route first left curve: white-reference regression

The user reported stopping on the first left curve of the intended A -> E -> B
-> C route. Windows key SSH and the camera were reachable; deployed controller
SHA-256 matched local source. The retained Docker log shows a right request of
approximately 0.133/0.047 at 08:51:56 UTC, followed by absent lane estimates and
zero requests. Later status was manually stopped (`Run ended by Stop`), with
white-only boundary-gap loss. That later status does not preserve the original
stop cause. The stopped frame had no visible yellow divider. There is no full
synchronized recording of this failed run, so the entire physical trajectory
cannot be reconstructed.

The accepted September 8 left-curve recording used the original white V=170
threshold. The continuous launcher inherited V=150 from the later dim right
bend. On the identical accepted first frame, lowering V to 150 shifted the
white centroid about 33 pixels right and changed the requested correction's
direction. On today's user-repositioned curve start, the old lane error was
0.04097; the brighter reference gives 0.00331. This is a reproduced perception
calibration regression, not evidence of a connection or route-planning failure.

Added opt-in `road_white_reference_value=170` to continuous mode and its camera
preview. Ordinary two-boundary tracking uses the brighter mask when sufficient
ordered white support survives. Dim-only white and one-boundary fallback retain
the configured V=150 mask. Junction row geometry, red approach, settling and
active corner/junction phases retain their existing calibration. Speed, gain,
trim, manoeuvre timing, lane-loss timeout and watchdog limits were not changed.
Status exposes the reference value and whether it was used on the current frame.

Saved-frame replay checked 415 frames from the accepted left curve, sharp right
bend and three junction directions. It introduced no additional lane losses,
no junction-geometry changes and no active-phase lane-error changes. For the
88-frame left recording, mean absolute disagreement with recorded lane error
fell from 0.02355 to 0.00442. Ordinary-road estimates also change before other
manoeuvres; replay does not prove the resulting physical path or a complete
route. The six new regressions cover bright/dim reference selection, preserved
junction/corner phases, stopped preview, missing borders and invalid settings.
The native project check ran 430 tests (427 passed, three skipped), plus source,
link and launcher checks. The focused Noetic run passed 126 tests. An initial
focused invocation lacked the desktop module search path and was corrected.

The exact-source ROS integration run on laptop Docker encountered backward clock
steps and duplicate/out-of-order camera rejection; those failed runs are not
counted as passes. The timestamp safeguards were retained. A disposable ARM64
container on duck2 then built the current source and passed 142 focused tests
and the complete isolated ROS transport suite with synthetic images, network
disabled and no hardware access. Evidence: `20260910T091332Z-8971ff8b` under
`Duck2/bench-checks`. The temporary container was removed after verification.

The updated continuous application was deployed without sending Start or a
route. Its source hash matched the local file; status confirmed `awaiting_route`,
manual stop, zero requested wheels and sole project publisher. The live stopped
scene uses reference V=170 and detects both boundaries. Six stopped status samples
passed (maximum sampled controller frame age 0.2552 seconds), and normal, mask
and overlay views decoded. These samples do not establish a future timing bound
or physical curve-following success. The app connection is restored; a fresh Go
and user observation are required for the physical retry.

Evidence, camera captures and the pre-change deployed source are outside the
repository under local application data in `Duck2/diagnostics/20260910-left-curve`.
No physical retry or Git operation was performed during this diagnosis. The
managed-session onboard route containing only A -> E is expected: the laptop
holds subsequent validated turn instructions. It is not proof of an incorrect
destination selection. Physical containment still requires a supervised retry.

## 2026-09-10 — extended checks without the track

Completed the additional checks in [BENCH_CHAT_CHECKS](BENCH_CHAT_CHECKS.md).
The actual map-only companion passed a multi-junction simulation, destination
stop, reconnect without resumption and Stop after completion (five checks).
The production launcher and independent watchdog passed isolated ARM tests for
encoder stall, competing publisher and controller crash, after a bounded
initial-registration fix. Evidence: `20260910T081205Z-c266e8bd`.

The first ARM production-watchdog result (`20260910T080443Z-11c527ee`) was a
false pass caused by shell error handling being disabled by the ROS environment.
Its detailed log contradicts that result. The runner now restores error checking
and requires the specific completion marker; later failed attempts are retained
as failures. The final result includes all three actual fault checks. A one-core
limit proved insufficient for this production workload; the isolated production
check uses two cores, with the real stopped application temporarily suspended.

Native regression after the fixes: 424 tests, 421 passed and three skipped.
Source syntax, documentation links and launcher targets passed. All 77 live
follower parameters matched the production launcher. Application source hashes
and runtime dependency versions were checked against the local project.

The updated real controller was prepared again and left manually stopped with
exclusive ownership. With all synthetic workloads removed, 30 consecutive real
status/frame samples passed: normal/mask/overlay decoded, timestamps advanced,
camera remained valid, and wheel requests remained zero. Maximum sampled image
age was 0.051 seconds; maximum status-request latency was 0.422 seconds. These are
sampled results, not guarantees of future timing. Evidence:
`Duck2/release-checks/presentation-stream-soak.json` outside the repository.

No physical movement or Git operations occurred. Lane and turn settings were
preserved. The track-free preparation is complete; it cannot establish physical
route containment, stopping distance or detection under tomorrow's course lighting.

## 2026-09-10 — post-reboot presentation preparation

Windows strict-key SSH authenticated without a password prompt. ROS Noetic,
ARM64 architecture and the camera/wheel message fingerprints match the recorded
configuration. Hardware interface and ROS containers are healthy. Both encoders,
front range and IMU published readable samples; simulated ROS time is disabled.
Robot storage had 44 GB available and the sampled temperature was 36 C.
Battery charge was not established by these software checks.

The installed dashboard reports unhealthy because its health command invokes
missing `curl`. An independent HTTP request to its health endpoint returned
200 / Healthy. This does not block the companion's SSH/ROS services. No installed
dashboard or ROS software was modified.

Verification performed against current files:

- Native suite: 421 tests, 418 passed and three skipped; source/link/launcher
  audit passed. Desktop interpreter: 20 focused startup/UI checks passed.
- Isolated onboard suite: 60 tests passed, plus the ROS transport/integration
  checks, including stale camera, heartbeat loss, red stop, remote Stop, route
  progression, managed chat and shutdown. Evidence folder suffix:
  `20260910T074642Z-07593612`.
- Actual companion against isolated onboard simulation: all 11 interactive
  checks passed, including real 30-second deadlines and Stop on app close.
  Result: `20260910T095312-interactive-ui.json`.
- Stationary preview: nine normal/mask/overlay images decoded, timestamps
  advanced and sampled frame ages were below 0.5 s. Metadata:
  `20260910T074530Z-camera.json`.

Synthetic containers and their relay were cleaned up. Current source was staged
using `prepare_companion_driving.py`; preview was stopped, normal car-interface
remains stopped, and the real controller has exclusive wheel ownership. Driver
stop release occurred only during verified zero-output preparation. Final state:
`awaiting_route`, manual Stop, fresh camera, no reported fault, and 38 consecutive
executed-wheel samples at zero. The companion was opened and verified connected
with a rendered real camera image, no selected app route and placement unconfirmed.
The controller's displayed default route is not an authorized presentation route.

The real camera shows a tabletop. Its boundary-gap/lane-loss diagnostic is
expected here; it is not evidence of track readiness. Starting-lane/destination
selection, current-lighting inspection on the track, battery confirmation and
observed physical route performance remain pending. No physical run, Start,
Continue, steering retuning or Git operation occurred during this preparation.
Evidence and the temporary app-opening helper are outside the repository under
local application data in `Duck2/bench-checks` and `Duck2/release-checks`.

## 2026-09-10 — cleanup and startup audit

Current native suite: 421 tests, 418 passed and three dependency/platform skips.
The bundled desktop interpreter separately passed 20 focused startup/UI checks,
including Pillow-dependent camera tests. Python syntax, relative Markdown links,
Windows launcher targets, PowerShell parsing and shell launcher syntax passed.
The default Docker launcher was executed with network disabled and started no
application node. Local AMD64 and ARM64 builds passed using cached pinned layers.
An export of the staged project also passed all 421 native tests (three skips),
the source/link/launcher audit, and an AMD64 build from that fresh directory.

The revised tunnel check verifies both command and camera services and reports
failed/occupied tunnels instead of treating one command port as success. Live
startup revalidation was attempted but `duck2.local` no longer resolved during
this cleanup; no robot services were changed. Prior successful stationary and
isolated onboard results are preserved in [BENCH_CHAT_CHECKS](BENCH_CHAT_CHECKS.md).

Added current startup and documentation indexes, dependency manifests, and an
offline project-check command. Historical investigations are labelled rather
than presented as current launch instructions. Retained older tools that remain
tested, shared dependencies, or research evidence. Generated Python caches remain
ignored and excluded from publication; automated recursive cleanup was blocked.
Runtime licences and tested
controller/launcher behavior were preserved during cleanup. Git publication is
explicitly authorized for this cleanup phase.

## 2026-09-10 — local live-chat implementation

Added a laptop-owned map-validated turn queue, robot completion reconciliation,
straight-only speed profiles, deferred pauses, timed resume and 30-second red/
indefinite-pause deadlines. Existing launcher values and physical turn profiles
were retained. Documentation: [LIVE_CHAT.md](LIVE_CHAT.md).

The final native offline suite ran 408 tests: 407 passed and one was skipped.
Tests include the actual controller with mocked ROS,
synthetic images and an in-process laptop/controller turn-pause-turn sequence.
The native Tk checks cover the resized map, minimum-size chat input and Stop
delivery while another command holds the app lock. Two pre-existing navigation
tests were corrected to allow the existing steering slew limit; both failures
were reproduced in the pre-change source copy. An existing test's source read
was made explicitly UTF-8 for Windows.

After repairing Docker Desktop's stale runtime sockets, both AMD64 and ARM64
project images built successfully. The ARM64 import check loaded the new session
module with OpenCV 4.2. The complete isolated Noetic transport script finished
with exit code 0 on 2026-09-10 at 00:36:38 UTC in container
`duck2-live-chat-ros-check`, with Docker network mode `none`.
Its managed-session scenario exercised real local ROS messages for Start,
straight classification/profile selection, timed zero-output pause, automatic
resume and Stop. The existing camera freshness, lane/red stopping, heartbeat,
gateway and shutdown checks also completed. Synthetic camera input and wheel
topics belonged exclusively to that isolated local ROS master.

Two old transport assertions were updated to match the existing multi-row
outgoing-corridor diagnostic and the configured seven-second search window
after white-clearance entry. These assertion changes did not alter the
controller's turn timing or movement logic. Earlier failed invocations remain
in the container's cumulative log; the final invocation exited successfully.

The native suite was rerun after the transport assertion changes: 408 tests,
407 passed and one skipped. `test_ros_transport.py` is a separate executable
requiring Noetic and is excluded from native unittest discovery.
No robot deployment, robot camera subscription, robot command, physical movement
or Git operation occurred in this work.

Straight-profile values (0.09/0.10/0.11; active-profile wheel cap 0.12) and the
straight classifier still require supervised verification under course lighting.
They are not a new physical speed/containment calibration.

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
records junction phase/progress/deadline evidence, then allows a short bounded
lane-following window after confirmed reacquisition. The initial maneuver values are
provisional references and have not moved duck2 through an intersection. The
focused lane, navigation, readiness, supervisor and session suite passed 150
tests after these changes. The broader offline suite passed 282 tests with four
environment-dependent skips. The isolated ROS transport suite then passed in a
disposable ARM64 container with `--network none` and no hardware devices. It
confirmed continuous nonzero requests through an authorized unmarked crossing,
stable 0.3-second outgoing-lane reacquisition, route advancement, camera and
heartbeat stopping, and shutdown zero delivery. Physical work must begin with
stationary red-line calibration and one straight crossing.

### 2026-09-09: red-line calibration and junction startup race

Stationary samples used the frontmost part of duck2's front wheels as the
distance reference. The median detected red-line bottom fractions were
`0.68125` at 15 cm, `0.87083` at 10 cm and `0.95000` at 5 cm. Every inspection
kept physical motion disabled, confirmed zero driver feedback and restored
`/duck2/kinematics_node` as the normal wheel publisher. A provisional `0.86`
trigger was selected for the first moving approach; this is a bounded-test
setting rather than a production default.

Run `20260909T134914Z-junction-straight-086-fdef5361` did not start physical
motion. Route setup succeeded, but the supervisor published its `continue`
command before the new node had processed a fresh camera frame. The node
correctly rejected it with `Waiting for a fresh camera frame`; final wheel
feedback was zero and normal ownership was restored. Later status showed a
fresh complete incoming lane, confirming a startup-order race rather than a
lane-colour or wheel fault.

The supervisor now waits after route setup until status confirms a valid camera
age no greater than 0.25 seconds, both incoming boundaries, a finite lane
estimate, no red stop or fault, the expected route/index, and the route's manual
stop. Only then may it publish `continue`, while the external emergency stop is
still held. The revised focused suite passed 152 tests. The full isolated ROS
transport suite also passed in the verified ARM64 image with `--network none`
and no hardware devices. No physical retry occurred after this fix.

### 2026-09-09: first moving straight crossing and search timeout

The subsequent retry, `20260909T135827Z-junction-straight-086-retry-d1723996`,
did move. The user observed a stop about 7–8 cm before the red line and accepted
that distance, a roughly 1–2 second dwell, then forward movement with about
five degrees of left drift and a stop inside the intersection. Distances in
this session refer to the frontmost part of the front wheels, not the body.
Keep the bounded-test red trigger at `0.86`; do not infer an exact distance
calibration from it. The 5 cm stationary detection reached the bottom of the
analysed region, so its `0.95` fraction is not a precise unclipped landmark.

Recorded evidence shows a roughly 9.89-second release-to-stop window. The
controller fault was `Outgoing lane was not reacquired; position must be
reset`: its three-second search expired. Camera ages in the recorded status
were approximately 0.04–0.12 seconds; there was no reported encoder stall.
The unmarked straight-search phase requested equal `0.09` wheel commands.
Equal requests do not prove equal physical wheel speeds or explain away the
user's observed drift. Final zero feedback and normal publisher restoration
were recorded. This is a failed crossing, with provisionally accepted red-stop
placement, not a successful intersection traversal.

Saved images show the outgoing lane ahead while its markings are still mostly
above the lane-analysis region. The opposite lane's transverse red line is
visible beside the outgoing yellow divider. It is not substituted for a
longitudinal yellow boundary: doing so would corrupt the lane midpoint.

The supervised **straight** profile now permits seven seconds of outgoing-lane
search, instead of three. Left/right supervised profiles and the ordinary node
default retain three seconds. The entire physical session remains capped at
15 seconds, including approach and dwell, and may end before that search
budget is exhausted. Colour thresholds, stop trigger, wheel profiles and lane
steering are unchanged. The supervisor rejects a stale node reporting the old
straight-search timeout. Freshness, publisher ownership, encoder checks,
manual stop and the independent watchdog remain active.

Additional status fields report departure-red clearance, remaining search time
and the outstanding reacquisition gate (minimum progress, red clearance,
ordered boundaries, alignment, or stability). A pair must remain acceptable
for 0.3 seconds before route progress advances; normal lane-loss stopping then
resumes. Left and right crossing profiles remain provisional and require
separate tests; the accepted road-bend pivot is not intersection calibration.

Counterfactual replay of the recorded lane-status samples, using their recorded
junction settings, reproduced the old timeout. With seven seconds it remained
in reacquisition and requested alignment at the same point, with route index
unchanged. This was a replay of perception outputs, not a physical simulation
or proof of eventual exit: there are no later moving frames from that run.

For the next authorized straight test, use the same four-way approach, centred
in the incoming right lane, approximately 15 cm before the red line using the
front-wheel reference. Camera uncovered; cable held slack. The prepared
Windows launcher settings are `-JunctionTurn straight -Duration 15
-RedStopTriggerBottomFraction 0.86`. Without `-Go`, it performs only its
read-only startup check; add `-Go` only after a fresh movement authorization.
Observe road containment, outgoing-lane acquisition and stopping. The test
stops up to two seconds after straight-lane reacquisition, or earlier on a fault. Do not
automatically retry or extend an unsuccessful run indefinitely.

Verification after this change: all 155 focused supervisor, session,
navigation, lane and readiness tests passed. The updated node built against
the verified ARM64 ROS image. The isolated ROS suite passed with synthetic
images, `--network none` and no hardware devices, including a 5.5-second
unmarked sequence with the seven-second search override, stable reacquisition,
shutdown and recorder checks. Its first run failed the recorder's assertion
that every initial frame already has status metadata; the unchanged recorder
test passed on rerun without the two-CPU container quota. This indicates a
timing-sensitive check, not a demonstrated recorder fix. Evidence verification
remains required after physical runs. No physical motion or Git operations
occurred during this diagnosis and change.

### 2026-09-09: straight crossing reached the outgoing-lane transition

Run `20260909T141957Z-junction-straight-search7-fde4e7f3` used the same
four-way approach, red trigger `0.86`, seven-second straight search and
15-second session limit. The user observed a short approach, a stop before the
red line, a roughly 2–3 second dwell, a straight crossing, then about 5–10
degrees of rightward correction and a prompt stop approximately 10 cm before
the outgoing lane entrance. The user did not report crossing a road boundary.

The synchronized evidence confirms a healthy camera age of approximately
0.054–0.125 seconds, advancing encoders, no supervisor fault, final zero wheel
feedback and restoration of `/duck2/kinematics_node` as the sole normal wheel
publisher. The controller found no usable boundaries through the unmarked
area, first accepted an ordered pair at about 7.16 seconds after release, and
declared it stable at about 7.46 seconds with lane error still approximately
`0.17`. During the supervisor's one-second post-reacquisition window the error
converged through zero; recorded values ended near `-0.08`. The supervisor then
stopped by design at about 8.51 seconds. This explains the early physical stop:
it was recorded as a completed junction, not a search timeout or safeguard
fault. The visible right correction was converging centring behavior, although
the short remaining window did not let the robot enter the outgoing lane.

For the next **straight-only** trial, stable outgoing-lane acceptance now
requires absolute lane error below `0.10`, rather than `0.35`. The supervisor
then permits two seconds of lane following, rather than one, before stopping.
The recorded error sequence would first meet the stricter stable condition
about 0.8 seconds later; the extra settling window keeps the predicted stop
inside the existing 15-second session. This is a counterfactual timing check,
not a physical result. The seven-second unmarked search, `0.09` straight speed,
colour bounds, red trigger and ordinary launchers are unchanged. Left and right
junction trials retain the original `0.35` gate and one-second settling window
until they are tested separately.

All 155 focused tests passed after the change. They cover the new parameter's
validation, rejection of the previously accepted `0.17` alignment, acceptance
of a stable `0.08` alignment, stale-profile rejection and direction-specific
test isolation. The verified ARM64 ROS image built successfully and the full
isolated ROS integration process exited `0`, including junction traversal,
camera freshness, controller loss, recording, shutdown-zero and launcher
checks. The integration container used `--network none` and no hardware
devices. No physical motion or Git operations occurred while implementing and
verifying this adjustment.

### 2026-09-09: transverse red line during straight crossing

Run `20260909T144855Z-junction-straight-align010-settle2-81bf74b7` stopped
after approximately 3.83 seconds with final zero feedback, restored normal
wheel ownership and verified evidence. Its camera ages remained approximately
0.061–0.129 seconds and both encoders advanced. The controller fault was not a
camera, motor, encoder, ownership or deadline fault. After the departure line
had cleared, a transverse red marking re-entered the red stop region while
duck2 was still in the authorised entry phase. Recorded images show this is the
intersection/cross-traffic marking, consistent with the course layout.

During an explicitly authorised, bounded junction crossing, a red line that
reappears after departure is now recorded as `junction_red_reappeared` but does
not cancel the crossing. This permits the known transverse course marking and
does not use it as a substitute for an outgoing lane boundary. A red line that
never clears still prevents departure and ends at the bounded deadline. Route
progress still requires the ordered outgoing yellow-left/white-right pair,
alignment, stability and a valid camera. Manual Stop, camera freshness,
publisher ownership, encoder-stall detection and the independent watchdog
continue to stop the test.

Verification passed after this phase-specific change: 155 focused tests and
287 broader offline tests passed, with one platform-specific skip. The full
isolated ARM64 ROS integration process exited `0`. Its synthetic junction
sequence explicitly cleared the departure red line, presented a transverse red
line again during crossing, confirmed no fault and the diagnostic flag, then
reacquired the outgoing lane. The same run passed camera freshness, controller
loss, evidence recording, launcher cleanup and shutdown-zero checks. It ran
with `--network none` and no hardware devices. No further physical movement or
Git operation occurred while implementing this fix.

### 2026-09-09: straight-crossing stall diagnosis and revised bounded preset

Run `20260909T150156Z-junction-straight-cross-red-retry-863b262a` stopped
safely after the right encoder produced no progress while its wheel remained
commanded. The user observed rightward travel before the red line, the expected
stop and dwell, then progressively slower crossing motion until the robot could
no longer move; motor sound continued briefly. Synchronized evidence separates
two effects. Before the stop, lane following requested about `0.112/0.068`, so
the rightward approach correction was commanded. During the unmarked crossing,
requested and executed output remained `0.09/0.09` and the motor-register
readback remained stable while both encoder rates declined to zero. Camera data
remained fresh and missing borders were accepted. The encoder watchdog caused
the final safe stop; it did not cause the preceding slowdown.

The next straight-only bounded preset uses `0.15/0.15`, based on the earlier
two-second equal-wheel result with 216/215 encoder ticks and visually straight
travel. When a red line is already visible on a configured straight approach,
steering is capped at `0.01` to reduce entry yaw. During the unmarked straight
crossing, a limited cumulative encoder correction compares left/right progress;
gain is `0.12` and the adjustment is capped at `0.015`. This correction is
inactive until both wheels have advanced at least 12 ticks and is unavailable
when either encoder sample is stale. During outgoing-lane alignment, speed
scaling now preserves the existing `0.03` active-wheel floor.

These aids are disabled by default and are enabled only by the supervised
straight-junction preset. The supervisor checks every setting before release.
Replay of the failed encoder trace would request corrections from approximately
`0.147/0.153` initially toward `0.149/0.151`; this demonstrates bounded
direction and magnitude, not a physical result. Focused tests cover approach
limiting, correction direction and cap, alignment floor and the existing stop
gates. No physical test or Git operation occurred during this change.

Verification completed with 158 focused tests and 290 broader offline tests
passing; one platform-specific test was skipped. The complete ROS integration
suite passed after building the package in duck2's installed ARM64 Noetic base
inside a disposable container with `--network none`, read-only project mounts
and no hardware devices. Normal `car-interface` operation and sole
`/duck2/kinematics_node` wheel-command ownership were confirmed afterward.

### 2026-09-09: straight-junction containment diagnosis and row-matched control

Run `20260909T153355Z-junction-straight-015-balanced-5d288a1a` and the farther
approach run
`20260909T153759Z-junction-straight-015-balanced-far-approach-4940f263`
both stopped safely and restored normal wheel ownership. The second run
completed its software sequence in approximately 13.01 seconds with encoder
deltas 851/855 and final zero output. The user observed a marked rightward
drift before and after the red-line stop, crossing of the right white boundary
and a transverse red line, followed by visible correction toward the outgoing
lane centre. This is recorded as failed physical road containment even though
the old software criterion reported reacquisition.

Synchronized evidence showed that the controller initially requested about
`0.107/0.073`, which intentionally steered right. The straight-approach cap
did not activate until the red line entered its image region and then toggled
with red detection. The detector also combined yellow and white centroids from
different image depths: initial matched-row lane centres were near image
centre while the aggregate centroid requested a right correction. The global
`0.0075` trim added steering in the same direction. At the outgoing side, the
old test accepted distant boundary fragments and stopped two seconds after a
lane-error threshold passed without checking heading and centring together.
The stronger `0.15` crossing power did not show the earlier progressive stall.

The supervised straight-junction mode now uses yellow-left/white-right pairs
sampled at shared image rows. It requires at least three plausible pairs, a
substantial row span and near-field support, and rejects broad transverse or
isolated distant fragments. This produces separate image-space lateral and
heading errors. The dedicated approach is active from the start of an
explicit straight-junction test, uses a provisional target fraction of `0.49`,
and omits the global steering trim. A large trustworthy heading error captured
at the red stop blocks automatic departure and reports the reason.

During an unmarked straight crossing, the proven `0.15` power and limited
encoder balancing remain active. A trustworthy visual corridor clears the
encoder reference and takes steering ownership; if the view becomes unmarked
again, balancing restarts from fresh encoder counts. Outgoing reacquisition
requires near-field multi-row position and heading limits for 0.5 seconds.
After route progress advances, the node remains in a settling phase until
position, heading and steering are all stable for 0.6 seconds. The supervisor
no longer treats a fixed two-second post-reacquisition delay as proof of
alignment; a 15-second expiry after reacquisition is reported as alignment
incomplete.

Offline verification used duck2's installed ARM64 Noetic/OpenCV 4.2 runtime
in disposable containers with `--network none`, read-only project mounts and
no hardware devices. All 301 broader project tests passed with two
environment-dependent skips. The project image built successfully, and the
complete isolated ROS transport process passed camera freshness, shutdown
zero, route traversal, command transport, controller-loss, recording and
launcher-cleanup checks. The temporary verification image was removed after
the run. No physical motion or Git operation occurred during this
implementation.

### 2026-09-09: partial-row steering after failed straight crossing

Run `20260909T162210Z-junction-straight-row-geometry-6b5c7224` stopped
safely after approximately 13.11 seconds, with encoder deltas 790/795,
verified final zero feedback and restoration of `/duck2/kinematics_node` as
the normal wheel publisher. The user observed leftward drift before the red
line, the expected red stop and restart, continued leftward drift, crossing
of the outgoing yellow divider and then a stop. This is a failed physical
road-containment result. The controller reported `Outgoing lane was not
reacquired`; camera ages remained approximately 0.059–0.161 seconds, so the
stop was the bounded search timeout rather than stale camera data.

The synchronized status explains both parts of the failure. During the
approach, the matched-row detector usually found two correctly ordered lane
slices, but its near-field reacquisition rule required at least three slices
and a slice near the bottom of the analysed region. The approach therefore
discarded the two-row evidence and requested equal wheels, leaving the
observed leftward drift uncorrected. On the outgoing side, useful ordered
pairs appeared several seconds before the deadline, but they were still in
the far and middle image rows. They were also discarded, so the controller
continued approximately straight while the yellow divider moved toward and
under the camera.

The straight-junction mode now separates *steering confidence* from
*reacquisition confidence*. Two correctly ordered, plausibly spaced pairs at
separated image rows may request bounded steering during the approach and
outgoing search. Route progress still requires the previous stricter
multi-row fit with near-field support, acceptable lateral and heading error,
and the stability window. Isolated or broad transverse fragments remain
invalid. Ordinary lane following and the left/right junction presets are
unchanged.

Replaying the failed frames through the revised detector requests the
existing `0.01` maximum right correction on the approach where two-row
evidence is available. During outgoing search, correction begins on recorded
frame 69, about 9.53 seconds after motion release, and reaches roughly
`-0.067` to `-0.080` steering while the lane centre lies to the image right.
At crossing speed `0.15`, this corresponds approximately to `0.15` on the
left wheel and `0.07–0.08` on the right after the wheel cap. None of the
recorded frames satisfies the strict near-field criterion, so replay does not
claim successful reacquisition or physical success.

Verification passed in duck2's installed ARM64 Noetic/OpenCV 4.2 base using
disposable containers with `--network none` and no hardware devices: 190
focused lane, navigation, supervisor, readiness, session and steering tests
passed; the project image built successfully; and the complete isolated ROS
transport process passed camera freshness, red stopping, client loss,
recording, route traversal, combined-launcher shutdown zero and obstacle
checks. The wider unit run completed 304 tests with three platform-specific
skips; its only initial error was rerun successfully after using the built
project image required by the launcher test. No physical movement or Git
operation occurred during this diagnosis and implementation.

### 2026-09-09: perspective width and approach authority correction

Run `20260909T165423Z-junction-straight-partial-row-retry-b7b5af48`
failed physical containment: the user observed approximately 10–15 degrees
of left drift on approach, the red stop/dwell, departure along that heading,
crossing of the outgoing yellow divider, then correction toward lane centre.
The search timer stopped the run at 13.53 seconds. Final feedback was zero;
normal wheel ownership was restored and 91 frames were verified.

The approach requested approximately 0.10 left / 0.08 right, a rightward
correction, yet physical left drift continued. This establishes inadequate
correction on this run, not a reversed steering sign or a proven hardware
cause. The opt-in supervisor approach limit is now 0.03 rather than 0.01;
base speed, colour thresholds and ordinary launchers are unchanged.

The detector rejected nearby boundary gaps wider than 70% of image width,
then extrapolated two distant samples down to an unseen near row. It now
accepts ordered gaps up to 95% of image width and evaluates the fit within
its observed row span. Near-field support, alignment and stability are still
required to advance the route. With the recorded colour settings, saved-frame
replay finds four or five approach pairs and valid near-field support, and
recovers valid outgoing geometry around 12.47 seconds. At 13.27 seconds the
recording remains outside the centring threshold; no successful containment
or completion is inferred. The straight-only search allowance becomes nine
seconds; the independent supervisor still caps the entire session at 15
seconds. Missing camera data and all existing fault stops remain active.

Correction to the preceding replay description: the live preset's wheel cap
is 0.20, not 0.15. The latest run reached 0.20 left-wheel output during visual
correction. Earlier replay using default fixture limits was not an exact
reproduction of the live preset. Current replay explicitly uses the recorded
yellow and white lower bounds; it is perception evidence, not a simulated path.

Verification: 194 focused lane/navigation/readiness/steering/supervisor/session
tests passed in a network-isolated ARM64 Noetic container with no hardware
devices. After the timer change, all 84 supervisor/session tests passed again.
The full ROS integration suite was not rerun for this change. No physical
movement or Git operations occurred. The next physical run requires a new Go.

### 2026-09-09: accepted straight crossing and departure-bias rollback

Run `20260909T170225Z-junction-straight-perspective-fix-279b05f1`
was accepted by the user: straight approach, red-line stop and dwell, slight
leftward departure, outgoing-lane centring, and no boundary crossings.
Software completed the junction and confirmed zero feedback and restored ownership.

Run `20260909T171651Z-junction-straight-departure-bias-8c413ab8`
added a fixed -0.004 departure bias. The user observed hesitation, declining
speed and failure to enter the outgoing lane before stopping. During unmarked
travel, recorded commands stayed near 0.155/0.145 while encoder progress fell
to roughly 35–65 ticks/second, versus about 100 in the accepted run. Visual
outgoing steering began about eight seconds after departure rather than four;
it did request a right correction, but reacquisition timed out. Final zero
feedback and restored ownership were confirmed. Equal total encoder counts
do not prove road alignment or adequate speed.

The earlier claim that a small encoder mismatch clearly caused the left drift
was overstated. The accepted run also retained a modest image-based heading
error at the stop; these recordings do not distinguish inherited heading,
wheel-response variation and ground travel sufficiently to justify a fixed
bias. The cause of the slower motion at steady commands remains unestablished.

Removed the fixed departure bias and its parameter plumbing. SHA-256 checks
confirm the controller and supervisor now exactly match the accepted run:
controller `e53a480f5ac47332007b432f7621abd0ac5c08d1b7244e56835544245c75d1d7`;
supervisor `3fe3da38bac0a703486469630724f9bb73fa71ec32048e87f7b60320527e66a2`.
118 focused mocked-ROS navigation, supervisor and readiness tests passed using
local temporary OpenCV 4.12.0 / NumPy 2.2.6 dependencies. This is not a new live
ROS or physical validation. No robot or Git operations occurred during rollback.
A fresh Go is required before a confirmation run; remaining minor drift is not
claimed fixed.

Run `20260909T172430Z-junction-straight-restored-baseline-b7dbc3de`
confirmed the rollback. The user observed a straight approach, red-line stop,
minor departure oscillation without a boundary crossing, smooth outgoing-lane
centring, and a centred stop. The controller completed the junction in 12.25
seconds. Both encoders recorded 867 ticks over the supervised window; camera
data remained fresh, final feedback was zero, and normal wheel ownership was
restored. Straight intersection traversal is provisionally accepted for this
approach and lighting.

The next supervised milestone is the left exit from the same intersection.
Its isolated profile remains provisional: 0.5 seconds of straight entry,
followed by left/right wheel requests of 0.03/0.15 for the calibrated phase and
while markings remain absent. It requires both outgoing borders and stable lane
error before route progress. Camera freshness, encoder-stall detection,
publisher ownership, the crossing deadline, manual stop, and final zero remain
active. No physical left-intersection result is claimed yet.

Run `20260909T173044Z-junction-left-first-7154a416` failed its outgoing-lane
deadline. The user observed a slight rightward approach drift, the expected red
stop, the beginning of a left turn, decreasing physical speed, and then a
stop before the turn completed. Telemetry shows the 0.03/0.15 turn request
continued through missing markings; this was not a lane-loss or encoder-stall
stop. Both outgoing borders first became usable about 2.9 seconds into the
three-second search, leaving only about 0.1–0.2 seconds before the fault and
less than the required 0.3-second confirmation. Final feedback was zero and
normal ownership was restored.

The supervised left profile now uses the proven row-matched approach steering
before the red line, a five-second left-only outgoing search allowance, and the
unchanged 0.03/0.15 left-turn pair. When outgoing borders appear, alignment
normalizes its faster wheel to the profile's established 0.15 cap instead of
leaving both requests near the unreliable 0.09 range. Ordinary launchers and
the accepted straight-intersection settings are unchanged. This remains an
unverified left-intersection candidate until a new supervised run passes.

### 2026-09-09: reject premature left-intersection handoff

Run `20260909T173938Z-junction-left-approach-and-reacquire-fix-a45eeed8`
failed. The user observed an improved straight approach, the normal red stop,
and the desired gradual left turn, followed by slowing and an unwanted right
turn halfway across the intersection. Final wheel feedback was zero and normal
publisher ownership was restored. The supervisor reported lane loss at 8.55
seconds of its supervised window; camera freshness remained within the limit.

The controller accepted centroid detections for 0.3 seconds as an outgoing lane,
then advanced the route and commanded right steering. The synchronized frame
still looked across the intersection. Its matched-row geometry reported no valid
corridor or near-field support; occasional isolated far-row pairs were insufficient.
The wheel requests changed from 0.03/0.15 to roughly 0.187/0.03 before lane-loss
stopping. This establishes a premature visual handoff and commanded reversal of
steering. It does not establish a separate hardware cause for perceived slowing.

For the opt-in visual junction test, left-turn reacquisition now requires the
existing matched-row corridor, near support, heading and lateral limits, and
stability interval. Without a nearby corridor and acceptable heading it retains
the authorized 0.03/0.15 arc. Once heading is acceptable, matched-row visual
centering can take over at the profile's outer-wheel cap; lateral alignment must
also stabilize before route advancement. The five-second search and total
session deadline remain; failure to find the lane still stops the maneuver.
The accepted straight crossing and ordinary launcher settings are unchanged.

124 focused navigation, supervisor and readiness tests passed. Replaying 24
recorded perception/status samples through the revised search retained the left
arc and rejected the original premature handoff. Regression tests cover distant
fragments, diagonal corridors, interrupted confirmation, alignment power,
deadline stopping and lane loss after completion. Replay is not a prediction of
the new camera trajectory. No new isolated live-ROS validation, physical run or
Git operation occurred during this fix; left-intersection success is pending.

### 2026-09-09: tighten the guarded left arc

Run `20260909T174803Z-junction-left-guarded-reacquisition-0779e499`
had the best left-intersection behavior so far by the user's observation:
centered approach, normal red stop, and smooth left turning, but insufficient
curvature. It stopped facing the white corner rather than the outgoing lane.
The saved final image supports that observation. During search the controller
held 0.03/0.15, with no valid matched-row outgoing corridor and no premature
completion. It stopped on the five-second search deadline at 11.61 seconds of
the supervised window. Camera age was 0.040–0.146 seconds; final and post-stop
feedback were zero, and normal kinematics ownership was restored. This remains
a failed traversal, not a failed stopping test.

The next test-only left profile uses center speed 0.105 and bias 0.075,
requesting left/right 0.03/0.18. The commanded difference increases from 0.12
to 0.15 (25%); this is not a measured 25% increase in physical curvature.
The inner wheel keeps its rolling command. The opt-in left entry is capped at
the existing 0.09 base speed, so its 0.5-second straight entry is unchanged.
Visual alignment uses the new outer-wheel cap. Approach steering, red threshold,
search and session deadlines, corridor acceptance, and independent stopping
checks are unchanged. No timeout extension or zero-inner-wheel pivot was added.

125 focused navigation, supervisor and readiness tests passed, including the
new turn pair, unchanged entry, continuous unmarked arc, alignment output cap,
and deadline stopping. No physical run or Git operation occurred during this
adjustment. Its turning radius and road containment require a fresh supervised
test; replay alone cannot establish the trajectory produced by increased power.

### Pending: bounded sharp right from an unmarked intersection

The sharp right after a red-line stop is implemented as a junction phase rather
than the ordinary road-corner detector. The road-corner detector requires a
yellow divider and is therefore intentionally disabled during junction tests.
After the red stop and two-second dwell, the right profile drives straight for
the existing 0.5-second entry, then commands the proven right-pivot pair:
left wheel 0.20 and right wheel 0.00. Missing yellow and white markings are
expected during this authorized, bounded phase.

The pivot no longer hands off merely because a colour fragment appears. It must
first find a nearby matched-row outgoing corridor with acceptable heading and
lateral alignment for the configured stability interval. Until then, it keeps
the selected pivot. Once this evidence appears, regular visual lane centering
takes over. A five-second outgoing-lane search deadline and the total
15-second supervisor deadline remain. Fresh camera data, encoder-stall
detection, publisher ownership, manual stop, watchdog and final zero remain
mandatory. No physical right-intersection result is claimed yet.

127 focused navigation, supervisor and readiness tests passed. The new cases
cover false or distant corridor fragments, diagonal corridor rejection,
continuous right pivot during missing markings, stable outgoing-lane handoff,
and deadline stopping. No robot movement or Git operation occurred while
preparing this test.

### 2026-09-09: right entry follows the white-boundary end

Run `20260909T180059Z-junction-right-bounded-pivot-ca318c57` stopped
with left-encoder stall detection at 7.03 seconds. The user observed a normal
approach and red stop, but the right pivot started before a wheel reached the
red line. The previous implementation used a fixed 0.5-second entry; it did
not wait for the incoming white boundary to disappear. Final wheel feedback
was zero and normal control was restored. Changing entry timing does not prove
that the observed pivot stall is resolved.

The opt-in right-junction entry now requires an observed white boundary followed
by 0.15 seconds of continuous absence. It then travels straight at the existing
base command for one second before requesting 0.20/0.00. White reappearance
before pivoting resets the absence/clearance sequence. No yellow marking is
required during this authorized crossing. Time spent waiting or clearing does
not consume the minimum pivot interval or the search timer. The existing
wall-clock and supervisor deadlines still include the entire maneuver.

The supervisor checks the new entry-policy diagnostic before release, preventing
an older fixed-entry controller from running this test. Added entry-stage status
distinguishes waiting for the white boundary, confirming its disappearance,
forward clearance and pivot start. 129 focused tests passed, including observed
boundary loss, flicker reset, the full clearance interval, no premature search,
and deadline stopping without an observed boundary. No robot or Git operations
occurred during this change. Physical entry clearance and pivot completion
remain unverified until a fresh Go.

### 2026-09-09: shorten right-entry clearance

Run `20260909T180713Z-junction-right-white-end-entry-53ffb7eb` executed
the boundary-based entry, but the user observed excessive forward travel before
the pivot. Status timing relative to departure shows white loss around 1.8 s,
forward clearance around 1.9 s, and pivot start around 3.0 s. The additional
one-second advance compounded the distance already travelled while waiting for
the white boundary to disappear. The saved pivot-start frame shows cross-lane
red/yellow markings in view.

Those markings did not stop the run: it remained in turning/searching with
0.20/0.00 requested and no red-stop latch. The supervisor stopped for left-wheel
stall at 8.88 s of its window. Final feedback was zero and normal ownership was
restored. This is evidence of missing encoder progress under command, not proof
of the underlying mechanical or electrical cause.

Reduced only the opt-in right clearance from 1.0 s to 0.25 s after the existing
white-loss confirmation. This should initiate pivot approximately 0.75 s earlier
for the same incoming detections; the changed physical path still needs testing.
The supervisor verifies both the revised policy and duration before release.
129 focused tests passed, including the revised timing bounds, flicker handling,
pivot/search sequencing and stopping regressions. No physical or Git operations
occurred. The repeated pivot stall remains unresolved; its safeguard was not
weakened and no claim of successful right traversal is made.

### 2026-09-09: end the right pivot on a visible outgoing corridor

Run `20260909T181223Z-junction-right-short-clearance-4bf0fa77` had the
user's accepted entry and turning path, but continued turning past the intended
outgoing lane. It stopped on outgoing-lane timeout at 13.27 s. Camera and encoder
feedback remained healthy; final output was zero and normal ownership restored.
The user observed the robot centered in front of the intended lane before it
overturned. This is failed handoff, not accepted intersection completion.

Recorded geometry detected three ordered yellow/white pairs at ROI fractions
0.18, 0.36 and 0.54 while the lane was ahead. The old visual-steering gate required
near support at 0.72 and heading error below 0.08. At the first near-supported
sample the heading error was -0.0839, so pivoting continued. A brief visual
handoff followed, but loss of near support restarted the pivot. The saved
frame-00069 visibly shows the intended outgoing corridor.

Changed only the opt-in right-turn visual handoff: after the unchanged minimum
pivot and departure-red clearance, three paired rows spanning at least 0.36
and reaching 0.54, with lateral/heading errors below 0.15, must persist for
0.10 s. This latches visual entry using the existing row-based steering and
alignment wheel cap. Near-row gaps cannot resume fixed pivoting. Loss of all
usable row geometry after the latch faults to zero rather than pivoting blindly.
The original strict near-field alignment/stability checks still govern route
advancement; search and overall deadlines remain unchanged.

Preserved white-loss confirmation, 0.25 s clearance, 0.20/0.00 pivot commands,
minimum pivot duration, colour thresholds, and straight/left controllers.
Added `junction_right_visual_entry` status evidence. Recorded-geometry replay
switches at monotonic 21974.9479, before the previous brief handoff around
21975.3454, without advancing the route. Replay is not a simulation of the changed
physical path. 131 focused navigation, supervisor and first-test-readiness tests
pass, including midfield recognition, flicker, near-row dropout, complete visual
loss and stable completion. No live ROS integration run, robot movement or Git
operation occurred during this fix. Physical lane entry remains pending a Go.

### 2026-09-09: pivot stopped before outgoing handoff was reached

Run `20260909T182312Z-junction-right-outgoing-handoff-285af70e` failed
before testing the revised handoff. The user observed less than about one second
of turning, followed by stationary motor sound. Status remained `crossing` /
`turning`, with `junction_right_visual_entry=false` throughout. There was no
navigation fault or red-stop latch at the failure; the camera remained fresh.

Relative to movement release, pivot output began around 7.15 s. The left encoder
ceased advancing around 7.4 s, while acceleration was still raising the left
command toward 0.20; the right command was zero. By 7.93 s, stable motor-register
snapshots showed channel-8 PWM 1584/4096 and channel-13 PWM zero, identical to the
earlier sustained pivot in `20260909T181223Z-junction-right-short-clearance-4bf0fa77`.
The left count remained unchanged through 8.2 s. The supervisor stopped at
approximately 8.23 s for missing left encoder progress. Final zero feedback and
restored normal publisher ownership were verified by the run report.

This is an intermittent pivot-start failure, not evidence that outgoing-lane
detection ended the pivot prematurely. Register agreement confirms programmed
output, not delivered motor current or torque. The existing acceleration ramp
leaves the outside wheel below full pivot command for roughly 0.7 s after the
inside wheel is stopped; this is a diagnostic hypothesis, not an established
root cause. The successful comparison also used this ramp. Do not infer a
hardware defect or blame placement from these records.

Retain the handoff change, entry timing, wheel caps and stall safeguard. No
movement-code change is justified by this comparison alone. A separately
authorized bounded pivot-start diagnostic is needed before claiming a physical
fix; compare ramp behavior and encoder progress without changing lane detection
at the same time. No new physical test or Git operation occurred during review.

### 2026-09-09: prepared direct-start pivot diagnostic (not yet run)

Added the explicit `-GroundStrongPivot` launcher option: fixed 0.20 left /
0.00 right, at most two seconds, through the existing supervised fixed-output
path. It starts directly at that pair rather than using the follower's
acceleration ramp. Ordinary launchers and intersection logic are unchanged.
The runner and robot supervisor independently reject durations above two
seconds; profile combinations and missing Go are rejected. The Windows launcher
defaults this option to two seconds. Without Go it performs only a startup check.

After fresh user authorization, from the repository in Windows PowerShell:

```powershell
.\tools\Start-Duck2-GroundTest.ps1 -GroundStrongPivot -Duration 2 -Label pivot-direct-start -Go
```

Place duck2 upright on the same track surface in a clear area with room for a
two-second right pivot, camera uncovered and cable slack. This is not an
intersection traversal or a lane-contained maneuver. Observe whether the left
wheel starts immediately, maintains rotation, and stops promptly; the right
wheel should remain unpowered. No automatic retry follows. Existing camera
freshness, ownership checks, encoder-stall stop, independent watchdog, explicit
zero, restoration and verified evidence-copy procedure remain active.

Compare requested/executed commands, encoder progress and motor registers with
the failed ramped start. A successful direct start supports further investigation
of the ramp but does not isolate it causally: placement, elapsed running time and
starting from rest differ. A failure must not cause automatic power escalation
or removal of stall detection. Preserve raw evidence outside the repository.

165 focused session, supervisor, navigation and readiness tests passed; Windows
launcher syntax parsed successfully. The startup-success text emitted by the
session tests is mocked, not a live connection verification. No robot connection,
movement or Git operation occurred during preparation.

### 2026-09-09: direct-start pivot also failed; ramp is not required for failure

Run `20260909T183231Z-pivot-direct-start-6ae2d656` sent constant
0.20/0.00 directly through the supervised fixed-command path. The user heard
motor-like beeping and observed no movement. Both encoder deltas were zero.
The supervisor stopped at 1.095 s for left-wheel stall, before the two-second
deadline. Final zero feedback and normal ownership were restored; 17 frames
and telemetry were copied and verified outside the repository.

Stable register snapshots during the command show left PWM 1584/4096, forward
direction, and right PWM zero, matching the previous successful pivot. A fresh
read-only inspection of the installed Dagu driver and HATv3 source confirms
that 0.20 maps to floor(0.20 * 195 + 60) = 99, then 99 * 16 = 1584.
The HAT frequency setup is 1600 Hz. No installed source was changed or hardware
driver object instantiated. Commands/registers are not measurements of motor
current or supply voltage.

This failure occurred without the follower or its ramp, and therefore cannot
be fixed merely by removing that ramp or relaxing outgoing-lane recognition.
The specific cause below the programmed-output layer remains unresolved. Do
not label it a confirmed mechanical obstruction, hardware defect, or low
battery. No movement-code change or automatic power escalation is supported
by this test alone. Next useful isolation is the same short command with the
wheels lifted, under new authorization, to distinguish current unloaded response
from the failed loaded start. Historical lifted tests do not measure the current
condition. Stop after that test for observation; do not retry automatically.

No new movement, motor-service call, ROS installation change or Git operation
occurred during this diagnosis. The outgoing handoff fix remains pending a
physical run that actually reaches its activation conditions.

### 2026-09-09: right handoff was incorrectly gated behind the pivot timer

Run `20260909T183700Z-junction-right-handoff-retry-ff6c45c6` reproduced
overturning. The user accepted the approach, clearance and turn path, but saw
nearly 180 degrees of rotation instead of entry into the visible outgoing lane.
The supervisor ultimately stopped on left encoder stall at 10.624 s of the
motion window (including red dwell); this is not continuous driving duration.
Final zero feedback and restored ownership were verified; 84 frames retained.

Recorded lane geometry at 7.71 and 7.81 s already met the visual-entry criteria:
three ordered paired rows through ROI fraction 0.54, span 0.36, lateral errors
0.1101/0.0708 and heading errors -0.1250/-0.1109. Frame-00067 confirms the
outgoing corridor. However, navigation called the handoff helper only after
the fixed 1.6-second pivot interval, around 8.1 s. By the next confirmation
sample the lateral error exceeded 0.15, so the latch never activated. The prior
helper-focused replay missed this enclosing state-machine timing gate.

The opt-in right controller now evaluates stable corridor entry during the
pivot, after 0.30 s initial pivot progress and departure-red clearance. Its
existing 0.10 s confirmation and geometry bounds remain. A confirmed corridor
transitions to visual reacquisition immediately, which persists even before the
old pivot interval expires. Strict near-field stability still governs route
advancement. Without a qualifying corridor the original timed search behavior
remains; deadlines and stall protection remain authoritative.

Preserved approach, white-loss detection, 0.25 s forward clearance, 0.20/0.00
pivot target, wheel acceleration, color values, and straight/left logic. Added
state-machine tests for the recorded early corridor, subsequent visual control,
departure-line rejection and deadline stopping. 167 focused tests passed.
Test startup-success output is mocked, not a new live check. No physical test,
robot change or Git operation occurred during this fix. Earlier intermittent
pivot-start stalls remain a separate unresolved limitation.

### 2026-09-09: passable right traversal; minor tracking trim

The user assessed `20260909T184145Z-junction-right-early-handoff-33cfe494`
as passable: no boundary crossing, with rightward drift during approach,
departure and outgoing alignment, followed by a left correction near the white
border. Preserve this physical assessment. Software completion did not pass:
at 12.302 s including dwell, geometry loss after visual entry caused a fault.
Camera remained fresh; zero feedback and normal ownership were restored.
The passable node SHA256 is
`bdd803e20961301ccda61ffdfdef048347b3a0e655ca9e2a2a1b39e701b27719`;
the run's staged source and recorded settings remain on the robot.

The initial command was approximately 0.1004/0.0796, a right correction from
row geometry. Global steering trim is not used in this approach. Forward entry
requested equal 0.09/0.09, so its reported drift is not proof of a commanded
differential. At visual handoff the inner wheel ramp took about 0.9 s to catch
the outer wheel, prolonging right rotation. Avoid a broad retune of this accepted
path; retain acceleration limits and outgoing detection for this small trial.

Added optional `junction_right_tracking_trim`, default zero, validated within
[0, 0.01]. Only the bounded right-junction test sets 0.003. It subtracts 0.003
from the left request and adds 0.003 to the right during approach, forward entry
and visual alignment, respecting the active-wheel floor and cap. Forward entry
therefore requests 0.087/0.093; an initial 0.1004/0.0796 becomes 0.0974/0.0826.
Clipping may reduce the applied correction. This is provisional compensation,
not measured motor calibration or a guarantee of centered travel.

The zero-inner-wheel pivot is excluded. Pivot strength, clearance, recognition,
handoff stability, deadlines, stops, ordinary launchers and left/straight tests
are unchanged. Status exposes the trim. Set it to zero to recover the previous
tracking behavior. 169 focused tests passed, including trim scope, stop/pivot
exclusion, wheel bounds and forward-entry behavior. Startup test output is
mocked. No physical test, robot change or Git operation occurred during tuning.

### 2026-09-09: remove trial trim and preserve steering during right handoff

The user reported `20260909T184746Z-junction-right-small-trim-188a3ad5`
still drifted right, then corrected left and crossed the yellow divider. Unlike
the previous passable run, this is failed physical containment. The supervisor
stopped for corridor loss at 11.945 s including dwell; camera stayed fresh and
final output was zero. Evidence was verified locally (88 frames).

At visual entry, published commands were approximately 0.197/0.016; the right
wheel ramp took about one second to catch up. Independent increase limits
preserved the previous pivot's outer-wheel power, so actual steering initially
differed substantially from the visual request. The small constant trim did
not address this transient. It is disabled in the bounded preset (parameter
zero); ordinary launchers already default to zero.

Only during opt-in right visual reacquisition, publication now applies a common
scale to both positive wheel requests when either increase would exceed the
existing acceleration limit. This preserves the desired wheel ratio while
respecting each wheel's increase limit and immediate stopping. It can reduce
the outer wheel during inner-wheel startup; physical restart speed remains
unverified. Fixed pivot, approach/clearance timing, perception gains and left/
straight handling remain unchanged.

Saved frames also show approach to the next red line while still reacquiring;
red was visible from approximately 11.4 s but ignored as intersection evidence.
After right visual entry and 0.30 s of a continuous valid near-field corridor
with red absent, subsequent red detection now latches zero with alignment
incomplete. It does not advance the route or authorize another intersection.
The existing red threshold is unchanged. A flickering corridor or persistent
cross-traffic red cannot arm this check. New status records rearming.

172 focused tests passed, including ratio-preserving acceleration, immediate
stop, stable red rearming, and rejection of red/geometry flicker. No live test,
robot modification or Git operation occurred. This addresses demonstrated
command-transient and stop-state faults; a successful physical correction is
not yet established, and the earlier intermittent pivot stall remains separate.

### 2026-09-09: bounded encoder assistance for right outgoing alignment

Run `20260909T185435Z-junction-right-ratio-ramp-a06ea30f` improved entry by
the user's observation, but subsequently crossed the yellow boundary to the
left. This remains failed physical containment. Corridor loss stopped the run
at 12.377 seconds including dwell; final zero and restored normal ownership
were recorded. The downloaded evidence contains 92 frames.

During late alignment the camera requested right correction, approximately
0.20 left / 0.12 right, while a one-second encoder interval showed 111/103
ticks. Similar wheel rotation despite unequal requests suggests weak realized
correction; it does not establish calibrated ground heading or motor speed.

Added opt-in `junction_right_encoder_assist` (default false), enabled only in
the bounded right-junction preset. During visual reacquisition with trustworthy
geometry and fresh, closely timed encoder samples, a 0.25-second measurement
window compares normalized wheel-count differential with the requested
differential. This command ratio is a provisional reference, not a calibrated
velocity model. Insufficient differential reduces the slower requested wheel
by at most 0.015 while respecting its active floor; outer-wheel power is never
increased. The adjustment is proportional and does not accumulate.

Straight requests, direction changes, steering resets, invalid geometry,
stale/reset counters and other driving phases clear the assistance. Stationary
or implausibly jumping counters receive no new compensation. Existing stall,
freshness, ownership and watchdog stopping remain active. Approach, clearance,
fixed pivot and ordinary launchers are unchanged; constant trim remains zero.
Status exposes whether assistance is enabled and its current adjustment.

174 focused offline tests passed, covering bounded directional assistance,
reset/stale data, adequate measured correction, stall and pivot exclusion,
alongside navigation, supervisor, session and readiness regressions. Session
startup messages in this suite are mocked, not live verification. Physical
effectiveness remains unverified. No movement or Git operations occurred.

### 2026-09-09: restore right outgoing-alignment steering authority

Run `20260909T190400Z-junction-right-encoder-assist-59674638` failed physical
containment: the user observed departure from the pivot still pointing left,
then travel over the outgoing yellow boundary without sufficient correction.
The supervisor stopped for outgoing corridor loss at 11.817 seconds including
dwell. Camera age was 0.053–0.253 seconds; zero feedback and restored normal
ownership were confirmed. All 89 downloaded frames were verified.

Recorded geometry at visual entry had lateral error approximately 0.077 and
heading error -0.112. Later values reached 0.399/-0.034. These are normalized
image estimates, not physical angles. The controller requested right steering,
and encoder assistance reached its 0.015 bound, but containment still failed.

Code review identified asymmetric saturation in right visual alignment:
`straight_visual_wheels(cap)` treated the outer-wheel cap as centre speed.
For steering -0.08 with cap 0.20, mixing yielded 0.28/0.12 and clipping yielded
0.20/0.12, halving the requested differential. The subsequent scaling did not
restore it. This weakens camera correction after early visual pivot exit.

Only right outgoing visual alignment now reserves steering headroom: centre
speed is outer cap minus absolute steering, then wheels are centre minus/plus
steering. The same example produces 0.20/0.04. Steering is bounded by the
existing 0.03 active-wheel floor (maximum differential 0.17 at cap 0.20).
Equal requests remain 0.20/0.20; encoder assistance and the ratio-preserving
acceleration limit remain bounded. No extra fixed pivot time, perception/gain
change, constant trim, increased outer power or reduced stopping protection
was introduced. Left/straight junction control and approach are unchanged.

175 focused offline tests passed. Added regression cases use recorded
alignment errors, mirrored errors, straight alignment and both flip settings;
they verify full attainable differential, the floor and outer cap. Existing
handoff, acceleration, stop, supervisor and session checks passed. This fixes
the demonstrated wheel-mixing defect; physical alignment remains unverified.
No new movement, robot deployment or Git operations occurred during this fix.

### 2026-09-09: right crossing accepted and continuous mode prepared

Run `20260909T190932Z-junction-right-full-alignment-4509f484` stopped at
12.630 seconds when the next red line appeared before software alignment had
completed. The user accepted the physical result as passable: duck2 remained
inside both outgoing boundaries, although it entered with a visible leftward
heading and did not fully centre. This is a passable individual crossing, not
proof of a repeatable or continuous route. Camera age was 0.062–0.166 seconds,
final zero was confirmed, normal ownership was restored and 93 frames were
preserved.

Added `lane-continuous`, which combines the current tested lane settings,
automatic red-line dwell/crossing, direction-specific junction profiles and
sharp-right road-bend recognition. Smooth curves remain under ordinary visual
lane following. Sharp-right recognition is now permitted with route mode only
while navigation is in `following`; it cannot activate during a red stop,
crossing or junction reacquisition. Obstacle detection and passing are disabled.

The Windows companion now sends a confirmed A* route and Continue through the
high-level gateway, displays live status and offers a direct Stop. Free-form
live chat is disabled. The gateway adds ROS wheel-publisher ownership to status,
and the desktop rejects Start unless the lane follower is the sole publisher.
The selected destination red line, camera/lane faults, encoder stalls,
heartbeat loss, ownership conflict and maneuver deadlines still stop motion.

191 focused controller/app tests and one native Windows UI lifecycle test
passed. The broader non-ROS suite ran 342 tests with four platform skips; its
only load error was the expected absence of `rospy` outside the project image.
No continuous physical route was run, no robot runtime was changed, and no Git
operation occurred.

For continuous routes, a right exit may lead quickly to the next red line. Once
the existing right-turn guard has observed a valid near-field outgoing corridor
for 0.30 seconds with the departure red cleared, a subsequent red line now
commits exactly one route edge and becomes the next junction stop (or the final
destination). It records `reacquired_at_next_red` with alignment
`contained_not_settled`. Red seen before that stable corridor still cannot
advance the route. Tests cover final and intermediate destinations and prevent
one persistent red image from advancing twice.

## 2026-09-10 — Immediate live-chat pause and straight-only pause scenario

- User requested an immediate three-second chat pause on the straight after the
  A -> E curve, followed by stopping and ending at the next red line.
- Existing local chat turn queue and slow/normal/fast profiles are retained.
  Pause now publishes zero in the command callback, without waiting for straight
  classification. Its monotonic countdown preserves the current turn and queue;
  paused time is excluded from maneuver timing. Camera/heartbeat/stall watchdogs
  remain active. Stop/fault cancels resume.
- Added the same companion's `--pause-check` option and
  `laptop/Start-Duck2-PauseCheck.cmd`. It prefills but never sends the chat text,
  uses A -> E, and requires the user's Start. The controller's
  `stop_at_next_red` flag ends the check at that line and rejects late departure.
- 75 focused native checks passed; source, documentation links and launchers
  checked. Focused HTTP/ROS checks in a local network-disabled container passed:
  three profiles; immediate zero without straight classification; three-second
  pause measured at 3.057 seconds in synthetic wheel requests; healthy resume;
  next-red completion with zero output and rejected late Resume.
- The broader isolated ROS script stopped earlier at its sharp-right-wheel
  assertion (test_ros_transport.py, line 271), outside the changed chat path.
  No turn/curve launcher settings were changed to address that separate check.
- Current source hashes match the staged robot code. Preparation verified
  awaiting_route, manual stop, zero requested/executed output, a fresh camera
  and exclusive lane-follower ownership. No physical run was initiated here.
- Earlier source is preserved in the local Duck2 backups directory. Logs and
  preparation status are in the local diagnostics/immediate-chat-pause-20260910
  directory. No Git operations. User-observed pause/resume/red-stop results
  remain pending. Automatic review blocked opening the app from this task;
  the prepared launcher can be opened by the user.

## 2026-09-10 — Successful pause report; live junction override preparation

- User reported the A -> E three-second pause test successful. This is physical
  confirmation from the user, in addition to the earlier isolated ROS results.
- Prepared a new companion scenario on the straight after the A -> E curve,
  selecting A -> E -> B -> C (initial future turns: left, right).
- User-entered "go straight at the next junction" replaces both planned turns
  with one straight instruction. Verified map transition: A -> E to E -> C.
  At C the queue is empty and the robot holds for a direction. "Go left"
  selects the legal exit C -> B; no message is sent automatically by setup.
- Red-line instruction waiting is now 60 seconds from arrival. Invalid input
  and ordinary status polling do not restart the wait. Its expiry ends the run;
  a late instruction cannot restart it. Indefinite chat pauses retain their
  separate 30-second limit; requested pause durations and driving profiles are
  unchanged.
- Added laptop/Start-Duck2-JunctionChatCheck.cmd (--junction-chat-check). This
  opens the existing companion, sets the route, shows live camera/chat and
  prefills an unsent first message. Start remains a user action. The app checks
  that the controller advertises the 60-second wait before starting this mode.
- 77 focused offline checks passed, including the actual controller and laptop
  queue for the exact A/E/C sequence, waiting past 30 seconds, accepting left
  at 55 seconds and rejecting departure after 60 seconds. Source, doc links
  and Windows launcher checks also passed. No movement logic was retuned.
- Updated source was prepared on duck2; controller remained awaiting_route,
  manually stopped with zero output and current camera frames. App reopened
  for the user. No physical run or chat instruction was started by preparation.
  Physical straight-at-E and commanded-left-at-C observations remain pending.
- Recovery source is saved outside the repository in the local Duck2 backups
  directory. No Git operations occurred.

### Companion presentation cleanup after the successful pause check

Removed both scenario launchers' prefilled chat input and scripted test narration.
The junction scenario displays the normal app title, selected route and normal
live status. The user types and sends every message. Five existing native UI
checks passed. Confirmed unchanged parser recognition for "stop for 3 seconds",
"go straight at the next turn", "left", and "go left". "3s" and misspelled
commands are not corrected automatically. The parser remains a fixed grammar
with validated route/queue context. No language-model integration, steering
changes, robot deployment or physical movement was introduced by this UI edit.
The old app was closed normally (Stop) and the revised companion reopened.

### 2026-09-10: E -> C straight-junction exit handoff

The user reported stopping at the E -> C curve during a live-chat route. The saved controller status recorded `Outgoing lane was not reacquired; position must be reset`, route A -> E -> C at index 1, active straight crossing, and zero requested/executed wheels. This was a junction reacquisition timeout, not the ordinary curve follower stopping. The user had subsequently pressed Stop, so the top-level Stop reason did not explain the original fault.

Replaying a stationary image captured during diagnosis reproduced rejection by the five fixed row samples: three ordered pairs were visible but none at the prescribed near rows. Denser sampling found 15 ordered row pairs reaching 70% of the analysed region, with small lateral/heading errors. The image is current stationary evidence, not a complete recording of the failed movement.

For app-controlled straight exits only, a fallback now samples additional rows and requires at least six pairs across a 54% depth span, real support in the lower third, plausible ordering/continuity, and the existing alignment/stability thresholds. Distant fragments, misalignment and unmarked images still cannot complete a crossing. After successful straight reacquisition, app routes release junction steering to ordinary lane following immediately; a separate straight-only settling phase no longer owns an ensuing road curve. Finite diagnostic routines retain their previous settling behaviour.

The continuous launcher, colour thresholds, ordinary curve steering, sharp-corner profiles, red-stop detection, seven-second chat pause support and 60-second red-line wait are unchanged. The crossing deadline and independent stopping safeguards remain active.

Validation: 180 focused offline tests passed (including seven new exit/handoff regressions and the camera/client/UI checks); project source/link/launcher checks passed. The first deployment exposed a read-only camera-preview compatibility issue; the scoped detector gate now handles that preview object and its tests pass. Captured-image replay completed the handoff exactly once to E -> C with normal following active. A local backup and diagnostic image/status/logs are outside the repository. No physical validation run and no Git operation occurred during this fix; road performance remains for the user's next app-started run.

### 2026-09-10: rightward departure after E; preserve visual steering

The user reported slow/hesitant approach, rightward departure after entering the outgoing lane, and leaving the road before reaching the curve. They then picked up/repositioned duck2; subsequent stationary geometry is excluded as evidence of its driving trajectory. Saved status again records a straight-crossing reacquisition fault at route index 1, rather than a completed handoff to road following. Sparse wheel logs show mostly near-equal requested power while the lane estimate was absent. There are no synchronized images of this failed run, so they do not establish the full cause of its physical drift.

Code review found that the previous extra-row acceptance fallback also replaced the geometry used for visual steering. That unintended coupling is removed: extra-row geometry is now separate acceptance evidence, and the previous five-row visual steering calculation remains intact. An app-controlled straight exit can also complete once the normal road follower's center estimate is acceptable in a stable, ordered near-field corridor, with acceptable heading and bounded lateral geometry. It need not first force the different junction-fit target. Left/right maneuvers and ordinary curve tuning are unchanged. No new stop condition was added, and existing stopping checks/deadlines were not changed.

Verification: 169 focused offline tests passed, including the exit handoff, differing targets, misalignment rejection, camera preview, live pause/turn delivery and existing junction safety checks. Replay of all 87 available images from the accepted straight-crossing recording `20260909T172430Z-junction-straight-restored-baseline-b7dbc3de` produced exactly the same lane errors, steering geometry and visual wheel requests as the source before the E -> C handoff changes. This is an offline regression result, not a physical revalidation. Compact junction-state/geometry diagnostics are logged every five processed crossing frames for the next user-started run. No robot movement or Git operations were performed for this diagnosis.

### 2026-09-10: straight instruction persisted into the E -> C curve

The latest user observation was successful pause/red-line departure followed by weak left curving, crossing the white border, and stopping. The recorded crossing trace confirms that ordinary road following never took ownership: route index stayed 1 with active turn straight until the reacquisition deadline. The straight-fit controller requested rightward corrections as the outgoing road bent left. A stable curved corridor exceeded the straight-heading acceptance bound, which is unsuitable for identifying a road that already curves.

For app-controlled straight crossings, stable ordered outgoing borders at multiple image heights now latch normal road tracking before final near-field route confirmation. The unchanged ordinary detector/reference and wheel mixer then own steering; losing a near yellow dash does not resume the straight crossing profile. The lane need not be straight or perfectly centred first. Near-field paired support remains necessary before the route index advances once to E -> C, clears active turn, and clears the crossing deadline. A new route or junction resets the latch. Initial unmarked crossing, left/right junction profiles, ordinary curve gains, chat pause duration and 60-second direction wait are unchanged. No additional stopping condition was introduced.

Validation: 173 focused offline tests passed. Replay of the latest recorded junction geometry/status sequence reproduced the old stuck-straight timeout and, with the revision, entered road_tracking 3.70 s into the recorded search sequence and following at route index 2 at 6.23 s. These are replayed state transitions, not predicted physical timings or proof of road containment. Tests cover early visual steering on a curve, later near-field confirmation despite curved geometry, partial dashes, false corridors, scope exclusions, route completion, live pause/turn delivery and existing stopping checks. Backup, original status/logs and replay results remain outside the repository. No physical run or Git operations occurred during implementation.


## 2026-09-10 — Fresh live-chat junction scenario (prepared, not physically run)

The prior run failed outgoing-lane acceptance at E: sampled traces still said
`searching`, with two distant boundary pairs and no near support; the nine-second
search timer expired. The user also observed hesitation after pausing. Recorded
requests included 0.03/0.175 followed by approximately 0.09/0.09. Those requests
show steering and low baseline power but do not establish the physical cause of
the hesitation. No new wheel-power or steering calibration was inferred.

The new scenario opens the selected A → E → B → C route with an empty message
box. The user starts on A → E after its curve, sends `stop for 7s`, replaces the
remaining turn queue with `go straight at the next junction`, follows E → C,
waits at C for up to 60 seconds, then sends `left`. It follows C → B and ends
stopped at B's red line. Final-stop policy is stored on the robot, rather than
requiring a finishing Stop message from the laptop. The existing junction-chat
launcher now uses `--junction-chat-scenario`; the old flag remains an alias.

Straight exits retain densely sampled, actually observed corridor evidence
through yellow dash gaps without relabeling mid-depth evidence as near-field
alignment. An ordered corridor across multiple depths, reaching at least row
0.54 of the analysis region and stable for 0.3 seconds, commits road steering
and route progress together. The E search timer is cleared at that transition.
No extra straight-alignment phase remains attached to the subsequent left
curve. Distant fragments, reversed boundaries, narrow patches and flickering
candidates still fail acceptance; unknown exits retain the crossing deadlines.

New session initialization clears prior pause, geometry-continuity, encoder-
balancing, crossing and reacquisition state. Each app Start uses a new run ID.
`stop for 7s` now accepts the same seven-second request as `stop for 7 seconds`.
No chatbot message or route Start is sent by opening the scenario launcher.

Verification completed:

- 161 focused native tests passed with socket creation blocked, covering the
  exact scenario through final B stop, existing navigation/safety regressions,
  pause/Stop priority, delayed directions, queue transactions and UI behavior.
- The subsequently added native UI scenario check passed with the other five
  UI checks: blank text, no automatic Start, initial route, final directed red
  line and distinct new session IDs. Total distinct focused checks: 162.
- Real HTTP/ROS verification passed in a disposable Docker container with
  `--network none`: all three speed profiles; immediate zero; `stop for 7s`
  measured 7.080 seconds until resumed wheel feedback; next-red completion;
  configured C → B final stop; and rejection of a late resume.
- Project validation passed for 75 Python sources, documentation links and
  Windows launcher targets.
- AST comparison against the pre-change local backup confirmed unchanged
  wheel mixing, acceleration ramp, approach steering, left-junction visual
  control, junction profiles, planned turn commands and sharp-corner control.
  The continuous launcher also matched the backup byte-for-byte.

The verified sources were deployed to the existing temporary driving container.
Read-only verification confirmed matching source hashes, `awaiting_route`,
manual stop, fresh camera/status, one wheel publisher and fresh executed-zero
feedback. No physical run or Git operation occurred. The previous app window
closed normally. Automatic approval review blocked the combined app-opening
command, so the updated launcher must be opened manually. A newly supervised
physical run is still required to establish road containment and curve behavior.


## 2026-09-10 — Stronger ordinary left-road response

The user reported improvement on E → C: the robot began following the left
curve and remained within the borders, but needed a stronger turn. The latest
saved log already reached approximately 0.03/0.15, then relaxed toward equal
wheel commands as the centroid error diminished. This observation does not
establish a new physical curvature calibration.

Added opt-in `road_left_response_scale`, default 1.0, set to 1.30 only in the
continuous launcher. It smoothly increases negative-error lane-following
feedback between the existing 0.05 deadband and 0.09 full-gain threshold. It
requires both boundaries and ordinary following with no active junction,
red approach or sharp-corner maneuver. Errors within the deadband, right
corrections, single-boundary fallback and all junction profiles are unchanged.
The existing 0.03 active-wheel floor, 0.20 wheel cap, 0.11 steering cap,
acceleration limiting and colour thresholds remain unchanged. The multiplier
is controller response, not a measured percentage increase in physical turning.

At a synthetic steady lane error of -0.08, the old wheel request was about
0.0425/0.1375 and the revised request is about 0.030/0.1514, reaching the
previously successful left-curve command range sooner. Straightening remains
camera-controlled; there is no fixed turn latch or fixed-duration left curve.

48 focused offline checks passed with networking blocked, including scope,
wheel caps, lane-loss zero, straight handoff, complete chat scenario and
steering-transition regressions. Project validation passed for 76 Python
sources, documentation links and Windows launcher targets. The pre-change
node and launcher are saved outside Git in `before-adjustment.zip` in this
run's diagnostics folder. No physical test or Git operation occurred.


## 2026-09-10 — Undo broad left response; stronger turning only on a detected curve

The user reported that the preceding 1.30 left-response change oscillated on
ordinary road and crossed both boundaries. Treat that attempt as failed. The
entire `road_left_response_scale` change was removed by restoring the node and
continuous launcher from the pre-adjustment snapshot before this replacement.
The earlier section describes a superseded implementation, not the current one.

The new opt-in `road_left_curve_boost` is enabled only in continuous app mode.
It requires matched, correctly ordered yellow/white observations across shared
near/far image depths and leftward curvature in both borders. A displaced or
slanted straight is insufficient. The fitted bows are image-space evidence,
not calibrated turning radii or distances. Confirmation lasts 0.3 seconds.
When currently confirmed, the target is left 0.03 / right 0.20, reached through
the existing steering/output slew limits. The boost clears on loss of curve
evidence, a needed rightward correction, stale camera data, missing borders,
red approach, pause/Stop, or junction/sharp-corner control. Normal road gains,
trim, colour thresholds, junction maneuvers and chat scenario remain unchanged.
The old experimental road lookahead remains disabled.

Focused offline checks cover curved images, dashed stripes, offset/yawed
straights, right curves, missing/partial markings, curve-to-straight reset,
requested wheel values, camera/Stop/pause gates, callback evidence transfer,
the E outgoing-lane handoff, fresh chat scenario and steering transitions.
Saved real images were replayed outside the repository: the accepted left-curve
sequence confirms the curve before straightening; the accepted straight-crossing
sequence produces no curve candidates. Replay is not a physical validation of
the stronger pair on this route. The user must evaluate containment on the next
app-started run. No physical run or Git operation is part of this change.

Verification result: 52 focused checks passed with robot networking blocked.
Project checks passed for 76 Python sources and documentation/launcher links.
The 88-frame accepted curve replay classified frames 0-26 as curve candidates
and released the boost classification on the exit; the 87-frame accepted
straight-crossing replay had zero candidates. Deployed source hash matched the
local file; the controller reported awaiting_route, manual stop, a fresh camera,
one wheel publisher and fresh executed zero. The app was refreshed without
starting a route or sending any chat message. Physical validation is pending.


## 2026-09-10 — Camera viewer connection refresh

After the curve-only deployment, Windows SSH and command HTTP were reachable,
but the read-only camera gateway raised AttributeError because LanePreview
does not initialize the new controller-only road_left_curve_boost option.
The shared detector now defaults an absent option to false for preview use.
This one-line compatibility fix does not change the configured steering.
29 existing camera/detector and curve regressions passed offline. The temporary
app service is refreshed while stopped; no route or chat command is sent.


## 2026-09-10 — Straight-crossing distant exit visibility

User observation: after the E red stop, duck2 drifted right and left the road.
The saved run 774b0e2c-8dbe-40e8-9b48-54ac191da7cc stayed in authorized straight
searching, never accepted the outgoing lane, and stopped on its reacquisition
deadline. Recorded crossing commands were approximately 0.15/0.15 with small
encoder corrections. The curve-only boost was not eligible during this phase.
The user moved the robot after stopping, so subsequent images do not document
its failed endpoint or prove its physical heading during that run.

After the user positioned it at E for stationary inspection, the camera showed
outgoing yellow/white boundaries around image rows 147-174. The regular lane
ROI begins at row 240 in this 480-row image. It therefore could not use the
visible distant corridor to correct the crossing. This is a demonstrated
perception blind spot; it does not prove every cause of physical drift.

Added distant-corridor aiming only for authorized app straight crossings, after
the entry interval and while reacquiring. It requires ordered boundaries,
multiple shared rows, widening toward the camera, coherent line fits and 0.2 s
of fresh observations. A small deadband suppresses dash jitter; steering is
limited to +/-0.025 around the existing crossing speed. It aims toward the
optical centre without applying ordinary-road trim. Existing usable lower-ROI
lane steering takes priority. Distant evidence alone cannot advance route
progress or bypass the existing near-lane handoff, deadline or stop gates.
No ordinary-road/curve tuning, left/right intersection profile, red threshold,
wheel cap, timing, or chatbot command behavior changed.

The stationary image produced ten matched forward row pairs with median centre
321.125 in a 640-pixel image, requesting equal wheels after the deadband.
79 offline checks passed with networking blocked, covering forward detection,
steering direction/caps, missing/reversed/transverse markings, freshness,
near-lane takeover, Stop output, camera preview, curve scope, straight handoff,
chat scenario and steering transitions. Project validation passed for 77 Python
sources and documentation/launcher links. Source backup and camera evidence
remain outside Git. No physical run or Git operation occurred. Road containment
with the correction remains unverified until a user-started app run.


## 2026-09-10 — Camera freshness stop during aligned straight crossing

User observation: straight crossing stayed aligned but stopped before entering
the outgoing lane. The app screenshot and robot fault agree: Camera lost during
junction; position must be reset. Run 7391915f-65a6-469f-8135-b01c54c7e2d3 remained
in straight reacquisition; the stop was not another red-line stop. Red detection
bounds and thresholds were therefore left unchanged.

Both image processes were using the four-worker OpenCV default; the controller
and preview each consumed roughly one CPU core in the sampled process report.
A stationary benchmark on duck2, with ROS transport mocked and sockets blocked,
measured full perception at median/95th-percentile/maximum 89.76/129.55/159.31 ms
with four workers, versus 83.90/101.07/130.51 ms with one. These short samples
support reducing competing processing, but do not prove the cause of every
possible future camera gap.

Both application image entry points now select one OpenCV worker. The read-only
viewer renders at most 10 fps, skipping surplus callbacks before decoding;
skipped frames never renew captured/received timestamps. Controller camera
processing remains unrestricted by that preview cap. Camera freshness stays
0.5 seconds and real gaps still fault and stop. Persistent timeout diagnostics
record frame age, receive age and in-flight/previous processing duration so a
future gap can be distinguished from a rendering delay. Steering, crossing,
red-stop logic and maneuver timing match the pre-change source.

39 focused offline checks passed for preview throttling, image parity, true
camera loss, timeout diagnostics, forward-exit scope, left-curve scope, straight
handoff and the live-chat scenario. Project validation passed for 77 Python
sources and documentation/launcher links. The source backup and benchmark data
are outside the repository. No physical run or Git operation occurred.

Stopped live verification after deployment: both source hashes matched; both
image processes reported one OpenCV worker and preview reported 10 fps. Normal,
mask and overlay returned fresh frames. Across 30 stationary status samples,
maximum controller age was 0.305 s, viewer age 0.115 s and processing duration
0.154 s, with no recorded timeout. This is stationary timing evidence, not a
completed physical crossing. The app was reopened and left awaiting Start.


## 2026-09-10 — Latest app attempt interrupted by depleted battery

Status: INCOMPLETE / NOT VALIDATED — do not mark successful.

The user reported that the latest attempt seemed good, but the battery ran out.
This is a provisional visual observation only. Route completion, outgoing-lane
entry and normal commanded stopping are not established for this attempt.
Retain the current scenario and camera-performance settings for a later retry;
no steering, detection, timing or app behavior changes were requested here.
Further testing is paused pending the user's next task. No robot access, physical
test or Git operation was performed while saving this record.


## Documentation-refresh verification — 2026-09-10

- The 77-source syntax, relative documentation-link and Windows launcher-target
  audit passed.
- 63 focused native tests passed across `test_live_chat`, `test_live_navigation`
  and `test_route_planner`; no hardware or live ROS master was used.
- The report compiled to four pages. Every rendered page was visually reviewed;
  its PDF text and LaTeX source passed the requested topic-exclusion check.
- Runtime Python, controller/ROS launchers, configuration, dependencies, tests
  and licence were byte-compared against the recovery snapshot and unchanged.
- Three obsolete shortcuts were the only removed files. The latest physical
  attempt is still incomplete; no physical test, robot change or Git operation
  was performed during this documentation refresh.


## 2026-09-10 — Live-chat curve yellow-gap audit

Reproduced loss of confirmed curve authority during a short missing-yellow gap,
and a V=170 to V=150 white-reference switch that changed lane-error sign. Added
a maximum 0.8-second continuation requiring recently confirmed complete curve
evidence and a matching, still-curved white trace. It cannot recognize a new
turn or renew itself using white alone. Straight/junction settings and red
detection are unchanged. Full-boundary saved-frame outputs are identical.

516 native tests passed, three skipped; 137 focused tests and actual HTTP/ROS
chat checks passed in an isolated Noetic container. The seven-second synthetic
pause measured 7.081 seconds. The earlier battery-interrupted run remains
incomplete; no physical run is claimed. See [lane review](LIVE_CHAT_LANE_REVIEW.md)
for reproduction, safety coverage, replay and deferred physical checks.


### Stopped deployment verification

The robot was reachable after reboot. The current source was staged with the
standard preparation tool; its SHA-256 matched the local node. Exclusive
project ownership, manual stop, awaiting_route and executed zero were verified.
Across 18 later stopped-state samples, maximum controller frame age was 0.256 s;
nine normal/mask/overlay samples decoded with advancing timestamps and maximum
preview age 0.131 s. No new timeout occurred in that window. An earlier brief
0.515-second timeout remained in the diagnostic history; camera reliability
through a moving route is not established by these samples.

The same junction-chat scenario was reopened. No route, Continue, Start or
chat message was sent. Physical validation still requires the user's placement
and observation. The installed ROS runtime and calibration were preserved.

## 2026-09-10 — yellow-gap and outgoing-lane follow-up

User observation: rightward drift soon after entering the outgoing lane, then
robot repositioned. Not accepted as a successful run. The retained log remained
in straight-junction reacquisition until manual Stop; its final current image
is not failure evidence after repositioning.

Fixed a reproduced mismatch between the mixed-depth lane centroid and confirmed
left-curve geometry. Kept the established 0.03/0.20 curve command and 0.8-second
maximum confirmed yellow gap, with near-corridor displacement protection.
Separately corrected fresh-camera confirmation resets and far-end fragment
rejection in distant straight-exit aiming. Steering caps, red detection, fresh
camera limits, Stop and route-completion requirements remain unchanged.

Validation: 523 native tests passed, three platform skips; 144 isolated Noetic
focused tests plus real synthetic HTTP/ROS chat/pause/red-stop checks passed.
The seven-second synthetic pause measured 7.093 s. Saved accepted curve/straight
frame replay retained all prior wheel requests. Recorded-coordinate challenges
verify the outgoing aiming correction, not physical lane containment.

See [the detailed lane review](LIVE_CHAT_LANE_REVIEW.md) for evidence and limits.
No physical movement or Git operation was performed. Track validation is pending.

## 2026-09-10 — additive right-white protection

Failed run: the user reports a rightward heading already at the red stop and
rightward departure, then repositioned duck2. The log records equal approach
commands when paired guidance was unusable, followed by near-equal crossing
commands without a usable outgoing yellow/white pair. Do not infer the old pose
from the current camera view.

Added an opt-in white-boundary guard to the existing companion controller's final
wheel request. It protects the latched approach and straight crossing/search
when a coherent near right stripe intrudes toward the forward path, even if
yellow is absent. Correction is bounded at 0.03, with existing wheel limits;
normal white at the side and an aligned paired corridor do not receive a trim.
Ordinary curves and deliberate left/right junction maneuvers are unchanged.

Validation: the full native run passed 543 cases with three platform skips
(540 passes); the final 19-case guard module also passed, including two added
paired-corridor cases. The isolated pinned Noetic run passed 163 focused cases
and real synthetic HTTP/ROS chat checks; seven-second pause measured 7.096 s.
Replaying 88 accepted curve frames and 87 straight frames as ordinary road
following retained all previous errors and wheel requests. These are software
checks, not evidence of successful physical correction.

See [the lane review](LIVE_CHAT_LANE_REVIEW.md) for the exact scope and limits.
No motion or Git operations were performed. A supervised app retry is pending.
