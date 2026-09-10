# Live-chat lane review — 2026-09-10

Scope: the current A → E pause/straight-override/E → C curve/C left-instruction
scenario. No Git operation or physical Start is part of this review. The prior
battery-interrupted run remains incomplete; no full-route success is claimed.

## Reproduced problems and focused correction

1. **A short missing-yellow interval cancelled the confirmed curve command.**
   `road_left_curve_boost` previously required both boundaries on every frame.
   Once yellow disappeared, the 0.03/0.20 curve request fell back to ordinary
   steering. White-only tracking expired after 0.3 seconds. The separate five-
   second *yellow-only* setting handles missing white; it does not help missing
   yellow. The older lookahead continuation is disabled in the current launcher.
2. **The white reference changed when yellow disappeared.** With both borders,
   the road controller prefers the bright V=170 reference. White-only tracking
   instead used the broader V=150 mask. In the accepted curve recording with
   yellow artificially removed for two frames, the lane error changed from
   approximately -0.050 to +0.040 without a corresponding border movement.
   This could remove curve authority and trigger a stop/restart sequence.

The correction keeps the bright reference and existing 0.03/0.20 command through
a short yellow gap only after the same curve was confirmed with both borders.
The remaining white trace must match the previous shared image depths, still
curve left, retain sufficient row support and avoid large jumps. White-only
frames cannot establish a new turn or renew the confirmation deadline.

The allowance ends at **0.8 seconds from the last confirmed complete curve**.
Straight white, changed edges, red approach, junction ownership, Stop/pause,
stale images and expired evidence remove it. Existing white-boundary risk,
camera, watchdog, heartbeat and wheel-ownership gates remain. Ordinary one-
boundary settings, colour values, full-boundary curve commands, intersection
profiles and chat grammar are unchanged. The read-only preview is kept separate
from this controller-only state.

Controller status now exposes `road_left_curve_yellow_gap_active` and
`road_left_curve_confirmation_age`. A continuation reports
`White-only confirmed curve: bounded yellow gap`. This is estimated continuation,
not a claim that yellow is visible. Preview diagnostics describe a separate
read-only detector and do not own motion or the controller's turn memory.

## Failure paths reviewed

| Risk | Finding / verification |
| --- | --- |
| Yellow dash gap during a confirmed curve | Reproduced with real OpenCV callbacks; corrected and bounded |
| White threshold changes as yellow disappears | Reproduced on a saved frame; reference now consistent for the confirmed gap |
| Straight instruction stays active on E → C | Existing full-scenario and handoff tests clear the active crossing/timer and return road ownership |
| Old left/right queue survives the straight override | Queue replacement and run/index acknowledgments tested |
| Pause delays or restarts an old turn | Immediate zero, seven-second pause, timer accounting and state reset tested |
| Pause changes straight speed profile | Profile remains normal; dedicated tests cover isolation from turning |
| Yellow remains absent, white becomes straight or jumps | New tests reject continued boost and stop when ordinary evidence expires |
| Blind turning on an unconfirmed road | New tests reject white-only initial recognition and disabled-option use |
| Camera preview and controller processing contend | Existing one-worker/10-fps-preview change retained; freshness not relaxed |
| Lost camera, manual Stop, competing publisher or stalled encoder | Existing native safety regressions retained and passed |
| Lost/duplicate command acknowledgment | Existing session reconciliation and deduplication tests passed |
| Old app/controller survives a source edit | Reprepare from the current source while stopped; verify hash and startup state |
| Battery depletion | Last physical attempt remains incomplete; sufficient charge must be established before retry |
| Red tape outside the course | Existing detector can still see transverse red inside its image ROI; cover unrelated tape as agreed. No red filtering change |
| Lighting, physical placement, cable resistance, long loss of both borders | Not resolved by a code-only proof; inspect the stationary scene and observe the next run |

No finite review establishes that every physical failure is impossible. These
checks distinguish reproduced software defects from conditions still requiring
the actual track, battery and camera scene.

## Offline evidence

- Baseline: 509 native tests, 506 passed and three skipped.
- Final native suite: 519 tests, 516 passed and three skipped.
- Isolated pinned Noetic container: 137 focused tests passed, followed by actual
  HTTP/ROS profile, immediate pause, red-stop and final-directed-line checks.
  The requested seven-second pause measured 7.081 seconds between observed zero
  and resumed output in this synthetic run. No hardware access or ports were
  provided to the container; it was removed afterward.
- Original accepted recordings: all 88 curve frames and 87 straight frames
  produced unchanged errors and requested wheels before/after the patch.
- Controlled missing-yellow challenge on two recorded curve frames (~0.53 s):
  old code dropped power, stopped, then ramped up from zero. New code retained
  0.03/0.20 and resumed complete-boundary tracking when yellow returned. The
  altered frames are a synthetic challenge, not a recording of physical success.
- New tests cover expiry, full lane loss, straight/jumping white, disabled option,
  pause/Stop, stale camera, junction exclusion, reference consistency and recovery.
- Python source, documentation links and Windows launcher targets passed checks.

The previous source and replay details are saved outside the repository in a
dated `live-chat-lane-audit` diagnostics directory. The report's previously
accepted component evidence is unchanged. This patch still needs a supervised
physical validation; it has not been marked successful on the track.

## Next user-run scenario

Use [LIVE_CHAT](LIVE_CHAT.md) and the same junction-chat scenario launcher.
Start after the A → E curve on the straight, centred and facing along the lane,
with camera uncovered and cable clear. Cover the unrelated red tape and ensure
adequate charge. A fresh Go is required for assistant-executed movement; app
Start and every chat message remain the user's actions.

Type `stop for 7s`, then `go straight at the next junction` before E. At C, type
`go left` during the 60-second wait. The scenario finishes at the C → B red line.
After any failed run, Stop and report what happened before another change.


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

## Follow-up: steering toward white after losing yellow

The user reports rightward drift soon after entering the outgoing lane and
confirms moving duck2 afterward. Its current camera view therefore cannot
reconstruct the failure. The saved controller log shows the last crossing
remained in `reacquiring/searching`, followed by a manual stop inside the
junction. It never confirmed the outgoing lane. The stopped state and executed
zero were checked before this review; no physical retry was started.

A separate callback reproduction exposed a remaining road-curve defect. Moving
the same curved white edge 20 pixels in a 640-pixel image changed the mixed-depth
centroid error from -0.019 to +0.044. The old +0.03 gate discarded the confirmed
curve even with coherent white evidence. Intermittent yellow detections could
then keep ordinary tracking alive while requesting a right turn.

Curve authority now uses the near corridor at matched image rows. A confirmed
curve can retain the existing 0.03/0.20 command through its bounded yellow gap
without that centroid cancellation. A substantial displacement left of the
near corridor still releases left-turn authority. The comparison uses image
position relative to observed lane width, not a calibrated angle or distance;
the global lane target is unchanged. Missing yellow alone cannot establish a
left turn, and the absolute 0.8-second confirmation limit is unchanged.

This callback defect is reproduced software evidence, not proof that it alone
caused the reported physical drift: the latest log was still in junction
reacquisition, where the road-curve boost is deliberately inactive. Outgoing
lane guidance had two additional reproduced failures:

- Confirmation reset after a 0.2-second interval even when camera frames were
  fresh under the existing 0.5-second limit. It now tolerates that valid cadence;
  missing geometry or stale images still clears confirmation.
- One or two far-end fragments invalidated an otherwise coherent corridor. The
  fit may now discard at most two far-prefix pairs, never more than 20 percent,
  and only when those points disagree with the retained fit. The retained rows
  must satisfy the original ordering, opening, spread and residual limits.
  Interior corruption, larger prefixes and incomplete boundaries remain invalid.

Recorded row-pair challenges now preserve the small leftward aiming correction
instead of falling back to nearly equal wheels. This is a replay of detected
points, not a reconstruction of the missing camera images. The 0.025 aiming cap,
crossing power, near-lane takeover and stricter route-progress criteria are
unchanged. No global left bias, red-region change or longer blind turn was added.

Source backups, logs and replay results are retained outside the repository in the dated
`yellow-gap-followup` diagnostics directory.

Follow-up validation: 526 native tests (523 passed, three platform skips), plus
144 focused tests in an isolated pinned Noetic container and HTTP/ROS chat
checks. The synthetic seven-second pause measured 7.093 seconds. All 88 frames
of the accepted curve and 87 of the accepted straight retained their previous
errors and wheel requests. Fourteen logged distant-corridor coordinate sets
were replayed; three rejected fits became usable after trimming one, one and
two far points respectively. No physical success is inferred from those checks.

The verified source was then loaded into the temporary driving container and
the junction-chat companion reopened. Source hashes matched; 18 stopped-state
samples retained manual stop, awaiting_route, zero requests and exclusive
publisher ownership. Nine camera previews decoded with advancing timestamps.
Maximum controller age was 0.265 s and preview age 0.123 s during that window.
A 0.517-second startup timeout remained recorded before the window; no new
timeout occurred during it. Startup freshness history was not erased or relaxed.
No route, Start, Continue or chat message was sent. The app is prepared for a
user-started retry; physical lane containment remains unverified for this patch.

## Follow-up: protect the right white border during straight crossing

The next attempt failed containment. The user observed a rightward heading
already at the red stop, followed by continuing rightward travel after departure,
and confirms repositioning afterward. The log shows equal 0.09 approach requests
after paired guidance became unusable, then approximately equal 0.15 crossing
requests with no usable distant yellow/white corridor. Encoders alone cannot
identify an incorrect starting heading. Current camera images cannot reconstruct
the failure after repositioning.

Added `junction_white_boundary_guard` to the companion's continuous launcher.
It wraps the existing final wheel request without rewriting crossing phases:

- Active only during the latched red-line approach and explicitly straight
  crossing/reacquisition, with valid fresh camera input and permitted motion.
- Independently trace a narrow longitudinal white stripe over at least five
  separated ROI depths, including the near field. Reject broad horizontal tape,
  distant-only fragments, large jumps and inconsistent fits.
- Request a small left correction when the near right edge intrudes toward the
  forward corridor. A recent, well-supported paired lane width can detect an
  inward displacement earlier at the same image rows. That reference expires
  after 0.8 seconds, resets on stop/pause and is never extrapolated into unseen
  near rows. White-line slope alone is not treated as robot heading.
- The correction is at most 0.03 in wheel space, uses the existing approach gain,
  retains the mean requested speed and respects wheel limits. A stronger existing
  left correction remains unchanged. A complete aligned lane takes precedence.
- Release when the white edge is back at the side, absent or unusable. Missing
  white does not generate a turn; white alone cannot complete a junction.

Ordinary roads/curves and intentional left/right crossings retain their previous
controllers. Stop/pause, red stopping, camera freshness, crossing deadlines and
watchdog behavior remain. No permanent trim, red ROI change or turning-speed
increase was added. New status and junction-trace fields record the observed
white geometry, whether protection intervened and why, including during approach.

Source backups and the failed-run log are retained outside Git in a dated
`right-white-guard` diagnostics folder. Physical validation of this addition
remains pending.

Verification: the native regression run completed 543 cases (540 passed, three
platform skips); all 19 final guard-specific cases passed as well, including
two paired-corridor cases added afterward. Pinned isolated Noetic checks passed
163 focused cases plus synthetic HTTP/ROS chat/pause/red-stop checks. The
seven-second pause measured 7.096 s. Ordinary-road replay retained previous
outputs for all 88 saved curve and 87 saved straight frames.

The current source hash and enabled guard parameter were verified on duck2.
The refreshed companion remained awaiting_route with manual stop, zero requests
and zero executed feedback. Eighteen stopped samples and nine camera views
passed, with maximum controller age 0.315 s and preview age 0.130 s. A prior
0.519-second startup timeout stayed in the diagnostic record; no new timeout
occurred within the verification window. No Start or chat command was sent.
