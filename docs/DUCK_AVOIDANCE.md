# Experimental duck detection and passing

This is our own implementation using the existing node and the user's duck
photo as a visual reference. No other team's code, vision logic, movement logic
or colour thresholds were consulted. The photo shows yellow toys with red beaks;
it is not a robot-camera calibration image.

## Current behaviour

The default launchers still stop and latch for obstacle candidates. Passing is
**disabled by default**. The offline chatbot recognizes obstacle reports and
passing requests, but does not enable the controller or send a new command.

The optional prototype follows this sequence:

1. Find a compact yellow candidate inside the right lane.
2. Require both outer white borders and the yellow divider in three image bands.
3. Steer toward the observed left-lane centre at a capped speed.
4. Follow the left lane while observing the duck move toward the lower right
   camera edge.
5. After that cue, require continuous absence and centred left-lane tracking for
   the configured clearance allowance, weighted by published wheel commands.
6. Steer back toward the observed right-lane centre. Require stable reacquisition
   before returning control to ordinary lane following.

Route, route index, pending turn and speed scale are retained throughout a
successful pass. Route navigation does not advance during the manoeuvre.
Stop, camera loss, invalid timestamps, missing/ambiguous boundaries, a red line,
an additional obstacle candidate, manual interruption, lost client heartbeat or
the manoeuvre timeout stop passing. Faults latch: ordinary Continue cannot
resume an interrupted pass. Stop and physically confirm/reposition the robot,
then restart the node and reconfigure its route as appropriate.

## Provisional settings

All are private ROS parameters of the existing lane-follower node.

| Parameter | Default | Meaning |
| --- | --- | --- |
| duck_lower | [15, 90, 70] | Generic OpenCV HSV lower bound |
| duck_upper | [40, 255, 255] | Generic OpenCV HSV upper bound |
| avoidance_enabled | false | Explicit opt-in to experimental passing |
| avoidance_calibrated | false | Operator declaration after physical checks; not automatic verification |
| avoidance_speed | 0.04 | Maximum normalized wheel command during passing, not m/s |
| avoidance_clear_seconds | 1.0 | Equivalent seconds at the passing speed cap after side-passage cue |
| avoidance_timeout | 12.0 | Maximum total manoeuvre time; exceeding it latches a stop |

Enabling requires both calibration and the existing obstacle detector. The
avoidance speed cannot exceed the configured base or maximum speed. Existing
acceleration limits and final wheel safety gates still apply.

Existing lane yellow/white and stop-line red defaults are unchanged. Duck
candidates are masked from yellow lane estimation so a large duck cannot directly
pull that estimate toward itself. Compactness and road placement help distinguish
ducks from thin dashed paint, but **do not establish semantic duck recognition**.
A broad painted dash can look like an object, and a duck touching a lane marking
can merge into an undetectable contour.

Status adds duck_candidates, road_geometry, avoidance_enabled, avoidance_state
and avoidance_reason. Camera debug shows candidate boxes, the three boundary
estimates when available, and the passing state/reason.

## Limits that need the actual bot

This prototype is restricted to straight, completely empty two-lane road sections.
It cannot verify that an adjacent lane is safe from oncoming traffic. The existing
bright/coloured-object check can miss dark objects. Passing stops when markings
are missing; dashed dividers may therefore prevent passing in many frames.
There is no inferred road boundary or timed blind steering fallback.

The three sampled stripe positions and pixel margins are image heuristics, not a
calibrated road projection or a vehicle-footprint model. They cannot guarantee
that the wheels stay on the road. The side-passage cue plus clearance interval
cannot prove rear-body clearance; disappearance alone is explicitly rejected.
The allowance slows with lower published speeds, but wheel commands are not motion measurements.
A stalled/slipping wheel, occlusion, shadows, duck orientation and camera pitch
can all invalidate these assumptions. No distance sensor, odometry or physical
collision-clearance estimate was added.

## First physical sequence

1. Keep passing disabled. Use the existing stationary camera-debug launcher on
   the actual track, with ducks in both lanes and directly beside dashed paint.
   Inspect candidate boxes and road geometry at different distances and lighting.
2. Confirm detection before contact and ordinary latched stopping. Record frames
   for false detections, merged paint/duck contours and missing markings.
3. With wheels lifted, check steering polarity, the speed cap, and Stop/Ctrl+C.
4. On a clear straight section, measure camera margins, actual robot width,
   minimum passing gap, and front/rear duck clearance. Use a spotter and very slow
   supervised trials to establish the clearance interval at the chosen speed.
5. Only after those checks, opt into the prototype for supervised passing trials.
   Test interruptions before attempting a complete pass.
6. Verify right-lane return and retention of the pending route instruction.

If reliable clearance and road-boundary estimates cannot be established, retain
the default stop-only behaviour. A real guarantee would need stronger calibrated
geometry and motion/clearance feedback.

## Offline checks

From the repository root in WSL:

~~~bash
docker run --rm --network none -v "$PWD:/project:ro" --entrypoint python3 altata-duck2:noetic-amd64 /project/tests/test_duck_avoidance.py
python3 tests/test_obstacle_language.py
~~~

Synthetic scenes exercise a completed pass, route retention, boundary loss,
dashed lines, other obstacle candidates, camera faults, speed caps and stops.
The isolated ROS suite also exercises JPEG camera messages, actual wheel-message
publication, debug mode and shutdown during passing. These tests establish
software behaviour on artificial inputs, not a collision-free physical path.
