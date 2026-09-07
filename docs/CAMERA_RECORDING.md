# Collecting real camera evidence

The lane-record launcher runs camera_capture.py. This node has subscribers only:
it cannot publish wheel commands. It records at most 300 frames at 5 Hz over
60 seconds by default and stops automatically. No-frame capture returns an error.

The default output root is /data/captures. Bind that container directory to a
persistent laptop folder before running; otherwise container removal loses output.
Each run creates a unique subdirectory with PNG frames and frames.jsonl.
The JSONL records camera timestamps, receipt time, latest lane-follower state,
and the age of that state. A recording does not itself prove physical provenance.

Run the project with lane-record as launcher, connected to duck2, and pass a
Docker bind mount for /data/captures using the installed dts docker_args option.
The installed dts help confirms forwarded Docker arguments are supported.
The complete real-bot invocation must be checked when robot connectivity is available.

Collect stationary camera examples before driving:
- straight right lane, both directions of curves, and dashed-yellow gaps;
- red approach line at several distances before crossing;
- duck/bright/colored obstacles at lane center and edges;
- empty road, shadows, reflections, dark obstacles and objects outside the lane;
- each junction approach and each intended outgoing lane.

The current obstacle heuristic searches for compact bright or colored regions in
the estimated lane corridor. It is not a trained object recognizer. Dark objects
may be missed, and glare or inaccurate lane boundaries can cause false stops.
It stops and requires the corridor to remain clear for .5 seconds plus explicit
Continue. An interruption inside a junction requires the route position to be reset.

For each scene, retain the original PNG, a human label (lane / stop-line /
obstacle / clear), and approximate physical placement or measured distance.
Use held-out frames to measure missed stops and false detections after tuning.
Do not adjust thresholds only to make a small set of synthetic examples pass.

The recorder was tested with five synthetic ROS JPEG frames. Its output PNGs
decoded successfully and its JSONL included timestamps and fresh node status.
No real camera recording has been captured in this task yet.
