#!/bin/bash
source /environment.sh
dt-launchfile-init
# Records camera/status only. Bind /data/captures to a laptop folder to keep output.
rosrun duckie_lane_follower camera_capture.py
dt-launchfile-join
