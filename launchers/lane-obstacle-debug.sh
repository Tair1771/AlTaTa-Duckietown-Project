#!/bin/bash
set -eo pipefail
source /environment.sh
dt-launchfile-init
# No movement. Detect lanes AND obstacles on current camera frames for calibration.
rosrun duckie_lane_follower lane_follower_node.py \
    _drive_enabled:=false _show_debug:=false \
    _route_enabled:=false _obstacle_enabled:=true _avoidance_enabled:=false \
    _wheels_topic:=/${VEHICLE_NAME}/lane_follower/diagnostic_wheels_cmd
dt-launchfile-join
