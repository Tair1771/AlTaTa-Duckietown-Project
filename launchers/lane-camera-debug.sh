#!/bin/bash

source /environment.sh

dt-launchfile-init

# Debug mode: show camera/mask windows, but do NOT drive the wheels.
# Use the pinned-image procedure in docs/ROBOT_SETUP.md (no .dtproject exists).
# For obstacle diagnostics use lane-obstacle-debug instead.
rosrun duckie_lane_follower lane_follower_node.py \
    _drive_enabled:=false \
    _show_debug:=true \
    _obstacle_enabled:=false \
    _avoidance_enabled:=false \
    _wheels_topic:=/${VEHICLE_NAME}/lane_follower/diagnostic_wheels_cmd

dt-launchfile-join
