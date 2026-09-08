#!/bin/bash

source /environment.sh

dt-launchfile-init

# Debug mode: show camera/mask windows, but do NOT drive the wheels.
# Run this from the laptop with: dts devel run -R ROBOT_NAME -L lane-camera-debug -X
rosrun duckie_lane_follower lane_follower_node.py \
    _drive_enabled:=false \
    _show_debug:=true \
    _obstacle_enabled:=false \
    _avoidance_enabled:=false \
    _wheels_topic:=/${VEHICLE_NAME}/lane_follower/diagnostic_wheels_cmd

dt-launchfile-join
