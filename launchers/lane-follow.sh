#!/bin/bash

source /environment.sh

dt-launchfile-init

# Legacy lane-only CALIBRATION mode: obstacle handling is deliberately off.
# Not the final autonomous demo. Prefer the bounded supervisor for ground tests.
rosrun duckie_lane_follower lane_follower_node.py \
    _drive_enabled:=true \
    _show_debug:=false \
    _base_speed:=0.08 \
    _max_speed:=0.18 \
    _lost_speed:=0.0 \
    _route_enabled:=false \
    _obstacle_enabled:=false \
    _avoidance_enabled:=false

dt-launchfile-join
