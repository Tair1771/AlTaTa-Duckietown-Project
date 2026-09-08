#!/bin/bash

source /environment.sh

dt-launchfile-init

# Real lane-following mode. Start with conservative speed values.
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
