#!/bin/bash

source /environment.sh

dt-launchfile-init

# Stand test: wheels are enabled at low speed, no debug windows.
# Put the Duckiebot on a stand or hold the wheels in the air before running.
rosrun duckie_lane_follower lane_follower_node.py \
    _drive_enabled:=true \
    _show_debug:=false \
    _base_speed:=0.05 \
    _max_speed:=0.07 \
    _max_steering:=0.02 \
    _lost_speed:=0.0 \
    _route_enabled:=false \
    _obstacle_enabled:=false \
    _avoidance_enabled:=false

dt-launchfile-join
