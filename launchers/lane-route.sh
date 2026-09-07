#!/bin/bash
source /environment.sh
dt-launchfile-init
# Set route/position through the command interface before continuing.
# Junction calibration must be supplied after lifted-wheel/track measurement.
rosrun duckie_lane_follower lane_follower_node.py _drive_enabled:=true _show_debug:=false _route_enabled:=true _require_client_heartbeat:=true _junctions_calibrated:=false _base_speed:=0.08 _max_speed:=0.18 _lost_speed:=0.0
dt-launchfile-join
