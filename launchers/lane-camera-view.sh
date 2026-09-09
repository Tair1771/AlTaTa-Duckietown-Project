#!/bin/bash
set -eo pipefail
source /environment.sh
dt-launchfile-init

# Read-only service: subscribes to compressed images and owns no ROS publisher.
rosrun duckie_lane_follower camera_gateway.py _listen_host:=127.0.0.1
