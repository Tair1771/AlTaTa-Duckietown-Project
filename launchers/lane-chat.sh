#!/bin/bash
set -eo pipefail
source /environment.sh
dt-launchfile-init

# Route control starts stopped and requires the desktop heartbeat.
: "${DUCK2_CONTROL_TOKEN:?Set DUCK2_CONTROL_TOKEN for the desktop gateway}"
node_pid=""
gateway_pid=""
cleanup() {
    trap - INT TERM EXIT
    if [ -n "$node_pid" ]; then kill -INT "$node_pid" 2>/dev/null || true; fi
    if [ -n "$gateway_pid" ]; then kill -INT "$gateway_pid" 2>/dev/null || true; fi
    wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

rosrun duckie_lane_follower lane_follower_node.py \
    _drive_enabled:=true _show_debug:=false _route_enabled:=true \
    _require_client_heartbeat:=true _junctions_calibrated:=false \
    _base_speed:=0.08 _max_speed:=0.18 _lost_speed:=0.0 &
node_pid=$!
rosrun duckie_lane_follower command_gateway.py _listen_host:=0.0.0.0 &
gateway_pid=$!
wait -n "$node_pid" "$gateway_pid"
