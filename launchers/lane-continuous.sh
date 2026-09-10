#!/bin/bash
set -eo pipefail
source /environment.sh
dt-launchfile-init

# Continuous right-lane route mode. Starts stopped and requires a live desktop
# heartbeat plus a confirmed route before motion. Obstacle passing is disabled.
pids=""
cleanup() {
    trap - INT TERM EXIT
    for pid in $pids; do kill -INT "$pid" 2>/dev/null || true; done
    wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

rosrun duckie_lane_follower lane_follower_node.py \
    _drive_enabled:=true _show_debug:=false _route_enabled:=true \
    _junctions_calibrated:=true _auto_continue:=true \
    _require_client_heartbeat:=true _obstacle_enabled:=false \
    _avoidance_enabled:=false _lost_speed:=0.0 \
    _base_speed:=0.09 _max_speed:=0.20 _max_steering:=0.11 \
    _min_active_wheel_speed:=0.03 _taper_inner_wheel_floor:=false \
    _k_p:=0.75 _near_center_k_p:=0.35 _full_gain_error:=0.09 \
    _lane_target_fraction:=0.441 _alpha:=0.20 _deadband:=0.05 \
    _steering_bias:=0.0075 _smooth_steering_deadband:=true \
    _temporal_lane_width_fallback:=true _temporal_lane_width_timeout:=0.30 \
    _temporal_yellow_only_timeout:=5.0 _boundary_risk_stop:=true \
    _white_boundary_risk_fraction:=0.43 \
    _yellow_lower:="[20,70,80]" _yellow_upper:="[35,255,255]" \
    _white_lower:="[0,0,150]" _white_upper:="[180,55,255]" \
    _road_white_reference_value:=170 \
    _road_heading_guard:=false _road_left_lookahead:=false _road_left_curve_boost:=true _junction_left_visual_latch:=true \
    _red_stop_trigger_bottom_fraction:=0.86 _stop_hold_seconds:=2.0 \
    _junction_entry_seconds:=0.5 _junction_straight_seconds:=1.0 \
    _junction_left_seconds:=1.6 _junction_right_seconds:=1.6 \
    _junction_reacquire_timeout:=9.0 _junction_reacquire_max_error:=0.10 \
    _junction_straight_speed:=0.15 \
    _junction_straight_approach_max_steering:=0.03 \
    _junction_straight_visual_approach:=true \
    _junction_white_boundary_guard:=true \
    _junction_straight_lane_target_fraction:=0.49 \
    _junction_straight_lateral_gain:=0.25 _junction_straight_heading_gain:=0.30 \
    _junction_straight_departure_max_heading:=0.12 \
    _junction_straight_reacquire_max_lateral:=0.10 \
    _junction_straight_reacquire_max_heading:=0.08 \
    _junction_straight_reacquire_seconds:=0.50 \
    _junction_straight_settle_max_lateral:=0.06 \
    _junction_straight_settle_max_heading:=0.05 \
    _junction_straight_settle_max_steering:=0.025 \
    _junction_straight_settle_seconds:=0.60 \
    _junction_straight_encoder_balance:=true \
    _junction_straight_encoder_balance_gain:=0.12 \
    _junction_straight_encoder_balance_max:=0.015 \
    _junction_straight_encoder_balance_min_ticks:=12.0 \
    _junction_left_speed:=0.115 _junction_left_bias:=0.085 \
    _junction_right_speed:=0.10 _junction_right_bias:=0.10 \
    _junction_right_tracking_trim:=0.0 _junction_right_encoder_assist:=true \
    _sharp_corner_enabled:=true _sharp_corner_confirm_seconds:=0.20 \
    _sharp_corner_recent_lane_seconds:=1.0 _sharp_corner_approach_seconds:=1.0 \
    _sharp_corner_turn_speed:=0.20 _sharp_corner_min_turn_seconds:=0.0 \
    _sharp_corner_white_confirm_seconds:=0.10 \
    _sharp_corner_pivot_seconds:=0.45 _sharp_corner_relief_seconds:=0.25 \
    _sharp_corner_relief_inner_speed:=0.0 \
    _sharp_corner_max_turn_seconds:=10.0 \
    _sharp_corner_reacquire_seconds:=0.30 \
    _sharp_corner_trigger_error:=0.10 _sharp_corner_exit_error:=0.13 &
pids="$pids $!"

rosrun duckie_lane_follower command_gateway.py _listen_host:=127.0.0.1 &
pids="$pids $!"
rosrun duckie_lane_follower camera_gateway.py _listen_host:=127.0.0.1 \
    _lane_target_fraction:=0.441 \
    _junction_straight_lane_target_fraction:=0.49 \
    _temporal_lane_width_fallback:=true _temporal_lane_width_timeout:=0.30 \
    _temporal_yellow_only_timeout:=5.0 _boundary_risk_stop:=true \
    _white_boundary_risk_fraction:=0.43 \
    _yellow_lower:="[20,70,80]" _yellow_upper:="[35,255,255]" \
    _white_lower:="[0,0,150]" _white_upper:="[180,55,255]" \
    _road_white_reference_value:=170 &
pids="$pids $!"
rosrun duckie_lane_follower continuous_safety_watchdog.py _vehicle:=${VEHICLE_NAME} &
pids="$pids $!"

wait -n $pids
