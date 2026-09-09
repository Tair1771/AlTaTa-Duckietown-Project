#!/usr/bin/env python3
"""Robot-side supervisor for one deliberately authorized ground check.

This is a test harness, not an autonomous driving launcher.  It must run inside
duck2's ROS container after the normal kinematics publisher has been stopped.
The supervisor owns the emergency-stop timing and starts a separate watchdog
before releasing the stop.  A lost parent pipe makes the watchdog stop early.
"""

import argparse
import json
import math
import os
import queue
import select
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path


MAX_DURATION = 8.0
MAX_BASE_SPEED = 0.05
MAX_WHEEL_SPEED = 0.05
MAX_STEERING = 0.02
WATCHDOG_MARGIN = 0.25
FIXED_TURN_LEFT = 0.03
# The left value is already at the installed driver's minimum-PWM region.
# Increase only the right side for the next bounded curve diagnostic.
FIXED_TURN_RIGHT = 0.15
UPSIDEDOWN_LEFT_PIVOT_LEFT = 0.15
UPSIDEDOWN_LEFT_PIVOT_RIGHT = 0.0
UPSIDEDOWN_LOAD_PROFILE_LEFT = 0.20
UPSIDEDOWN_LOAD_PROFILE_RIGHT = 0.03
GROUND_ROLLING_RIGHT_LEFT = 0.15
GROUND_ROLLING_RIGHT_RIGHT = 0.03
GROUND_EQUAL_WHEELS_LEFT = 0.15
GROUND_EQUAL_WHEELS_RIGHT = 0.15
MAX_UPSIDEDOWN_LEFT_PIVOT_DURATION = 2.0
# This test path is only used after explicit live authorization.  It remains
# bounded independently from the general ground-test maximum.
MAX_FIXED_TURN_DURATION = 8.0
MAX_CAMERA_GUIDED_DURATION = 15.0
CAMERA_GUIDED_BASE_SPEED = 0.09
CAMERA_GUIDED_MAX_SPEED = 0.20
# Restore the steering authority that produced the accepted curve result.
# Safety now comes from bounded low-confidence fallback instead of weakening
# the requested turn until it can no longer hold the curve.
CAMERA_GUIDED_MAX_STEERING = 0.11
# Two loaded sharp-right attempts stalled while holding 0.15/0.00, while the
# same outside wheel ran continuously when lifted. Keep the inside wheel
# rolling through ordinary corrections. Confirmed corners have their own pair.
CAMERA_GUIDED_MIN_ACTIVE_WHEEL_SPEED = 0.03
# Ordinary lane tracking keeps the previously verified rolling floor.  The
# confirmed sharp-corner state owns the deliberate zero-inner-wheel pivot.
# Allowing the general controller to taper to zero can stall duck2 before the
# corner recognizer has had a chance to latch.
CAMERA_GUIDED_TAPER_INNER_WHEEL_FLOOR = False
CAMERA_GUIDED_K_P = 0.75
CAMERA_GUIDED_NEAR_CENTER_K_P = 0.35
CAMERA_GUIDED_FULL_GAIN_ERROR = 0.09
CAMERA_GUIDED_LANE_TARGET_FRACTION = 0.441
CAMERA_GUIDED_ALPHA = 0.20
CAMERA_GUIDED_DEADBAND = 0.05
# The smooth-mode straight test still finished slightly right of centre. This
# smaller provisional trim avoids building in a stronger right-turn request.
CAMERA_GUIDED_STEERING_BIAS = 0.0075
CAMERA_GUIDED_TEMPORAL_LANE_WIDTH_FALLBACK = True
CAMERA_GUIDED_TEMPORAL_LANE_WIDTH_TIMEOUT = 0.30
# This is restricted to yellow-only tracking after a measured two-boundary
# lane. White-only tracking keeps the shorter general timeout and boundary guard.
CAMERA_GUIDED_TEMPORAL_YELLOW_ONLY_TIMEOUT = 5.00
CAMERA_GUIDED_BOUNDARY_RISK_STOP = True
CAMERA_GUIDED_WHITE_BOUNDARY_RISK_FRACTION = 0.43
# Derived only from duck2's 2026-09-08 stationary curve recording. These are
# passed by this bounded test harness; normal launcher defaults stay unchanged.
CAMERA_GUIDED_YELLOW_LOWER = [20, 70, 80]
CAMERA_GUIDED_YELLOW_UPPER = [35, 255, 255]
# Stationary right-bend evidence from 2026-09-09: V=170 rejected all
# 44 frames; V=150 retained both boundaries without changing saturation.
CAMERA_GUIDED_WHITE_LOWER = [0, 0, 150]
CAMERA_GUIDED_WHITE_UPPER = [180, 55, 255]
# The corner state machine is confined to this bounded supervised profile.
# Restore the continuous 0.20/0.00 pair recorded in today's sustained right
# turn. This remains provisional: earlier attempts at this power also failed.
CAMERA_GUIDED_SHARP_CORNER_ENABLED = True
CAMERA_GUIDED_SHARP_CORNER_CONFIRM_SECONDS = 0.20
CAMERA_GUIDED_SHARP_CORNER_RECENT_LANE_SECONDS = 1.0
CAMERA_GUIDED_SHARP_CORNER_APPROACH_SECONDS = 1.0
CAMERA_GUIDED_SHARP_CORNER_TURN_SPEED = 0.20
CAMERA_GUIDED_SHARP_CORNER_MIN_TURN_SECONDS = 0.0
CAMERA_GUIDED_SHARP_CORNER_WHITE_CONFIRM_SECONDS = 0.10
# Retain the optional relief timing for replay, but disable relief in this
# reviewed continuous-turn trial. A longer approach is provisional; it does
# not prove the loaded pivot stall is resolved. Keep encoder stopping active.
CAMERA_GUIDED_SHARP_CORNER_PIVOT_SECONDS = 0.45
CAMERA_GUIDED_SHARP_CORNER_RELIEF_SECONDS = 0.25
CAMERA_GUIDED_SHARP_CORNER_RELIEF_INNER_SPEED = 0.0
CAMERA_GUIDED_SHARP_CORNER_MAX_TURN_SECONDS = 10.0
CAMERA_GUIDED_SHARP_CORNER_REACQUIRE_SECONDS = 0.30
CAMERA_GUIDED_SHARP_CORNER_TRIGGER_ERROR = 0.10
CAMERA_GUIDED_SHARP_CORNER_EXIT_ERROR = 0.13
# Provisional one-junction test settings.  These are isolated from ordinary
# launchers and must be calibrated one direction at a time on the track.
JUNCTION_TEST_ROUTES = {
    "straight": ["A", "B", "C"],
    "left": ["A", "B", "D"],
    "right": ["A", "B", "E"],
}
JUNCTION_STOP_HOLD_SECONDS = 2.0
JUNCTION_ENTRY_SECONDS = 0.5
JUNCTION_STRAIGHT_SECONDS = 1.0
JUNCTION_LEFT_SECONDS = 1.6
JUNCTION_RIGHT_SECONDS = 1.6
JUNCTION_REACQUIRE_TIMEOUT = 3.0
JUNCTION_STRAIGHT_SPEED = 0.09
# The left pair references the accepted fixed left curve; the right pair
# references the accepted sharp-right bend. Neither is yet a junction result.
JUNCTION_LEFT_SPEED = 0.09
JUNCTION_LEFT_BIAS = 0.06
JUNCTION_RIGHT_SPEED = 0.10
JUNCTION_RIGHT_BIAS = 0.10
JUNCTION_POST_REACQUIRE_SECONDS = 1.0
MOTION_FEEDBACK_TIMEOUT = 0.5
ENCODER_FEEDBACK_TIMEOUT = 0.5
ENCODER_STALL_TIMEOUT = 0.75
ENCODER_COMMAND_THRESHOLD = 0.07
WATCHDOG_NODE = "/duck2_ground_watchdog"


def validate_limits(duration, base_speed, max_speed, max_steering):
    values = (duration, base_speed, max_speed, max_steering)
    if any(type(value) is bool or not math.isfinite(value) for value in values):
        raise ValueError("Ground-check limits must be finite numbers")
    if not 0 < duration <= MAX_DURATION:
        raise ValueError("Ground-check duration must be in (0, 8.0] seconds")
    if not 0 < base_speed <= MAX_BASE_SPEED:
        raise ValueError("Ground-check base speed exceeds 0.05")
    if not base_speed <= max_speed <= MAX_WHEEL_SPEED:
        raise ValueError("Ground-check wheel cap must be between base speed and 0.05")
    if not 0 <= max_steering <= MAX_STEERING:
        raise ValueError("Ground-check steering cap exceeds 0.02")


def fixed_turn_requested(args):
    return args.fixed_left is not None or args.fixed_right is not None


def validate_fixed_turn(duration, left, right):
    values = (duration, left, right)
    if any(type(value) is bool or not math.isfinite(value) for value in values):
        raise ValueError("Fixed-turn values must be finite numbers")
    established = (math.isclose(left, FIXED_TURN_LEFT, abs_tol=1e-9)
                   and math.isclose(right, FIXED_TURN_RIGHT, abs_tol=1e-9))
    upside_down_pivot = (
        math.isclose(left, UPSIDEDOWN_LEFT_PIVOT_LEFT, abs_tol=1e-9)
        and math.isclose(right, UPSIDEDOWN_LEFT_PIVOT_RIGHT, abs_tol=1e-9))
    upside_down_load_profile = (
        math.isclose(left, UPSIDEDOWN_LOAD_PROFILE_LEFT, abs_tol=1e-9)
        and math.isclose(right, UPSIDEDOWN_LOAD_PROFILE_RIGHT, abs_tol=1e-9))
    ground_rolling_right = (
        math.isclose(left, GROUND_ROLLING_RIGHT_LEFT, abs_tol=1e-9)
        and math.isclose(right, GROUND_ROLLING_RIGHT_RIGHT, abs_tol=1e-9))
    ground_equal_wheels = (
        math.isclose(left, GROUND_EQUAL_WHEELS_LEFT, abs_tol=1e-9)
        and math.isclose(right, GROUND_EQUAL_WHEELS_RIGHT, abs_tol=1e-9))
    if established:
        if not 0 < duration <= MAX_FIXED_TURN_DURATION:
            raise ValueError("Fixed-turn duration must be in (0, 8.0] seconds")
    elif (upside_down_pivot or upside_down_load_profile
          or ground_rolling_right or ground_equal_wheels):
        if not 0 < duration <= MAX_UPSIDEDOWN_LEFT_PIVOT_DURATION:
            raise ValueError("Diagnostic duration must be in (0, 2.0] seconds")
    else:
        raise ValueError("Fixed-turn command is not an approved diagnostic profile")


def apply_camera_guided_preset(args):
    """Apply the one reviewed curve-to-straight test configuration."""
    if fixed_turn_requested(args):
        raise ValueError("Camera-guided and fixed-turn modes cannot be combined")
    if (type(args.duration) is bool or not math.isfinite(args.duration)
            or not 0 < args.duration <= MAX_CAMERA_GUIDED_DURATION):
        raise ValueError("Camera-guided duration must be in (0, 15.0] seconds")
    args.base_speed = CAMERA_GUIDED_BASE_SPEED
    args.max_speed = CAMERA_GUIDED_MAX_SPEED
    args.max_steering = CAMERA_GUIDED_MAX_STEERING
    args.min_active_wheel_speed = CAMERA_GUIDED_MIN_ACTIVE_WHEEL_SPEED
    args.taper_inner_wheel_floor = CAMERA_GUIDED_TAPER_INNER_WHEEL_FLOOR
    args.k_p = CAMERA_GUIDED_K_P
    args.near_center_k_p = CAMERA_GUIDED_NEAR_CENTER_K_P
    args.full_gain_error = CAMERA_GUIDED_FULL_GAIN_ERROR
    args.lane_target_fraction = CAMERA_GUIDED_LANE_TARGET_FRACTION
    args.alpha = CAMERA_GUIDED_ALPHA
    args.deadband = CAMERA_GUIDED_DEADBAND
    args.steering_bias = CAMERA_GUIDED_STEERING_BIAS
    args.smooth_steering_deadband = True
    args.temporal_lane_width_fallback = CAMERA_GUIDED_TEMPORAL_LANE_WIDTH_FALLBACK
    args.temporal_lane_width_timeout = CAMERA_GUIDED_TEMPORAL_LANE_WIDTH_TIMEOUT
    args.temporal_yellow_only_timeout = CAMERA_GUIDED_TEMPORAL_YELLOW_ONLY_TIMEOUT
    args.boundary_risk_stop = CAMERA_GUIDED_BOUNDARY_RISK_STOP
    args.white_boundary_risk_fraction = CAMERA_GUIDED_WHITE_BOUNDARY_RISK_FRACTION
    args.yellow_lower = list(CAMERA_GUIDED_YELLOW_LOWER)
    args.yellow_upper = list(CAMERA_GUIDED_YELLOW_UPPER)
    args.white_lower = list(CAMERA_GUIDED_WHITE_LOWER)
    args.white_upper = list(CAMERA_GUIDED_WHITE_UPPER)
    args.sharp_corner_enabled = CAMERA_GUIDED_SHARP_CORNER_ENABLED
    args.sharp_corner_confirm_seconds = CAMERA_GUIDED_SHARP_CORNER_CONFIRM_SECONDS
    args.sharp_corner_recent_lane_seconds = CAMERA_GUIDED_SHARP_CORNER_RECENT_LANE_SECONDS
    args.sharp_corner_approach_seconds = CAMERA_GUIDED_SHARP_CORNER_APPROACH_SECONDS
    args.sharp_corner_turn_speed = CAMERA_GUIDED_SHARP_CORNER_TURN_SPEED
    args.sharp_corner_min_turn_seconds = CAMERA_GUIDED_SHARP_CORNER_MIN_TURN_SECONDS
    args.sharp_corner_white_confirm_seconds = CAMERA_GUIDED_SHARP_CORNER_WHITE_CONFIRM_SECONDS
    args.sharp_corner_pivot_seconds = CAMERA_GUIDED_SHARP_CORNER_PIVOT_SECONDS
    args.sharp_corner_relief_seconds = CAMERA_GUIDED_SHARP_CORNER_RELIEF_SECONDS
    args.sharp_corner_relief_inner_speed = CAMERA_GUIDED_SHARP_CORNER_RELIEF_INNER_SPEED
    args.sharp_corner_max_turn_seconds = CAMERA_GUIDED_SHARP_CORNER_MAX_TURN_SECONDS
    args.sharp_corner_reacquire_seconds = CAMERA_GUIDED_SHARP_CORNER_REACQUIRE_SECONDS
    args.sharp_corner_trigger_error = CAMERA_GUIDED_SHARP_CORNER_TRIGGER_ERROR
    args.sharp_corner_exit_error = CAMERA_GUIDED_SHARP_CORNER_EXIT_ERROR


def apply_junction_preset(args):
    """Apply one bounded, route-backed intersection test configuration."""
    if args.junction_turn not in JUNCTION_TEST_ROUTES:
        raise ValueError("Junction turn must be straight, left, or right")
    if (type(args.red_stop_trigger_bottom_fraction) is bool
            or not math.isfinite(args.red_stop_trigger_bottom_fraction)
            or not .65 <= args.red_stop_trigger_bottom_fraction < .95):
        raise ValueError("Red-stop trigger must be in [0.65, 0.95)")
    apply_camera_guided_preset(args)
    # Road-bend recognition must never compete with an authorized junction.
    args.sharp_corner_enabled = False
    args.stop_hold_seconds = JUNCTION_STOP_HOLD_SECONDS
    args.junction_entry_seconds = JUNCTION_ENTRY_SECONDS
    args.junction_straight_seconds = JUNCTION_STRAIGHT_SECONDS
    args.junction_left_seconds = JUNCTION_LEFT_SECONDS
    args.junction_right_seconds = JUNCTION_RIGHT_SECONDS
    args.junction_reacquire_timeout = JUNCTION_REACQUIRE_TIMEOUT
    args.junction_straight_speed = JUNCTION_STRAIGHT_SPEED
    args.junction_left_speed = JUNCTION_LEFT_SPEED
    args.junction_left_bias = JUNCTION_LEFT_BIAS
    args.junction_right_speed = JUNCTION_RIGHT_SPEED
    args.junction_right_bias = JUNCTION_RIGHT_BIAS


def require_camera_guided_mode(status, sharp_corner_expected=True):
    if status.get("smooth_steering_deadband") is not True:
        raise RuntimeError("Lane node did not confirm smooth steering; update the test node before release")
    if status.get("temporal_lane_width_fallback") is not True:
        raise RuntimeError("Lane node did not confirm temporal lane-width fallback before release")
    if not math.isclose(status.get("temporal_lane_width_timeout", float("nan")),
                        CAMERA_GUIDED_TEMPORAL_LANE_WIDTH_TIMEOUT, abs_tol=1e-9):
        raise RuntimeError("Lane node did not confirm the boundary-gap timeout before release")
    if not math.isclose(status.get("temporal_yellow_only_timeout", float("nan")),
                        CAMERA_GUIDED_TEMPORAL_YELLOW_ONLY_TIMEOUT, abs_tol=1e-9):
        raise RuntimeError("Lane node did not confirm the yellow-only timeout before release")
    if status.get("boundary_risk_stop") is not True:
        raise RuntimeError("Lane node did not confirm boundary-risk stopping before release")
    if not math.isclose(status.get("white_boundary_risk_fraction", float("nan")),
                        CAMERA_GUIDED_WHITE_BOUNDARY_RISK_FRACTION, abs_tol=1e-9):
        raise RuntimeError("Lane node did not confirm the white-boundary guard before release")
    if not math.isclose(status.get("max_steering", float("nan")),
                        CAMERA_GUIDED_MAX_STEERING, abs_tol=1e-9):
        raise RuntimeError("Lane node did not confirm the bounded steering cap before release")
    if not math.isclose(status.get("min_active_wheel_speed", float("nan")),
                        CAMERA_GUIDED_MIN_ACTIVE_WHEEL_SPEED, abs_tol=1e-9):
        raise RuntimeError("Lane node did not confirm the active-wheel floor before release")
    if status.get("taper_inner_wheel_floor") is not CAMERA_GUIDED_TAPER_INNER_WHEEL_FLOOR:
        raise RuntimeError("Lane node did not confirm the reviewed inner-wheel-floor mode before release")
    if status.get("sharp_corner_enabled") is not sharp_corner_expected:
        raise RuntimeError("Lane node did not confirm sharp-corner mode before release")
    for field, expected in (
            ("k_p", CAMERA_GUIDED_K_P),
            ("near_center_k_p", CAMERA_GUIDED_NEAR_CENTER_K_P),
            ("full_gain_error", CAMERA_GUIDED_FULL_GAIN_ERROR)):
        if not math.isclose(status.get(field, float("nan")), expected, abs_tol=1e-9):
            raise RuntimeError("Lane node did not confirm {} before release".format(field))
    if (status.get("yellow_lower") != CAMERA_GUIDED_YELLOW_LOWER
            or status.get("yellow_upper") != CAMERA_GUIDED_YELLOW_UPPER):
        raise RuntimeError("Lane node did not confirm the reviewed yellow range before release")
    if (status.get("white_lower") != CAMERA_GUIDED_WHITE_LOWER
            or status.get("white_upper") != CAMERA_GUIDED_WHITE_UPPER):
        raise RuntimeError("Lane node did not confirm the reviewed white range before release")
    for field, expected in (
            ("sharp_corner_confirm_seconds", CAMERA_GUIDED_SHARP_CORNER_CONFIRM_SECONDS),
            ("sharp_corner_recent_lane_seconds", CAMERA_GUIDED_SHARP_CORNER_RECENT_LANE_SECONDS),
            ("sharp_corner_approach_seconds", CAMERA_GUIDED_SHARP_CORNER_APPROACH_SECONDS),
            ("sharp_corner_turn_speed", CAMERA_GUIDED_SHARP_CORNER_TURN_SPEED),
            ("sharp_corner_min_turn_seconds", CAMERA_GUIDED_SHARP_CORNER_MIN_TURN_SECONDS),
            ("sharp_corner_white_confirm_seconds", CAMERA_GUIDED_SHARP_CORNER_WHITE_CONFIRM_SECONDS),
            ("sharp_corner_pivot_seconds", CAMERA_GUIDED_SHARP_CORNER_PIVOT_SECONDS),
            ("sharp_corner_relief_seconds", CAMERA_GUIDED_SHARP_CORNER_RELIEF_SECONDS),
            ("sharp_corner_relief_inner_speed", CAMERA_GUIDED_SHARP_CORNER_RELIEF_INNER_SPEED),
            ("sharp_corner_max_turn_seconds", CAMERA_GUIDED_SHARP_CORNER_MAX_TURN_SECONDS),
            ("sharp_corner_reacquire_seconds", CAMERA_GUIDED_SHARP_CORNER_REACQUIRE_SECONDS),
            ("sharp_corner_trigger_error", CAMERA_GUIDED_SHARP_CORNER_TRIGGER_ERROR),
            ("sharp_corner_exit_error", CAMERA_GUIDED_SHARP_CORNER_EXIT_ERROR)):
        if not math.isclose(status.get(field, float("nan")), expected, abs_tol=1e-9):
            raise RuntimeError("Lane node did not confirm {} before release".format(field))


def require_junction_mode(status, turn, red_trigger=0.65):
    """Reject release unless the updated node confirms the selected profile."""
    require_camera_guided_mode(status, sharp_corner_expected=False)
    if status.get("route_enabled") is not True:
        raise RuntimeError("Lane node did not confirm route mode")
    if status.get("junctions_calibrated") is not True:
        raise RuntimeError("Lane node did not confirm supervised junction settings")
    if not math.isclose(
            status.get("red_stop_trigger_bottom_fraction", float("nan")),
            red_trigger,
            abs_tol=1e-9):
        raise RuntimeError("Lane node reported an invalid red-stop trigger")
    settings = status.get("junction_settings") or {}
    expected = {
        "entry_seconds": JUNCTION_ENTRY_SECONDS,
        "straight_seconds": JUNCTION_STRAIGHT_SECONDS,
        "left_seconds": JUNCTION_LEFT_SECONDS,
        "right_seconds": JUNCTION_RIGHT_SECONDS,
        "reacquire_timeout": JUNCTION_REACQUIRE_TIMEOUT,
        "straight_speed": JUNCTION_STRAIGHT_SPEED,
        "left_speed": JUNCTION_LEFT_SPEED,
        "right_speed": JUNCTION_RIGHT_SPEED,
        "left_bias": JUNCTION_LEFT_BIAS,
        "right_bias": JUNCTION_RIGHT_BIAS,
    }
    for field, value in expected.items():
        if not math.isclose(settings.get(field, float("nan")), value, abs_tol=1e-9):
            raise RuntimeError("Lane node did not confirm junction {}".format(field))
    route = JUNCTION_TEST_ROUTES[turn]
    if status.get("route") != route or status.get("route_index") != 1:
        raise RuntimeError("Lane node did not confirm the supervised junction route")
    if status.get("state") != "following" or status.get("manual_stop") is not False:
        raise RuntimeError("Lane node did not confirm an armed approach")
def require_two_boundary_preflight(status):
    """Release a camera-guided run only from a measured, complete lane."""
    if status.get("lane_both_visible") is not True:
        raise RuntimeError(
            "Camera-guided release requires visible yellow and white boundaries")


def camera_guided_scene_issue(status):
    """Return the concrete reason a stopped camera-guided scene is unsafe.

    A bend is a valid starting scene.  What matters is that the current frame
    has a complete, usable lane and no active red-line stop; this check must
    not be described as a straight-road requirement.
    """
    if not status or status.get("camera_valid") is not True:
        return "no current valid camera status"
    if status.get("red_stop") is True:
        return "red-stop detection is active"
    lane_error = status.get("lane_error")
    if (type(lane_error) is bool or not isinstance(lane_error, (int, float))
            or not math.isfinite(lane_error)):
        return "no finite lane estimate ({})".format(
            status.get("lane_diagnostic", "no perception diagnostic"))
    try:
        require_two_boundary_preflight(status)
    except RuntimeError as error:
        return str(error)
    return None


def encoder_motion_fault(now, executed_samples, left_encoder_samples,
                         right_encoder_samples, released_at):
    """Detect a commanded wheel that has produced no encoder progress."""
    if released_at is None or now - released_at < ENCODER_STALL_TIMEOUT:
        return None
    window_start = now - ENCODER_STALL_TIMEOUT
    for wheel, command_index, samples in (
            ("left", 1, left_encoder_samples),
            ("right", 2, right_encoder_samples)):
        preceding = [s for s in executed_samples if s[0] <= window_start]
        commands = [sample for sample in executed_samples
                    if window_start <= sample[0] <= now]
        # A command remains active between messages. A sparse or bursty echo
        # stream must not repeatedly reset evidence of a sustained stall.
        if preceding:
            commands.insert(0, preceding[-1])
        if (not commands or commands[0][0] > window_start
                or now - commands[-1][0] > MOTION_FEEDBACK_TIMEOUT
                or any(b[0]-a[0] > MOTION_FEEDBACK_TIMEOUT
                       for a, b in zip(commands, commands[1:]))
                or any(abs(sample[command_index]) < ENCODER_COMMAND_THRESHOLD
                       for sample in commands)):
            continue
        available = [sample for sample in samples if sample[0] <= now]
        if (not available
                or now - available[-1][0] > ENCODER_FEEDBACK_TIMEOUT):
            return "{} wheel encoder feedback became stale".format(wheel.capitalize())
        ticks = [value for stamp, value in available if stamp >= window_start]
        if len(ticks) < 2 or max(ticks) == min(ticks):
            return "{} wheel stalled while commanded".format(wheel.capitalize())
    return None


def motion_fault(now, camera_fresh, status_sample, lane_running,
                 topic_publishers, expected_publishers, executed_samples,
                 require_lane_status, left_encoder_samples=None,
                 right_encoder_samples=None, released_at=None,
                 junction_mode=False):
    """Return the first reason that requires an early bounded-test stop."""
    if not camera_fresh:
        return "Camera became stale"
    if set(topic_publishers) != set(expected_publishers):
        return "Wheel publisher ownership changed"
    if lane_running is False:
        return "Lane follower exited"
    if (not executed_samples
            or now - executed_samples[-1][0] > MOTION_FEEDBACK_TIMEOUT):
        return "Executed-wheel feedback became stale"
    if left_encoder_samples is not None and right_encoder_samples is not None:
        encoder_fault = encoder_motion_fault(
            now, executed_samples, left_encoder_samples,
            right_encoder_samples, released_at)
        if encoder_fault is not None:
            return encoder_fault
    if require_lane_status:
        if status_sample is None or now - status_sample[0] > MOTION_FEEDBACK_TIMEOUT:
            return "Lane status became stale"
        status = status_sample[1]
        if not status.get("camera_valid"):
            return "Lane follower rejected the camera"
        if status.get("red_stop") and not junction_mode:
            return "Red stop line"
        if status.get("fault"):
            return "Lane follower fault: {}".format(status["fault"])
        if status.get("lane_error") is None:
            unmarked_junction = (
                junction_mode
                and status.get("state") in ("crossing", "reacquiring")
                and status.get("junction_phase") in (
                    "entry", "straight", "turning", "searching"))
            # The latched pivot does not steer from an inferred lane centre.
            # Expiry of the ordinary width estimate must not shorten its
            # separate deadline while a current yellow boundary remains.
            waiting_for_white = (
                status.get("sharp_corner_enabled") is True
                and status.get("sharp_corner_state") == "turning"
                and status.get("sharp_corner_phase") == "pivot"
                and status.get("yellow_boundary_visible") is True
                and status.get("white_boundary_visible") is False
                and status.get("lane_diagnostic") == "Lane lost: boundary gap exceeded"
            )
            if not waiting_for_white and not unmarked_junction:
                return "Lane lost"
    return None


def junction_completion_status(status, expected_turn):
    """Return whether one selected junction has been visually reacquired."""
    if not status:
        return False
    result = status.get("junction_last_result") or {}
    return (status.get("state") == "following"
            and status.get("junction_phase") == "complete"
            and status.get("route_index") == 2
            and result.get("outcome") == "reacquired"
            and result.get("turn") == expected_turn)


def encoder_delta_in_window(samples, started_at, stopped_at):
    """Measure tick change from the last pre-release sample to the stop deadline."""
    before = [value for stamp, value in samples if stamp <= started_at]
    through_stop = [value for stamp, value in samples if stamp <= stopped_at]
    if not before or not through_stop:
        return None
    return through_stop[-1] - before[-1]


def wait_until(deadline, monotonic=time.monotonic, sleep=time.sleep):
    """Wait only until an absolute monotonic deadline."""
    while True:
        remaining = deadline - monotonic()
        if remaining <= 0:
            return
        sleep(min(remaining, 0.01))


def parent_pipe_alive_until(deadline, stream, selector=select.select,
                            monotonic=time.monotonic):
    """Watch the parent pipe until the deadline; EOF means control was lost."""
    while True:
        remaining = deadline - monotonic()
        if remaining <= 0:
            return True
        readable, _, _ = selector([stream], [], [], min(0.05, remaining))
        if readable and stream.read(1) == "":
            return False


def fixed_command_due(now, next_publish_at, interval=0.1):
    """Return whether a fixed diagnostic command must be refreshed."""
    if now < next_publish_at:
        return False, next_publish_at
    return True, now + interval


def preflight_zero_feedback_ready(samples, stop_sent_at):
    """Require an executed zero received after this preflight's stop command."""
    return any(
        received_at >= stop_sent_at
        and abs(left) <= 1e-7 and abs(right) <= 1e-7
        for received_at, left, right in samples
    )


def read_motor_registers(bus):
    """Read only the verified DB21J/HATv3 PWM registers; never initialize a HAT."""
    # Constructing the installed PWM/HAT/Motor class would write/reset outputs.
    # Use the bus's read operation only. These snapshots can overlap a driver
    # write; two matching snapshots are required before treating them as stable.
    registers = [0x00, 0x01, 0xFE]
    for channel in (8, 9, 10, 13):
        registers.extend(range(0x06 + 4*channel, 0x0A + 4*channel))
    first = [bus.read_byte_data(0x60, reg) for reg in registers]
    second = [bus.read_byte_data(0x60, reg) for reg in registers]
    return {'stable': first == second,
            'registers': dict(zip(map(str, registers), second))}


def motor_register_reader():
    """Optional read-only child; failure is reported, never repaired on the bot."""
    try:
        from dt_robot_utils import get_robot_configuration
        import smbus
        if get_robot_configuration().name != 'DB21J':
            raise RuntimeError('Readback pin/bus mapping is verified only for DB21J')
        bus = smbus.SMBus(1)
        try:
            deadline = time.monotonic() + 30.0
            while time.monotonic() < deadline:
                started = time.monotonic()
                result = read_motor_registers(bus)
                result.update(started_monotonic_s=started,
                              finished_monotonic_s=time.monotonic())
                print(json.dumps(result), flush=True)
                readable, _, _ = select.select([sys.stdin], [], [], .2)
                if readable and sys.stdin.read(1) == '':
                    break
        finally:
            bus.close()
        return 0
    except Exception as error:
        print(json.dumps({'readback_error': str(error)}), flush=True)
        return 1


def finish_reader(process, log):
    if process is not None:
        if process.stdin:
            try:
                process.stdin.close()
            except OSError:
                pass
        try:
            process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1)
    if log is not None:
        log.close()


def wheel_response(left, right, expected_left, expected_right):
    """Encoder evidence, not motor power, heading calibration or lane success."""
    if left is None or right is None or left < 0 or right < 0:
        return {'assessment': 'unavailable'}
    total = left + right
    fraction = (left-right)/total if total else None
    if expected_left == expected_right:
        assessment = 'equal_command_comparison'
    elif not total:
        assessment = 'no_encoder_progress'
    else:
        sign = 1 if expected_left > expected_right else -1
        # Advisory threshold only. It is not a measured turn radius or an
        # automatic control input; wheel geometry and slip remain uncalibrated.
        assessment = ('weak_or_wrong_turn_response' if sign*fraction < .20
                      else 'encoder_turn_response_present')
    return {'assessment': assessment, 'left_minus_right_ticks': left-right,
            'differential_fraction': fraction, 'physical_success': 'unverified'}


def summarize_samples(samples, released_at, stopped_at):
    active = [(left, right) for stamp, left, right in samples
              if released_at <= stamp < stopped_at
              and (abs(left) > 1e-7 or abs(right) > 1e-7)]
    final = [(left, right) for stamp, left, right in samples if stamp >= stopped_at]
    return {
        "nonzero_executed_samples": len(active),
        "executed_left_range": ([min(v[0] for v in active), max(v[0] for v in active)]
                                if active else None),
        "executed_right_range": ([min(v[1] for v in active), max(v[1] for v in active)]
                                 if active else None),
        "post_stop_samples": len(final),
        "post_stop_all_zero": bool(final) and all(
            abs(left) <= 1e-7 and abs(right) <= 1e-7 for left, right in final
        ),
    }


def latest_samples_zero(samples, count=8):
    """Require recent driver feedback to confirm the completed stop sequence."""
    recent = samples[-count:]
    return len(recent) == count and all(
        abs(left) <= 1e-7 and abs(right) <= 1e-7
        for _, left, right in recent
    )


def post_stop_zero_confirmed(samples, stopped_at, count=8):
    """Confirm a fresh consecutive zero-feedback window after stopping."""
    post_stop = [sample for sample in samples if sample[0] >= stopped_at]
    return latest_samples_zero(post_stop, count)


def wait_for_post_stop_zero(samples, stopped_at, timeout_s,
                            monotonic=time.monotonic, sleep=time.sleep,
                            count=8):
    """Wait only a bounded interval for new driver zero feedback after stop."""
    deadline = monotonic() + timeout_s
    while monotonic() < deadline:
        if post_stop_zero_confirmed(samples, stopped_at, count):
            return True
        sleep(min(0.02, max(0.0, deadline - monotonic())))
    return post_stop_zero_confirmed(samples, stopped_at, count)


def summarize_status_samples(samples, released_at, stopped_at):
    active = [status for stamp, status in samples
              if released_at <= stamp <= stopped_at]
    ages = [float(status["camera_age"]) for status in active
            if (isinstance(status.get("camera_age"), (int, float))
                and not isinstance(status.get("camera_age"), bool)
                and math.isfinite(status["camera_age"]))]
    steering = []
    for status in active:
        wheels = status.get("wheel_speeds")
        if (isinstance(wheels, list) and len(wheels) == 2
                and all(isinstance(value, (int, float))
                        and not isinstance(value, bool) and math.isfinite(value)
                        for value in wheels)):
            steering.append((float(wheels[1]) - float(wheels[0])) / 2.0)
    return {
        "lane_status_samples": len(active),
        "camera_age_range_s": [min(ages), max(ages)] if ages else None,
        "requested_steering_range": ([min(steering), max(steering)]
                                     if steering else None),
    }


def summarize_red_line_samples(samples):
    """Summarize stationary red geometry without claiming physical distance."""
    detections = []
    triggered = 0
    for _, status in samples:
        detection = status.get("red_line_detection")
        if not isinstance(detection, dict):
            continue
        bottom = detection.get("bottom_fraction")
        width = detection.get("width_fraction")
        area = detection.get("area_fraction")
        values = (bottom, width, area)
        if not all(isinstance(value, (int, float))
                   and not isinstance(value, bool) and math.isfinite(value)
                   for value in values):
            continue
        detections.append(tuple(float(value) for value in values))
        triggered += int(detection.get("triggered") is True)
    if not detections:
        return {
            "valid_detection_samples": 0,
            "triggered_samples": 0,
            "bottom_fraction_range": None,
            "median_bottom_fraction": None,
            "width_fraction_range": None,
            "area_fraction_range": None,
        }
    bottoms = sorted(value[0] for value in detections)
    widths = [value[1] for value in detections]
    areas = [value[2] for value in detections]
    middle = len(bottoms) // 2
    median = (bottoms[middle] if len(bottoms) % 2 else
              (bottoms[middle - 1] + bottoms[middle]) / 2.0)
    return {
        "valid_detection_samples": len(detections),
        "triggered_samples": triggered,
        "bottom_fraction_range": [min(bottoms), max(bottoms)],
        "median_bottom_fraction": median,
        "width_fraction_range": [min(widths), max(widths)],
        "area_fraction_range": [min(areas), max(areas)],
    }


class CameraEvidenceRecorder:
    """Save timestamped compressed frames beside a bounded physical test."""

    def __init__(self, output_dir, frame_rate=4.0):
        if (type(frame_rate) is bool or not math.isfinite(frame_rate)
                or not 1.0 <= frame_rate <= 10.0):
            raise ValueError("Evidence frame rate must be between 1 and 10 Hz")
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        if any(self.output_dir.iterdir()):
            raise ValueError(
                "Evidence directory must be new or empty: {}".format(
                    self.output_dir))
        self.manifest = (self.output_dir / "frames.jsonl").open("w")
        self._lock = threading.Lock()
        self._frozen = False
        self._closed = False
        self.frame_interval = 1.0 / frame_rate
        self.last_saved_at = -float("inf")
        self.last_received_at = None
        self.last_stamp = None
        self.motion_started_at = None
        self.count = 0

    def callback(self, message):
        with self._lock:
            if self._frozen:
                return
            now = time.monotonic()
            stamp = message.header.stamp.to_sec()
            self.last_received_at = now
            self.last_stamp = stamp
            if now - self.last_saved_at < self.frame_interval:
                return
            filename = "frame-%05d.jpg" % self.count
            (self.output_dir / filename).write_bytes(bytes(message.data))
            row = {
                "file": filename,
                "received_monotonic_s": now,
                "camera_stamp_s": stamp,
                "phase": "motion" if self.motion_started_at is not None else "preflight",
            }
            self.manifest.write(json.dumps(row, sort_keys=True) + "\n")
            self.manifest.flush()
            self.last_saved_at = now
            self.count += 1

    def is_fresh(self, rospy, timeout=0.5, future_limit=0.1):
        with self._lock:
            if self.last_received_at is None or self.last_stamp is None or self.last_stamp <= 0:
                return False
            if time.monotonic() - self.last_received_at > timeout:
                return False
            age = rospy.Time.now().to_sec() - self.last_stamp
            return -future_limit <= age <= timeout

    def freeze(self):
        """Stop accepting callbacks before counting files for completion."""
        with self._lock:
            self._frozen = True
            self.manifest.flush()
            return self.count

    def close(self):
        with self._lock:
            self._frozen = True
            if not self._closed:
                self.manifest.close()
                self._closed = True


def write_evidence_telemetry(output_dir, mode, released_at, stopped_at,
                             requested, executed, left_ticks, right_ticks,
                             status_samples=None, early_stop_reason=None):
    """Keep raw timestamped ROS observations for one bounded test."""
    payload = {
        "mode": mode,
        "motion_window_monotonic_s": {
            "released": released_at,
            "stopped": stopped_at,
        },
        "requested_wheels": requested,
        "executed_wheels": executed,
        "left_encoder_ticks": left_ticks,
        "right_encoder_ticks": right_ticks,
        "lane_status": status_samples or [],
        "early_stop_reason": early_stop_reason,
    }
    destination = Path(output_dir) / "telemetry.json"
    destination.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    return destination


def write_evidence_completion(output_dir, summary):
    """Write the final run result into the evidence bundle itself."""
    payload = {
        "complete": True,
        "summary": summary,
    }
    destination = Path(output_dir) / "completion.json"
    destination.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    return destination


def record_aborted_run(recorder, mode, released_at, requested, executed,
                       left_ticks, right_ticks, status_samples, reason, stop_again):
    """Called only after temporary controllers have exited; retain failed runs."""
    stopped_at = time.monotonic()
    stop_again()
    zero_confirmed = wait_for_post_stop_zero(executed, stopped_at, timeout_s=1.0)
    frames = recorder.freeze()
    write_evidence_telemetry(
        recorder.output_dir, mode, released_at, stopped_at,
        requested, executed, left_ticks, right_ticks,
        status_samples=status_samples, early_stop_reason=reason)
    summary = {
        "mode": mode, "motion_started": released_at is not None,
        "early_stop_reason": reason, "evidence_frames": frames,
        "final_feedback_all_zero": zero_confirmed,
        "post_stop_zero_confirmed": zero_confirmed,
        "last_lane_status": status_samples[-1][1] if status_samples else None,
    }
    write_evidence_completion(recorder.output_dir, summary)
    return summary


def ros_types():
    import rosgraph
    import rospy
    from duckietown_msgs.msg import BoolStamped, WheelEncoderStamped, WheelsCmdStamped
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    return (rosgraph, rospy, BoolStamped, WheelEncoderStamped, WheelsCmdStamped,
            CompressedImage, String)


def publishers(rospy, BoolStamped, WheelsCmdStamped, vehicle):
    stop = rospy.Publisher(
        "/{}/wheels_driver_node/emergency_stop".format(vehicle),
        BoolStamped,
        queue_size=1,
        latch=True,
    )
    wheels = rospy.Publisher(
        "/{}/wheels_driver_node/wheels_cmd".format(vehicle),
        WheelsCmdStamped,
        queue_size=1,
    )
    return stop, wheels


def publish_estop(rospy, BoolStamped, publisher, value, repeats=4):
    for _ in range(repeats):
        message = BoolStamped()
        message.header.stamp = rospy.Time.now()
        message.data = bool(value)
        publisher.publish(message)
        time.sleep(0.02)


def publish_wheel_command(rospy, WheelsCmdStamped, publisher, left, right, repeats=4):
    for _ in range(repeats):
        message = WheelsCmdStamped()
        message.header.stamp = rospy.Time.now()
        message.vel_left = left
        message.vel_right = right
        publisher.publish(message)
        time.sleep(0.02)


def publish_stop(rospy, BoolStamped, WheelsCmdStamped, stop, wheels, repeats=8):
    for _ in range(repeats):
        now = rospy.Time.now()
        stopped = BoolStamped()
        stopped.header.stamp = now
        stopped.data = True
        zero = WheelsCmdStamped()
        zero.header.stamp = now
        zero.vel_left = 0.0
        zero.vel_right = 0.0
        stop.publish(stopped)
        wheels.publish(zero)
        time.sleep(0.02)


def publish_json_command(rospy, String, publisher, command, repeats=3):
    """Publish one idempotent high-level command through the existing API."""
    payload = dict(command)
    payload.setdefault("issued_at", time.time())
    message = String(data=json.dumps(payload, sort_keys=True))
    for _ in range(repeats):
        publisher.publish(message)
        time.sleep(0.03)


def command_acknowledged(statuses, command_id):
    for status in reversed(statuses):
        result = status.get("last_command") or {}
        if result.get("id") == command_id:
            if result.get("accepted") is not True:
                raise RuntimeError(
                    "Lane node rejected {}: {}".format(
                        command_id, result.get("reason", "no reason")))
            return True
    return False


def configure_supervised_junction(rospy, String, publisher, statuses, turn,
                                  hold_stopped, timeout=8.0):
    """Set one test route and arm its approach while physical output is held."""
    route = JUNCTION_TEST_ROUTES[turn]
    commands = [
        {
            "id": "junction-set-route-{}".format(turn),
            "action": "set_route",
            "route": route,
            "position_confirmed": True,
        },
        {
            "id": "junction-continue-{}".format(turn),
            "action": "continue",
        },
    ]
    deadline = time.monotonic() + timeout
    for command in commands:
        next_publish = 0.0
        while time.monotonic() < deadline:
            hold_stopped()
            if command_acknowledged(statuses, command["id"]):
                break
            now = time.monotonic()
            if (publisher.get_num_connections() > 0 and statuses
                    and now >= next_publish):
                publish_json_command(rospy, String, publisher, command)
                next_publish = now + .25
            time.sleep(.03)
        else:
            raise RuntimeError(
                "Lane node did not acknowledge {}".format(command["id"]))
    return route


def watchdog_main(args):
    _, rospy, BoolStamped, _, WheelsCmdStamped, _, _ = ros_types()
    # A stable name lets the supervisor distinguish its own watchdog from an
    # unexpected third-party wheel publisher during the motion window.
    rospy.init_node(WATCHDOG_NODE.lstrip("/"), anonymous=False, disable_signals=True)
    stop, wheels = publishers(rospy, BoolStamped, WheelsCmdStamped, args.vehicle)
    time.sleep(0.3)
    publish_stop(rospy, BoolStamped, WheelsCmdStamped, stop, wheels)
    print("WATCHDOG_READY", flush=True)
    command = sys.stdin.readline()
    if command.strip() != "GO":
        publish_stop(rospy, BoolStamped, WheelsCmdStamped, stop, wheels)
        print("WATCHDOG_STOPPED_PARENT_LOST", flush=True)
        return 2
    deadline = time.monotonic() + args.watchdog_delay
    if not parent_pipe_alive_until(deadline, sys.stdin):
        publish_stop(rospy, BoolStamped, WheelsCmdStamped, stop, wheels)
        print("WATCHDOG_STOPPED_PARENT_LOST", flush=True)
        return 2
    publish_stop(rospy, BoolStamped, WheelsCmdStamped, stop, wheels)
    print("WATCHDOG_STOPPED_AT_DEADLINE", flush=True)
    return 0


def wait_for_watchdog(watchdog, timeout=8.0, monotonic=time.monotonic):
    """Wait for the watchdog's exact readiness sentinel within one deadline.

    ROS can write harmless connection diagnostics to the merged stdout/stderr
    pipe while the watchdog node starts. Those lines are retained for failure
    reporting but must not be mistaken for the readiness response itself.
    """
    lines = queue.Queue()

    def read_output():
        try:
            while True:
                raw_line = watchdog.stdout.readline()
                if raw_line == "":
                    lines.put(("eof", None))
                    return
                lines.put(("line", raw_line))
        except BaseException as error:
            lines.put(("error", error))

    # Reading in a daemon thread keeps the deadline effective even if the
    # child leaves its pipe open without producing another complete line. It
    # also avoids select(2) missing a line already buffered by TextIOWrapper.
    threading.Thread(target=read_output, daemon=True).start()

    deadline = monotonic() + timeout
    diagnostics = []
    while True:
        remaining = deadline - monotonic()
        if remaining <= 0:
            detail = " | ".join(diagnostics[-8:])
            suffix = ": {}".format(detail) if detail else ""
            raise RuntimeError(
                "Independent stop watchdog did not become ready{}".format(suffix)
            )

        try:
            kind, payload = lines.get(timeout=remaining)
        except queue.Empty:
            detail = " | ".join(diagnostics[-8:])
            suffix = ": {}".format(detail) if detail else ""
            if watchdog.poll() is not None:
                raise RuntimeError(
                    "Independent stop watchdog exited before readiness "
                    "(code {}){}".format(watchdog.returncode, suffix)
                )
            raise RuntimeError(
                "Independent stop watchdog did not become ready{}".format(suffix)
            )

        if kind == "error":
            raise RuntimeError(
                "Independent stop watchdog output failed: {}".format(payload)
            )
        if kind == "eof":
            detail = " | ".join(diagnostics[-8:])
            suffix = ": {}".format(detail) if detail else ""
            watchdog.poll()
            raise RuntimeError(
                "Independent stop watchdog exited before readiness "
                "(code {}){}".format(watchdog.returncode, suffix)
            )

        line = payload.strip()
        if line == "WATCHDOG_READY":
            return
        if line:
            diagnostics.append(line)
            diagnostics = diagnostics[-8:]


def supervisor_main(args):
    fixed_turn = fixed_turn_requested(args)
    camera_guided = args.camera_guided_curve
    junction_mode = args.junction_turn is not None
    red_line_inspection = args.inspect_red_line
    selected_modes = sum(bool(value) for value in (
        fixed_turn, camera_guided, junction_mode, red_line_inspection))
    if selected_modes > 1:
        raise ValueError(
            "Inspection, junction, camera-guided and fixed-turn modes are exclusive")
    if red_line_inspection:
        apply_camera_guided_preset(args)
        args.sharp_corner_enabled = False
    elif junction_mode:
        apply_junction_preset(args)
    elif camera_guided:
        apply_camera_guided_preset(args)
    elif fixed_turn:
        if args.fixed_left is None or args.fixed_right is None:
            raise ValueError("Fixed-turn mode requires both --fixed-left and --fixed-right")
        validate_fixed_turn(args.duration, args.fixed_left, args.fixed_right)
    else:
        validate_limits(args.duration, args.base_speed, args.max_speed, args.max_steering)
    if not args.acknowledge_upright_clear:
        raise ValueError("Ground motion requires --acknowledge-upright-clear")
    if not fixed_turn and not os.path.isfile(args.node_path):
        raise ValueError("Lane-follower source is missing: {}".format(args.node_path))

    (rosgraph, rospy, BoolStamped, WheelEncoderStamped, WheelsCmdStamped,
     CompressedImage, String) = ros_types()
    rospy.init_node("duck2_bounded_ground_supervisor", anonymous=False, disable_signals=True)
    stop, wheels = publishers(rospy, BoolStamped, WheelsCmdStamped, args.vehicle)
    command_publisher = rospy.Publisher(
        "/{}/lane_follower/command".format(args.vehicle), String,
        queue_size=1, latch=False)
    requested, executed, left_ticks, right_ticks = [], [], [], []
    statuses, status_samples = [], []
    wheel_topic = "/{}/wheels_driver_node/wheels_cmd".format(args.vehicle)
    rospy.Subscriber(wheel_topic, WheelsCmdStamped,
                     lambda m: requested.append((time.monotonic(), float(m.vel_left), float(m.vel_right))),
                     queue_size=100)
    rospy.Subscriber(wheel_topic + "_executed", WheelsCmdStamped,
                     lambda m: executed.append((time.monotonic(), float(m.vel_left), float(m.vel_right))),
                     queue_size=100)
    rospy.Subscriber("/{}/left_wheel_encoder_node/tick".format(args.vehicle), WheelEncoderStamped,
                     lambda m: left_ticks.append((time.monotonic(), int(m.data))), queue_size=100)
    rospy.Subscriber("/{}/right_wheel_encoder_node/tick".format(args.vehicle), WheelEncoderStamped,
                     lambda m: right_ticks.append((time.monotonic(), int(m.data))), queue_size=100)
    def status_callback(message):
        try:
            decoded = json.loads(message.data)
            received_at = time.monotonic()
            statuses.append(decoded)
            status_samples.append((received_at, decoded))
        except (TypeError, ValueError):
            pass
    rospy.Subscriber("/{}/lane_follower/status".format(args.vehicle), String,
                     status_callback, queue_size=10)
    recorder = CameraEvidenceRecorder(args.record_dir, args.record_frame_rate)
    rospy.Subscriber("/{}/camera_node/image/compressed".format(args.vehicle), CompressedImage,
                     recorder.callback, queue_size=1, buff_size=2**24)

    lane = None
    lane_log = None
    watchdog = None
    motor_reader = None
    motor_log = None
    released_at = None
    stopped_at = None
    early_stop_reason = None
    failure_reason = None
    try:
        time.sleep(0.3)
        preflight_stop_sent_at = time.monotonic()
        publish_stop(rospy, BoolStamped, WheelsCmdStamped, stop, wheels)
        next_preflight_stop = preflight_stop_sent_at + 0.1
        if not fixed_turn:
            environment = os.environ.copy()
            environment["VEHICLE_NAME"] = args.vehicle
            lane_log = open(args.lane_log, "w")
            lane_args = [
                sys.executable, args.node_path,
                "_drive_enabled:={}".format(
                    str(not red_line_inspection).lower()),
                "_show_debug:=false",
                "_camera_topic:=/{}/camera_node/image/compressed".format(args.vehicle),
                "_wheels_topic:=" + wheel_topic,
                "_base_speed:={}".format(args.base_speed),
                "_max_speed:={}".format(args.max_speed),
                "_max_steering:={}".format(args.max_steering),
                    "_min_active_wheel_speed:={}".format(args.min_active_wheel_speed),
                    "_taper_inner_wheel_floor:={}".format(
                        str(args.taper_inner_wheel_floor).lower()),
                "_lost_speed:=0.0", "_route_enabled:={}".format(
                    str(junction_mode).lower()),
                "_obstacle_enabled:=false", "_avoidance_enabled:=false",
                "_avoidance_speed:={}".format(args.base_speed),
                "_junction_speed:={}".format(args.base_speed),
                "_junction_bias:={}".format(min(0.025, args.base_speed)),
            ]
            if camera_guided or junction_mode or red_line_inspection:
                lane_args.extend([
                    "_k_p:={}".format(args.k_p),
                    "_near_center_k_p:={}".format(args.near_center_k_p),
                    "_full_gain_error:={}".format(args.full_gain_error),
                    "_lane_target_fraction:={}".format(args.lane_target_fraction),
                    "_alpha:={}".format(args.alpha),
                    "_deadband:={}".format(args.deadband),
                    "_steering_bias:={}".format(args.steering_bias),
                    "_smooth_steering_deadband:={}".format(
                        str(args.smooth_steering_deadband).lower()),
                    "_temporal_lane_width_fallback:={}".format(
                        str(args.temporal_lane_width_fallback).lower()),
                    "_temporal_lane_width_timeout:={}".format(
                        args.temporal_lane_width_timeout),
                    "_temporal_yellow_only_timeout:={}".format(
                        args.temporal_yellow_only_timeout),
                    "_boundary_risk_stop:={}".format(
                        str(args.boundary_risk_stop).lower()),
                    "_white_boundary_risk_fraction:={}".format(
                        args.white_boundary_risk_fraction),
                    "_yellow_lower:={}".format(args.yellow_lower),
                    "_yellow_upper:={}".format(args.yellow_upper),
                    "_white_lower:={}".format(args.white_lower),
                    "_white_upper:={}".format(args.white_upper),
                    "_red_stop_trigger_bottom_fraction:={}".format(
                        args.red_stop_trigger_bottom_fraction),
                    "_sharp_corner_enabled:={}".format(
                        str(args.sharp_corner_enabled).lower()),
                    "_sharp_corner_confirm_seconds:={}".format(
                        args.sharp_corner_confirm_seconds),
                    "_sharp_corner_recent_lane_seconds:={}".format(
                        args.sharp_corner_recent_lane_seconds),
                    "_sharp_corner_approach_seconds:={}".format(
                        args.sharp_corner_approach_seconds),
                    "_sharp_corner_turn_speed:={}".format(
                        args.sharp_corner_turn_speed),
                    "_sharp_corner_min_turn_seconds:={}".format(
                        args.sharp_corner_min_turn_seconds),
                    "_sharp_corner_white_confirm_seconds:={}".format(
                        args.sharp_corner_white_confirm_seconds),
                    "_sharp_corner_pivot_seconds:={}".format(
                        args.sharp_corner_pivot_seconds),
                    "_sharp_corner_relief_seconds:={}".format(
                        args.sharp_corner_relief_seconds),
                    "_sharp_corner_relief_inner_speed:={}".format(
                        args.sharp_corner_relief_inner_speed),
                    "_sharp_corner_max_turn_seconds:={}".format(
                        args.sharp_corner_max_turn_seconds),
                    "_sharp_corner_reacquire_seconds:={}".format(
                        args.sharp_corner_reacquire_seconds),
                    "_sharp_corner_trigger_error:={}".format(
                        args.sharp_corner_trigger_error),
                    "_sharp_corner_exit_error:={}".format(
                        args.sharp_corner_exit_error),
                ])
            if junction_mode:
                lane_args.extend([
                    "_junctions_calibrated:=true",
                    "_auto_continue:=true",
                    "_stop_hold_seconds:={}".format(args.stop_hold_seconds),
                    "_junction_entry_seconds:={}".format(args.junction_entry_seconds),
                    "_junction_straight_seconds:={}".format(args.junction_straight_seconds),
                    "_junction_left_seconds:={}".format(args.junction_left_seconds),
                    "_junction_right_seconds:={}".format(args.junction_right_seconds),
                    "_junction_reacquire_timeout:={}".format(
                        args.junction_reacquire_timeout),
                    "_junction_straight_speed:={}".format(args.junction_straight_speed),
                    "_junction_left_speed:={}".format(args.junction_left_speed),
                    "_junction_right_speed:={}".format(args.junction_right_speed),
                    "_junction_left_bias:={}".format(args.junction_left_bias),
                    "_junction_right_bias:={}".format(args.junction_right_bias),
                ])
            lane = subprocess.Popen(lane_args, env=environment, stdout=lane_log,
                                    stderr=subprocess.STDOUT)

        if junction_mode:
            configure_supervised_junction(
                rospy, String, command_publisher, statuses, args.junction_turn,
                lambda: publish_stop(
                    rospy, BoolStamped, WheelsCmdStamped, stop, wheels,
                    repeats=1))

        deadline = time.monotonic() + 10.0
        topic_publishers = []
        release_status = None
        last_scene_issue = None
        while time.monotonic() < deadline:
            now = time.monotonic()
            due, next_preflight_stop = fixed_command_due(
                now, next_preflight_stop
            )
            if due:
                publish_stop(rospy, BoolStamped, WheelsCmdStamped, stop, wheels)
            state = rosgraph.Master(rospy.get_name()).getSystemState()
            topic_publishers = dict(state[0]).get(wheel_topic, [])
            camera_ready = recorder.is_fresh(rospy) if fixed_turn else (
                statuses and statuses[-1].get("camera_valid"))
            lane_ready = fixed_turn or (
                bool(statuses and requested) if red_line_inspection else
                any(abs(left) > 1e-7 or abs(right) > 1e-7
                    for _, left, right in requested))
            if camera_guided or junction_mode or red_line_inspection:
                # An old node silently ignores unknown ROS parameters. Do not
                # release the stop unless the updated node confirms this mode.
                try:
                    if junction_mode:
                        require_junction_mode(
                            statuses[-1], args.junction_turn,
                            args.red_stop_trigger_bottom_fraction)
                    elif red_line_inspection:
                        require_camera_guided_mode(
                            statuses[-1], sharp_corner_expected=False)
                    else:
                        require_camera_guided_mode(statuses[-1])
                    last_scene_issue = (
                        None if red_line_inspection
                        else camera_guided_scene_issue(statuses[-1]))
                    guided_mode_ready = (
                        statuses[-1].get("camera_valid") is True
                        if red_line_inspection else last_scene_issue is None)
                except (RuntimeError, IndexError):
                    guided_mode_ready = False
                lane_ready = lane_ready and guided_mode_ready
            expected = {rospy.get_name()} if fixed_turn else {
                rospy.get_name(), "/lane_follower_node"}
            zero_ready = preflight_zero_feedback_ready(
                executed, preflight_stop_sent_at
            )
            if (set(topic_publishers) == expected and camera_ready and lane_ready
                    and zero_ready):
                release_status = statuses[-1] if not fixed_turn else None
                break
            if lane is not None and lane.poll() is not None:
                raise RuntimeError("Lane follower exited during preflight")
            time.sleep(0.05)
        if set(topic_publishers) != expected:
            raise RuntimeError("Wheel topic is not exclusive: {}".format(topic_publishers))
        if fixed_turn:
            if not recorder.is_fresh(rospy):
                raise RuntimeError("No current camera frame before fixed-turn release")
        else:
            if release_status is None:
                if (camera_guided or junction_mode) and last_scene_issue:
                    raise RuntimeError("Camera-guided scene is not suitable: {}".format(
                        last_scene_issue))
                raise RuntimeError("No valid camera status before release")
            # Check the latest sample, not only the snapshot which ended the
            # wait: a subsequent callback may have lost the lane or latched red.
            release_status = statuses[-1]
            if (not release_status.get("camera_valid")
                    or (not red_line_inspection and (
                        release_status.get("red_stop")
                        or release_status.get("lane_error") is None))):
                raise RuntimeError("Current scene is unsafe before release: {}".format(
                    release_status.get("lane_diagnostic", "invalid camera or red stop")))
            if camera_guided or junction_mode or red_line_inspection:
                if junction_mode:
                    require_junction_mode(
                        release_status, args.junction_turn,
                        args.red_stop_trigger_bottom_fraction)
                elif red_line_inspection:
                    require_camera_guided_mode(
                        release_status, sharp_corner_expected=False)
                else:
                    require_camera_guided_mode(release_status)
                issue = (None if red_line_inspection
                         else camera_guided_scene_issue(release_status))
                if issue is not None:
                    raise RuntimeError("Camera-guided scene is not suitable: {}".format(issue))
        if not preflight_zero_feedback_ready(executed, preflight_stop_sent_at):
            raise RuntimeError("No fresh executed-wheel feedback during stopped preflight")
        if any(abs(left) > 1e-7 or abs(right) > 1e-7
               for _, left, right in executed[-8:]):
            raise RuntimeError("Emergency stop did not hold zero during preflight")
        now = time.monotonic()
        if (not left_ticks or not right_ticks
                or now - left_ticks[-1][0] > ENCODER_FEEDBACK_TIMEOUT
                or now - right_ticks[-1][0] > ENCODER_FEEDBACK_TIMEOUT):
            raise RuntimeError("Fresh left and right encoder feedback is required before release")

        if red_line_inspection:
            sample_deadline = time.monotonic() + min(args.duration, 3.0)
            while time.monotonic() < sample_deadline:
                publish_stop(
                    rospy, BoolStamped, WheelsCmdStamped, stop, wheels,
                    repeats=1)
                if lane.poll() is not None:
                    raise RuntimeError("Lane follower exited during red-line inspection")
                time.sleep(.05)
            stopped_at = time.monotonic()
            publish_stop(rospy, BoolStamped, WheelsCmdStamped, stop, wheels)
            zero_confirmed = wait_for_post_stop_zero(
                executed, stopped_at, timeout_s=1.0)
            frames = recorder.freeze()
            latest = statuses[-1] if statuses else {}
            summary = {
                "mode": "red_line_inspection",
                "motion_started": False,
                "requested_duration_s": args.duration,
                "evidence_frames": frames,
                "red_line_visible": latest.get("red_line_visible"),
                "red_line_detection": latest.get("red_line_detection"),
                "red_line_samples": summarize_red_line_samples(status_samples),
                "red_stop": latest.get("red_stop"),
                "post_stop_zero_confirmed": zero_confirmed,
                "final_feedback_all_zero": zero_confirmed,
                "early_stop_reason": None,
            }
            write_evidence_telemetry(
                recorder.output_dir, summary["mode"], None, stopped_at,
                requested, executed, left_ticks, right_ticks,
                status_samples=status_samples)
            write_evidence_completion(recorder.output_dir, summary)
            print(json.dumps(summary, sort_keys=True), flush=True)
            print("SUPERVISOR_COMPLETE", flush=True)
            return 0

        if args.preflight_only:
            print("PREFLIGHT_COMPLETE", flush=True)
            return 0

        watchdog_args = [
            sys.executable, os.path.abspath(__file__), "--watchdog",
            "--vehicle", args.vehicle,
            "--watchdog-delay", str(args.duration + WATCHDOG_MARGIN),
        ]
        watchdog = subprocess.Popen(
            watchdog_args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, universal_newlines=True, bufsize=1,
            start_new_session=True,
        )
        wait_for_watchdog(watchdog)
        motor_log = (recorder.output_dir / 'motor_registers.jsonl').open('x')
        motor_reader = subprocess.Popen(
            [sys.executable, os.path.abspath(__file__), '--read-motor-registers'],
            stdin=subprocess.PIPE, stdout=motor_log, stderr=motor_log,
            start_new_session=True)
        watchdog.stdin.write("GO\n")
        watchdog.stdin.flush()
        active_expected = set(expected)
        active_expected.add(WATCHDOG_NODE)

        released_at = time.monotonic()
        recorder.motion_started_at = released_at
        publish_estop(rospy, BoolStamped, stop, False)
        if fixed_turn:
            publish_wheel_command(rospy, WheelsCmdStamped, wheels,
                                  args.fixed_left, args.fixed_right)
        motion_deadline = released_at + args.duration
        next_fixed_publish = released_at + 0.1
        junction_completed = False
        junction_completion_started = None
        while time.monotonic() < motion_deadline:
            now = time.monotonic()
            if fixed_turn:
                due, next_fixed_publish = fixed_command_due(
                    now, next_fixed_publish
                )
                if due:
                    publish_wheel_command(
                        rospy, WheelsCmdStamped, wheels,
                        args.fixed_left, args.fixed_right
                    )
            state = rosgraph.Master(rospy.get_name()).getSystemState()
            topic_publishers = dict(state[0]).get(wheel_topic, [])
            early_stop_reason = motion_fault(
                now=now,
                camera_fresh=recorder.is_fresh(rospy),
                status_sample=status_samples[-1] if status_samples else None,
                lane_running=(None if fixed_turn else lane.poll() is None),
                topic_publishers=topic_publishers,
                expected_publishers=active_expected,
                executed_samples=executed,
                require_lane_status=not fixed_turn,
                left_encoder_samples=left_ticks,
                right_encoder_samples=right_ticks,
                released_at=released_at,
                junction_mode=junction_mode,
            )
            if early_stop_reason is not None:
                break
            if junction_mode and status_samples:
                if junction_completion_status(
                        status_samples[-1][1], args.junction_turn):
                    if junction_completion_started is None:
                        junction_completion_started = now
                    elif (now - junction_completion_started
                          >= JUNCTION_POST_REACQUIRE_SECONDS):
                        junction_completed = True
                        break
                else:
                    junction_completion_started = None
            time.sleep(min(0.05, max(0.0, motion_deadline - now)))
        if junction_mode and not junction_completed and early_stop_reason is None:
            early_stop_reason = "Junction did not reacquire the outgoing lane before the session deadline"
        stopped_at = time.monotonic()
        publish_stop(rospy, BoolStamped, WheelsCmdStamped, stop, wheels)
        print("MOTION_STOPPED", flush=True)

        if lane is not None:
            lane.send_signal(signal.SIGINT)
            lane.wait(timeout=8)
        time.sleep(0.35)
        publish_stop(rospy, BoolStamped, WheelsCmdStamped, stop, wheels)
        zero_confirmed = wait_for_post_stop_zero(
            executed, stopped_at, timeout_s=1.0
        )
        finish_reader(motor_reader, motor_log)
        motor_reader = motor_log = None
        evidence_frames = recorder.freeze()
        summary = summarize_samples(executed, released_at, stopped_at)
        summary.update(summarize_status_samples(
            status_samples, released_at, stopped_at
        ))
        summary.update({
            "mode": ("fixed_turn" if fixed_turn else
                     "junction_{}".format(args.junction_turn) if junction_mode else
                     "camera_guided_curve" if camera_guided else "lane_following"),
            "requested_duration_s": args.duration,
            "measured_stop_deadline_s": stopped_at - released_at,
            "left_encoder_delta": encoder_delta_in_window(left_ticks, released_at, stopped_at),
            "right_encoder_delta": encoder_delta_in_window(right_ticks, released_at, stopped_at),
            "camera_valid": recorder.is_fresh(rospy),
            "lane_diagnostic": statuses[-1].get("lane_diagnostic") if (statuses and not fixed_turn) else None,
            "evidence_dir": str(recorder.output_dir),
            "evidence_frames": evidence_frames,
            # This is intentionally distinct from post_stop_all_zero: an
            # in-flight pre-stop command can be reported once after the stop
            # request, while a later fresh eight-sample zero window proves
            # the driver reached a stopped state.
            "final_feedback_all_zero": zero_confirmed,
            "post_stop_zero_confirmed": zero_confirmed,
            "early_stop_reason": early_stop_reason,
            "junction_completed": junction_completed if junction_mode else None,
        })
        if fixed_turn:
            summary["fixed_left"] = args.fixed_left
            summary["fixed_right"] = args.fixed_right
            summary['wheel_response'] = wheel_response(
                summary['left_encoder_delta'], summary['right_encoder_delta'],
                args.fixed_left, args.fixed_right)
        telemetry = write_evidence_telemetry(
            recorder.output_dir, summary["mode"], released_at, stopped_at,
            requested, executed, left_ticks, right_ticks,
            status_samples=status_samples, early_stop_reason=early_stop_reason,
        )
        summary["telemetry_file"] = str(telemetry)
        completion = write_evidence_completion(recorder.output_dir, summary)
        summary["completion_file"] = str(completion)
        print(json.dumps(summary, sort_keys=True), flush=True)
        if not summary["final_feedback_all_zero"]:
            raise RuntimeError("Executed-wheel feedback was not zero after stop")
        if early_stop_reason is not None:
            raise RuntimeError("Motion stopped early: {}".format(early_stop_reason))
        print("SUPERVISOR_COMPLETE", flush=True)
        return 0
    except BaseException as error:
        failure_reason = "{}: {}".format(type(error).__name__, error)
        raise
    finally:
        publish_stop(rospy, BoolStamped, WheelsCmdStamped, stop, wheels)
        finish_reader(motor_reader, motor_log)
        if lane is not None and lane.poll() is None:
            lane.send_signal(signal.SIGINT)
            try:
                lane.wait(timeout=5)
            except subprocess.TimeoutExpired:
                lane.kill()
                lane.wait(timeout=3)
        if watchdog is not None:
            if watchdog.stdin:
                try:
                    watchdog.stdin.close()
                except OSError:
                    pass
            try:
                watchdog.wait(timeout=5)
            except subprocess.TimeoutExpired:
                watchdog.kill()
                watchdog.wait(timeout=3)
        if lane_log is not None:
            lane_log.close()
        try:
            if failure_reason and not (recorder.output_dir / "completion.json").exists():
                mode = ("fixed_turn" if fixed_turn else
                        "red_line_inspection" if red_line_inspection else
                        "junction_{}".format(args.junction_turn) if junction_mode else
                        "camera_guided_curve" if camera_guided else "lane_following")
                summary = record_aborted_run(
                    recorder, mode, released_at, requested, executed,
                    left_ticks, right_ticks, status_samples, failure_reason,
                    lambda: publish_stop(rospy, BoolStamped, WheelsCmdStamped, stop, wheels))
                print(json.dumps(summary, sort_keys=True), flush=True)
        finally:
            recorder.close()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", default="duck2")
    parser.add_argument("--node-path")
    parser.add_argument("--lane-log", default="/tmp/duck2-ground-lane.log")
    parser.add_argument("--duration", type=float, default=0.8)
    parser.add_argument("--base-speed", type=float, default=0.04)
    parser.add_argument("--max-speed", type=float, default=0.05)
    parser.add_argument("--max-steering", type=float, default=0.01)
    parser.add_argument("--camera-guided-curve", action="store_true",
                        help="Use the reviewed bounded camera-guided preset")
    parser.add_argument("--junction-turn", choices=("straight", "left", "right"),
                        help="Run one route-backed supervised junction action")
    parser.add_argument("--inspect-red-line", action="store_true",
                        help="Record stationary red-line geometry; never release stop")
    parser.add_argument("--red-stop-trigger-bottom-fraction", type=float,
                        default=0.65,
                        help="Provisional image-height threshold; calibrate from stationary samples")
    parser.add_argument("--fixed-left", type=float,
                        help="Left command from an approved diagnostic pair")
    parser.add_argument("--fixed-right", type=float,
                        help="Right command from an approved diagnostic pair")
    parser.add_argument("--record-dir", default="/tmp/duck2-ground-evidence",
                        help="Temporary robot-side directory for frames and telemetry")
    parser.add_argument("--record-frame-rate", type=float, default=4.0,
                        help="Evidence capture rate, from 1 to 10 Hz")
    parser.add_argument("--acknowledge-upright-clear", action="store_true")
    parser.add_argument("--preflight-only", action="store_true",
                        help="Validate a stopped setup without releasing emergency stop")
    parser.add_argument("--watchdog", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--read-motor-registers", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--watchdog-delay", type=float, default=1.05,
                        help=argparse.SUPPRESS)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.read_motor_registers:
        return motor_register_reader()
    if args.watchdog:
        if not 0 < args.watchdog_delay <= MAX_CAMERA_GUIDED_DURATION + WATCHDOG_MARGIN:
            raise ValueError("Invalid watchdog delay")
        return watchdog_main(args)
    return supervisor_main(args)


if __name__ == "__main__":
    sys.exit(main())
