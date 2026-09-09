"""Replay saved JPEGs with mocked ROS; never connects or predicts a physical path.

Run with the repository image and --network none. Evidence stays outside Git.
The recorder subsamples images, so this is not an exact callback-timing replay.
"""
import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

from test_lane_follower import LaneTests
from test_bounded_ground_supervisor import MODULE as supervisor


def replay(directory, smooth, bias, temporal_lane_width_fallback=False):
    fixture = LaneTests()
    fixture.setUp()
    node = fixture.node
    args = supervisor.parse_args(['--camera-guided-curve'])
    supervisor.apply_camera_guided_preset(args)
    for name in ('base_speed', 'max_speed', 'max_steering',
                 'min_active_wheel_speed', 'taper_inner_wheel_floor', 'k_p',
                 'near_center_k_p', 'full_gain_error',
                 'lane_target_fraction', 'alpha', 'deadband',
                 'sharp_corner_enabled', 'sharp_corner_confirm_seconds',
                 'sharp_corner_recent_lane_seconds',
                 'sharp_corner_approach_seconds', 'sharp_corner_turn_speed',
                 'sharp_corner_min_turn_seconds',
                 'sharp_corner_white_confirm_seconds',
                 'sharp_corner_pivot_seconds', 'sharp_corner_relief_seconds',
                 'sharp_corner_relief_inner_speed',
                 'sharp_corner_max_turn_seconds', 'sharp_corner_reacquire_seconds',
                 'sharp_corner_trigger_error', 'sharp_corner_exit_error'):
        setattr(node, name, getattr(args, name))
    node.drive_enabled = True  # Only the test double's in-memory publisher exists.
    node.obstacle_enabled = False
    node.smooth_steering_deadband = smooth
    node.steering_bias = bias
    node.temporal_lane_width_fallback = temporal_lane_width_fallback
    node.temporal_lane_width_timeout = args.temporal_lane_width_timeout
    node.temporal_yellow_only_timeout = args.temporal_yellow_only_timeout
    node.boundary_risk_stop = (args.boundary_risk_stop
                               and temporal_lane_width_fallback)
    node.white_boundary_risk_fraction = args.white_boundary_risk_fraction
    node.yellow_lower = np.array(args.yellow_lower, dtype=np.uint8)
    node.yellow_upper = np.array(args.yellow_upper, dtype=np.uint8)
    node.white_lower = np.array(args.white_lower, dtype=np.uint8)
    node.white_upper = np.array(args.white_upper, dtype=np.uint8)
    telemetry = json.loads((directory / 'telemetry.json').read_text())
    window = telemetry['motion_window_monotonic_s']
    records = [json.loads(line) for line in (directory / 'frames.jsonl').read_text().splitlines()]
    if not records:
        raise ValueError('Recording has no frames')
    fixture.now = records[0]['received_monotonic_s']
    node.reset_steering()
    rows = []
    for record in records:
        filename = record['file']
        if Path(filename).name != filename:
            raise ValueError('Expected a frame basename')
        fixture.now = record['received_monotonic_s']
        frame = cv2.imread(str(directory / filename))
        error, _, _ = node.detect_lane_bgr(frame)
        corner = node.sharp_corner_wheels(error, False)
        if corner is None:
            left, right, steering = node.compute_wheel_speeds(error)
        else:
            left, right, steering = corner
        assert all(math.isfinite(x) and 0 <= x <= args.max_speed for x in (left, right))
        assert abs(steering) <= args.max_steering
        if window['released'] <= fixture.now <= window['stopped']:
            rows.append(dict(file=filename, error=error, filtered=node.filtered_error,
                             received_monotonic_s=fixture.now,
                             left=left, right=right, steering=steering,
                             lane_limits=(list(node._lane_limits)
                                          if node._lane_limits is not None else None),
                             diagnostic=node._lane_diagnostic,
                             sharp_corner_state=node._sharp_corner_state,
                             sharp_corner_phase=node._sharp_corner_phase))
    if not rows:
        raise ValueError('Recording has no motion-window frames')
    signs = [1 if row['steering'] > 0 else -1 for row in rows
             if abs(row['steering']) > .003]
    transitions = []
    prior_state = None
    for row in rows:
        if row['sharp_corner_state'] != prior_state:
            transitions.append({
                'file': row['file'],
                'seconds_from_motion_start': (
                    row['received_monotonic_s'] - window['released']),
                'state': row['sharp_corner_state'],
                'error': row['error'],
                'left': row['left'],
                'right': row['right'],
            })
            prior_state = row['sharp_corner_state']
    summary = dict(recording=directory.name, smooth=smooth, bias=bias,
                   temporal_lane_width_fallback=temporal_lane_width_fallback,
                   exact_reviewed_yellow_range=True,
                   k_p=node.k_p, near_center_k_p=node.near_center_k_p,
                   full_gain_error=node.full_gain_error,
                   min_active_wheel_speed=node.min_active_wheel_speed,
                   taper_inner_wheel_floor=node.taper_inner_wheel_floor,
                   temporal_lane_width_timeout=node.temporal_lane_width_timeout,
                   temporal_yellow_only_timeout=node.temporal_yellow_only_timeout,
                   boundary_risk_stop=node.boundary_risk_stop,
                   sharp_corner_enabled=node.sharp_corner_enabled,
                   sharp_corner_states=sorted(set(
                       row['sharp_corner_state'] for row in rows)),
                   sharp_corner_phases=sorted(set(
                       row['sharp_corner_phase'] for row in rows)),
                   sharp_corner_transitions=transitions,
                   frames=len(rows),
                   missing_lane=sum(r['error'] is None for r in rows),
                   first=rows[0], last=rows[-1],
                   direction_changes=sum(a != b for a, b in zip(signs, signs[1:])),
                   max_steering_step=max((abs(a['steering'] - b['steering'])
                                         for a, b in zip(rows, rows[1:])), default=0))
    # These are perception/control calculations, not controller acceptance or
    # wheel delivery. Safety gates are exercised separately by regression tests.
    return dict(summary=summary, frames=rows)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recordings', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path,
                        help='JSON report outside the repository')
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    if output == repo or repo in output.parents:
        parser.error('Store replay evidence outside the repository')
    reports = []
    for directory in args.recordings:
        profiles = ((False, .015, False), (True, .015, False),
                    (True, .0075, False), (True, 0., False),
                    (True, .0075, True))
        for smooth, bias, temporal in profiles:
            result = replay(directory, smooth, bias, temporal)
            reports.append(result)
            print(json.dumps(result['summary'], allow_nan=False))
    output.write_text(json.dumps(reports, indent=2, allow_nan=False) + '\n')
