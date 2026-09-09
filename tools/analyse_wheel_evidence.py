"""Offline comparison of encoder progress under recorded wheel commands."""
import argparse
import json
from pathlib import Path
from bounded_ground_supervisor import encoder_motion_fault


def compare(path):
    path = Path(path)
    if path.is_dir():
        nested = path / 'downloaded-evidence' / 'telemetry.json'
        path = nested if nested.exists() else path / 'telemetry.json'
    data = json.loads(path.read_text())
    start = data['motion_window_monotonic_s']['released']
    stop = data['motion_window_monotonic_s']['stopped']
    if start is None:
        return {'run': str(path.parent), 'motion_started': False}
    def value_at(key, t):
        values = [s[1] for s in data[key] if s[0] <= t]
        return values[-1] if values else None
    rows = []
    a = start
    while a < stop:
        b = min(a + .5, stop)
        deltas = []
        for key in ('left_encoder_ticks', 'right_encoder_ticks'):
            first, last = value_at(key, a), value_at(key, b)
            deltas.append(None if first is None or last is None else last-first)
        commands = [s[1:] for s in data['executed_wheels'] if a <= s[0] < b]
        rows.append({'offset_s': round(a-start, 2), 'ticks_left_right': deltas,
                     'last_echo': commands[-1] if commands else None})
        a = b
    stall = None
    for now, _ in data['left_encoder_ticks']:
        if not start <= now <= stop:
            continue
        reason = encoder_motion_fault(now, data['executed_wheels'],
                                      data['left_encoder_ticks'],
                                      data['right_encoder_ticks'], start)
        if reason:
            stall = {'offset_s': round(now-start, 3), 'reason': reason}
            break
    return {'run': str(path.parent), 'duration_s': round(stop-start, 3),
            'replayed_stall': stall, 'half_second_windows': rows}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('paths', nargs='+')
    parser.add_argument('--summary', action='store_true')
    args = parser.parse_args()
    results = [compare(p) for p in args.paths]
    if args.summary:
        for result in results:
            result.pop('half_second_windows', None)
    print(json.dumps(results, indent=2))
