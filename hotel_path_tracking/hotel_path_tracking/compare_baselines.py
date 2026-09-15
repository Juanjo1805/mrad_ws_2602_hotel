#!/usr/bin/env python3
"""Numerically compare a passive golden run with an automatic baseline."""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import math
from pathlib import Path
from typing import Any

import yaml


def _load(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    with path.open(encoding='utf-8') as stream:
        value = yaml.safe_load(stream) or {}
    return value if isinstance(value, dict) else {}


def _number(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _read_csv(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        return []
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def _path_rows(run: Path) -> list[tuple[float, float, float, float]]:
    rows = _read_csv(run / 'path_profile.csv')
    output = []
    for row in rows:
        values = tuple(_number(row.get(key)) for key in ('s_m', 'x_m', 'y_m', 'heading_rad'))
        if all(value is not None for value in values):
            output.append(values)  # type: ignore[arg-type]
    return output


def _path_comparison(left: Path, right: Path) -> dict[str, Any]:
    a, b = _path_rows(left), _path_rows(right)
    result: dict[str, Any] = {'left_points': len(a), 'right_points': len(b)}
    if not a or not b:
        result['status'] = 'missing_path_profile'
        return result
    digest_a, digest_b = hashlib.sha256(), hashlib.sha256()
    for digest, values in ((digest_a, a), (digest_b, b)):
        for value in values:
            digest.update((','.join(f'{item:.12f}' for item in value) + '\n').encode())
    result['left_profile_sha256'] = digest_a.hexdigest()
    result['right_profile_sha256'] = digest_b.hexdigest()
    def interpolate(rows: list[tuple[float, float, float, float]], s: float) -> tuple[float, float, float]:
        distances = [row[0] for row in rows]
        index = bisect.bisect_left(distances, s)
        if index <= 0:
            _, x, y, heading = rows[0]
            return x, y, heading
        if index >= len(rows):
            _, x, y, heading = rows[-1]
            return x, y, heading
        s0, x0, y0, h0 = rows[index - 1]
        s1, x1, y1, h1 = rows[index]
        ratio = 0.0 if s1 <= s0 else (s - s0) / (s1 - s0)
        heading = h0 + ratio * math.atan2(math.sin(h1 - h0), math.cos(h1 - h0))
        return x0 + ratio * (x1 - x0), y0 + ratio * (y1 - y0), heading

    # Path planners may yield a different number of samples while representing
    # almost the same curve.  Compare both paths on the same arc-length grid;
    # point counts and byte hashes remain separately recorded above.
    common_length = min(a[-1][0], b[-1][0])
    sample_count = max(2, int(common_length / 0.05) + 1)
    samples = [common_length * index / (sample_count - 1) for index in range(sample_count)]
    paired = [(interpolate(a, s), interpolate(b, s)) for s in samples]
    distances = [math.hypot(x0 - x1, y0 - y1)
                 for ((x0, y0, _), (x1, y1, _)) in paired]
    headings = [abs(math.atan2(math.sin(h0 - h1), math.cos(h0 - h1)))
                for ((_, _, h0), (_, _, h1)) in paired]
    result.update({
        'common_arc_length_m': common_length,
        'arc_length_samples': sample_count,
        'xy_rms_m': math.sqrt(sum(value * value for value in distances) / len(distances)),
        'xy_max_m': max(distances),
        'heading_rms_rad': math.sqrt(sum(value * value for value in headings) / len(headings)),
        'heading_max_rad': max(headings),
        'status': (
            'identical_within_numeric_tolerance'
            if len(a) == len(b) and max(distances) <= 1e-9 and max(headings) <= 1e-9
            else 'equivalent_within_1mm_1mrad'
            if max(distances) <= 1e-3 and max(headings) <= 1e-3
            else 'geometrically_different'
        ),
        'left_first_20': [{'s_m': row[0], 'x_m': row[1], 'y_m': row[2], 'heading_rad': row[3]}
                          for row in a[:20]],
        'right_first_20': [{'s_m': row[0], 'x_m': row[1], 'y_m': row[2], 'heading_rad': row[3]}
                           for row in b[:20]],
    })
    return result


def _startup(run: Path) -> dict[str, Any]:
    state = _load(run / 'initial_state.yaml')
    localized = _load(run / 'readiness_localized.yaml')
    ground_truth = _load(run / 'gazebo_ground_truth.yaml')
    if localized:
        state.setdefault('readiness_localized', localized)
        state.setdefault('pre_navigation_snapshot', localized.get('snapshot'))
    if ground_truth:
        state['gazebo_ground_truth_runner_query'] = ground_truth
    return state or {'status': 'not_captured_for_this_legacy_run'}


def _conflict(run: Path) -> dict[str, Any]:
    rows = _read_csv(run / 'monitor_trace.csv')
    if not rows:
        return {'status': 'missing_monitor_trace'}
    distances = [(_number(row.get('aeb_critical_distance_m')), row) for row in rows]
    valid = [(value, row) for value, row in distances if value is not None]
    if not valid:
        return {'status': 'no_front_distance_recorded'}
    _, selected = min(valid, key=lambda item: item[0])
    fields = (
        'time_s', 'x_m', 'y_m', 'yaw_rad', 'path_s_m', 'progress_percent',
        'closest_index', 'target_index', 'lookahead_x_m', 'lookahead_y_m',
        'curvature_1_m', 'future_curvature_1_m', 'lateral_error_m', 'heading_error_rad',
        'target_v_m_s', 'target_omega_rad_s', 'mux_v_m_s', 'mux_omega_rad_s',
        'output_v_m_s', 'output_omega_rad_s', 'actual_v_m_s', 'actual_omega_rad_s',
        'aeb_active', 'aeb_command_blocked', 'aeb_critical_distance_m', 'aeb_ttc_s',
    )
    return {'status': 'minimum_front_distance', **{field: selected.get(field) for field in fields}}


def _at_path_s(run: Path, requested_path_s: Any) -> dict[str, Any]:
    """Return the trace sample nearest the reference run's path coordinate."""
    requested = _number(requested_path_s)
    rows = _read_csv(run / 'monitor_trace.csv')
    if requested is None or not rows:
        return {'status': 'missing_path_coordinate_or_trace'}
    candidates = [(_number(row.get('path_s_m')), row) for row in rows]
    candidates = [(value, row) for value, row in candidates if value is not None]
    if not candidates:
        return {'status': 'no_path_coordinate_recorded'}
    selected_s, selected = min(candidates, key=lambda item: abs(item[0] - requested))
    fields = (
        'time_s', 'x_m', 'y_m', 'yaw_rad', 'path_s_m', 'progress_percent',
        'closest_index', 'target_index', 'lookahead_x_m', 'lookahead_y_m',
        'curvature_1_m', 'future_curvature_1_m', 'lateral_error_m', 'heading_error_rad',
        'target_v_m_s', 'target_omega_rad_s', 'mux_v_m_s', 'mux_omega_rad_s',
        'output_v_m_s', 'output_omega_rad_s', 'actual_v_m_s', 'actual_omega_rad_s',
        'aeb_active', 'aeb_command_blocked', 'aeb_critical_distance_m', 'aeb_ttc_s',
    )
    return {
        'status': 'nearest_reference_path_coordinate',
        'reference_path_s_m': requested,
        'path_s_difference_m': selected_s - requested,
        **{field: selected.get(field) for field in fields},
    }


def _summary(run: Path) -> dict[str, Any]:
    results = _load(run / 'results.yaml')
    return results.get('result', {}) if results else {}


def _find_manual(root: Path) -> Path | None:
    candidates = sorted(root.glob('run_*_golden_manual_baseline'))
    return candidates[-1] if candidates else None


def _resolve_run(root: Path, value: Path) -> Path:
    """Accept both a run name and a path already rooted at results_root."""
    if value.is_absolute() or value.is_dir():
        return value.resolve()
    return (root / value).resolve()


def _fmt(value: Any, digits: int = 4) -> str:
    number = _number(value)
    return '—' if number is None else f'{number:.{digits}f}'


def _report(manual: Path, automatic: Path, output: Path) -> dict[str, Any]:
    manual_metadata, automatic_metadata = _load(manual / 'metadata.yaml'), _load(automatic / 'metadata.yaml')
    manual_label = ('Manual' if manual_metadata.get('capture_mode') == 'passive_manual_observation'
                    else 'Automatic / left')
    automatic_label = ('Manual' if automatic_metadata.get('capture_mode') == 'passive_manual_observation'
                       else 'Automatic / right')
    automatic_conflict = _conflict(automatic)
    comparison = {
        'manual_run': str(manual),
        'automatic_run': str(automatic),
        'manual_metadata': manual_metadata,
        'automatic_metadata': automatic_metadata,
        'manual_startup': _startup(manual),
        'automatic_startup': _startup(automatic),
        'path_comparison': _path_comparison(manual, automatic),
        # Use the right run's critical path coordinate as the reference.  This
        # avoids treating two unrelated global LiDAR minima as the same place.
        'manual_conflict_section': _at_path_s(manual, automatic_conflict.get('path_s_m')),
        'automatic_conflict_section': automatic_conflict,
        'manual_result': _summary(manual),
        'automatic_result': _summary(automatic),
    }
    with output.with_suffix('.yaml').open('w', encoding='utf-8') as stream:
        yaml.safe_dump(comparison, stream, sort_keys=False)
    path = comparison['path_comparison']
    manual_result, automatic_result = comparison['manual_result'], comparison['automatic_result']
    lines = [
        f'# {manual_label} baseline vs {automatic_label} baseline', '',
        f'- {manual_label}: `{manual.name}`', f'- {automatic_label}: `{automatic.name}`', '',
        '## Mission', '',
        f'| Metric | {manual_label} | {automatic_label} |', '|---|---:|---:|',
        f'| Completed | {manual_result.get("completed", "—")} | {automatic_result.get("completed", "—")} |',
        f'| Laps | {manual_result.get("laps", "—")} | {automatic_result.get("laps", "—")} |',
        f'| Total time [s] | {_fmt(manual_result.get("total_time_s"))} | {_fmt(automatic_result.get("total_time_s"))} |',
        f'| Lateral RMSE [m] | {_fmt(manual_result.get("lateral_rmse_m"))} | {_fmt(automatic_result.get("lateral_rmse_m"))} |',
        f'| AEB time [s] | {_fmt(manual_result.get("aeb_active_time_s"))} | {_fmt(automatic_result.get("aeb_active_time_s"))} |', '',
        '## Planned path', '',
        f'- Status: `{path.get("status", "—")}`',
        f'- Poses: left `{path.get("left_points", "—")}`, right `{path.get("right_points", "—")}`.',
        f'- Common arc length / samples: `{_fmt(path.get("common_arc_length_m"), 4)}` m / `{path.get("arc_length_samples", "—")}`.',
        f'- XY RMS / max on that common arc: `{_fmt(path.get("xy_rms_m"), 8)}` / `{_fmt(path.get("xy_max_m"), 8)}` m.',
        f'- Heading RMS / max: `{_fmt(path.get("heading_rms_rad"), 8)}` / `{_fmt(path.get("heading_max_rad"), 8)}` rad.', '',
        f'## Critical section of {automatic_label}', '',
        f'The left sample is matched to the path coordinate of {automatic_label}\'s minimum front clearance.', '',
        f'| Signal | {manual_label} | {automatic_label} |', '|---|---:|---:|',
    ]
    fields = ('x_m', 'y_m', 'yaw_rad', 'path_s_m', 'closest_index', 'target_index',
              'lateral_error_m', 'target_v_m_s', 'target_omega_rad_s', 'actual_v_m_s',
              'actual_omega_rad_s', 'aeb_critical_distance_m', 'aeb_active', 'aeb_command_blocked')
    for field in fields:
        left = comparison['manual_conflict_section'].get(field, '—')
        right = comparison['automatic_conflict_section'].get(field, '—')
        lines.append(f'| {field} | {left} | {right} |')
    lines.extend([
        '', '## Startup evidence', '',
        'Full startup snapshots, covariance, TF samples and readiness timings are in the adjacent YAML comparison. '
        'An older run without these files is explicitly reported as legacy-not-captured rather than inferred.', '',
    ])
    output.write_text('\n'.join(lines), encoding='utf-8')
    return comparison


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-root', type=Path, default=Path('optimization_results'))
    parser.add_argument('--manual', type=Path)
    parser.add_argument('--automatic', type=Path, default=Path('run_006_baseline'))
    parser.add_argument('--output', type=Path)
    parsed = parser.parse_args(args)
    root = parsed.results_root.resolve()
    manual = _resolve_run(root, parsed.manual) if parsed.manual else _find_manual(root)
    automatic = _resolve_run(root, parsed.automatic)
    if manual is None or not manual.is_dir():
        raise SystemExit('No golden_manual_baseline directory found; capture it before comparing.')
    if not automatic.is_dir():
        raise SystemExit(f'Automatic run does not exist: {automatic}')
    output = parsed.output or root / f'{manual.name}_vs_{automatic.name}.md'
    if not output.is_absolute():
        output = root / output
    _report(manual, automatic, output)
    print(f'Wrote {output} and {output.with_suffix(".yaml")}')


if __name__ == '__main__':
    main()
