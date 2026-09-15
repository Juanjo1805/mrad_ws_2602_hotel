#!/usr/bin/env python3
"""Compare manual and corrected automatic baselines without changing ROS state."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import statistics
from typing import Any

import yaml

from .compare_baselines import _path_comparison


DEFAULT_AUTOMATIC = [
    'run_010_baseline_reproduced_manual',
    'run_011_baseline_spawn_corrected_repeat_1',
]
DEFAULT_FAILED_REFERENCE = 'run_006_baseline'
METRICS = (
    'total_time_s', 'lap_1_time_s', 'lap_2_time_s', 'lateral_rmse_m',
    'lateral_max_error_m', 'aeb_active_time_s', 'aeb_events',
    'aeb_longest_intervention_s', 'minimum_front_distance_m', 'non_aeb_time_s',
)
TRACE_FIELDS = (
    'time_s', 'x_m', 'y_m', 'yaw_rad', 'path_s_m', 'progress_percent', 'closest_index',
    'target_index', 'lookahead_x_m', 'lookahead_y_m', 'curvature_1_m',
    'future_curvature_1_m', 'lateral_error_m', 'heading_error_rad', 'target_v_m_s',
    'target_omega_rad_s', 'mux_v_m_s', 'mux_omega_rad_s', 'output_v_m_s',
    'output_omega_rad_s', 'actual_v_m_s', 'actual_omega_rad_s', 'aeb_active',
    'aeb_command_blocked', 'aeb_critical_distance_m', 'aeb_ttc_s',
)


def _load(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    with path.open(encoding='utf-8') as stream:
        value = yaml.safe_load(stream) or {}
    return value if isinstance(value, dict) else {}


def _number(value: Any) -> float | None:
    try:
        output = float(value)
    except (TypeError, ValueError):
        return None
    return output if math.isfinite(output) else None


def _format(value: Any, digits: int = 4) -> str:
    number = _number(value)
    return '—' if number is None else f'{number:.{digits}f}'


def _read_trace(directory: Path) -> list[dict[str, str]]:
    path = directory / 'monitor_trace.csv'
    if not path.is_file():
        return []
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def _bag_interval(directory: Path) -> dict[str, float | None]:
    """Read absolute rosbag time so concurrent observers never inflate n."""
    metadata_files = sorted((directory / 'rosbag').glob('*/metadata.yaml'))
    if not metadata_files:
        return {'start_epoch_s': None, 'end_epoch_s': None, 'duration_s': None}
    info = _load(metadata_files[0]).get('rosbag2_bagfile_information', {})
    if not isinstance(info, dict):
        return {'start_epoch_s': None, 'end_epoch_s': None, 'duration_s': None}
    starting = info.get('starting_time', {})
    duration = info.get('duration', {})
    start_ns = _number(starting.get('nanoseconds_since_epoch') if isinstance(starting, dict) else None)
    duration_ns = _number(duration.get('nanoseconds') if isinstance(duration, dict) else None)
    if start_ns is None or duration_ns is None:
        return {'start_epoch_s': None, 'end_epoch_s': None, 'duration_s': None}
    start_s, duration_s = start_ns / 1e9, duration_ns / 1e9
    return {'start_epoch_s': start_s, 'end_epoch_s': start_s + duration_s, 'duration_s': duration_s}


def _run(directory: Path) -> dict[str, Any]:
    results_file = _load(directory / 'results.yaml')
    return {
        'name': directory.name,
        'directory': directory,
        'metadata': _load(directory / 'metadata.yaml'),
        'results_file': results_file,
        'result': results_file.get('result', {}),
        'path': results_file.get('path_profile', {}),
        'safety': results_file.get('safety', {}),
        'initial_state': _load(directory / 'initial_state.yaml'),
        'readiness': _load(directory / 'readiness_localized.yaml'),
        'ground_truth': _load(directory / 'gazebo_ground_truth.yaml'),
        'trace': _read_trace(directory),
        'bag_interval': _bag_interval(directory),
    }


def _startup(record: dict[str, Any]) -> dict[str, Any]:
    snapshot = record['readiness'].get('snapshot')
    if not isinstance(snapshot, dict):
        snapshot = record['initial_state']
    return {
        'physical_gazebo': {key: record['ground_truth'].get(key)
                            for key in ('status', 'x_m', 'y_m', 'z_m', 'yaw_rad')},
        'ekf': snapshot.get('ekf_odometry'),
        'amcl': snapshot.get('amcl_pose'),
        'initialpose': snapshot.get('initialpose_observed'),
        'tf': snapshot.get('transforms'),
        'readiness': {
            'ready': record['readiness'].get('ready'),
            'reason': record['readiness'].get('reason'),
            'elapsed_wall_s': record['readiness'].get('elapsed_wall_s'),
        },
    }


def _metric_row(record: dict[str, Any]) -> dict[str, Any]:
    result = record['result']
    lap_1, lap_2 = result.get('lap_1', {}), result.get('lap_2', {})
    return {
        'run': record['name'],
        'mode': record['metadata'].get('capture_mode', 'automatic_runner'),
        'completed': result.get('completed'), 'laps': result.get('laps'),
        'total_time_s': result.get('total_time_s'),
        'lap_1_time_s': lap_1.get('time_s'), 'lap_2_time_s': lap_2.get('time_s'),
        'average_velocity_m_s': result.get('average_velocity_m_s'),
        'maximum_velocity_m_s': result.get('maximum_velocity_m_s'),
        'lateral_rmse_m': result.get('lateral_rmse_m'),
        'lateral_max_error_m': result.get('lateral_max_error_m'),
        'aeb_events': result.get('aeb_events'),
        'aeb_active_time_s': result.get('aeb_active_time_s'),
        'aeb_longest_intervention_s': result.get('aeb_longest_intervention_s'),
        'minimum_front_distance_m': result.get('minimum_lidar_distance_m'),
        'path_points': record['path'].get('points'), 'path_length_m': record['path'].get('length_m'),
        'path_sha256': record['path'].get('sha256'),
    }


def _nearest_path_coordinate(rows: list[dict[str, str]], path_s: float) -> dict[str, Any]:
    choices = [(abs((value or math.inf) - path_s), row)
               for row in rows if (value := _number(row.get('path_s_m'))) is not None]
    if not choices:
        return {'status': 'missing_trace'}
    _, selected = min(choices, key=lambda item: item[0])
    return {'status': 'nearest_path_coordinate', 'reference_path_s_m': path_s,
            **{field: selected.get(field) for field in TRACE_FIELDS}}


def _minimum_front_in_window(rows: list[dict[str, str]], lower_s: float, upper_s: float) -> dict[str, Any]:
    candidates = []
    for row in rows:
        path_s, front = _number(row.get('path_s_m')), _number(row.get('aeb_critical_distance_m'))
        if path_s is not None and front is not None and lower_s <= path_s <= upper_s:
            candidates.append((front, row))
    if not candidates:
        return {'status': 'missing_window_trace', 'window_path_s_m': [lower_s, upper_s]}
    _, selected = min(candidates, key=lambda item: item[0])
    return {'status': 'minimum_front_distance_in_window', 'window_path_s_m': [lower_s, upper_s],
            **{field: selected.get(field) for field in TRACE_FIELDS}}


def _stats(records: list[dict[str, Any]]) -> dict[str, Any]:
    output: dict[str, Any] = {'n': len(records)}
    for key in METRICS:
        values: list[float] = []
        for record in records:
            result = record['result']
            if key == 'lap_1_time_s':
                value = result.get('lap_1', {}).get('time_s')
            elif key == 'lap_2_time_s':
                value = result.get('lap_2', {}).get('time_s')
            elif key == 'minimum_front_distance_m':
                value = result.get('minimum_lidar_distance_m')
            elif key == 'non_aeb_time_s':
                total = _number(result.get('total_time_s'))
                active = _number(result.get('aeb_active_time_s'))
                value = None if total is None or active is None else total - active
            else:
                value = result.get(key)
            number = _number(value)
            if number is not None:
                values.append(number)
        if not values:
            continue
        mean = statistics.mean(values)
        std = statistics.stdev(values) if len(values) > 1 else None
        output[key] = {
            'values': values, 'mean': mean, 'sample_std': std,
            'coefficient_of_variation_percent': None if std is None or mean == 0.0 else 100.0 * std / mean,
        }
    return output


def _manual_records(root: Path) -> list[dict[str, Any]]:
    output = []
    for directory in sorted(root.glob('run_*')):
        record = _run(directory)
        result = record['result']
        if (record['metadata'].get('capture_mode') == 'passive_manual_observation' and
                result.get('completed') is True and int(result.get('laps', 0)) == 2):
            output.append(record)
    return output


def _independent_manual_records(
    records: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Keep concurrent observer captures from artificially inflating sample size."""
    ordered = sorted(
        records,
        key=lambda item: float(item.get('bag_interval', {}).get('start_epoch_s') or math.inf),
    )
    independent: list[dict[str, Any]] = []
    duplicates: list[dict[str, Any]] = []
    for record in ordered:
        interval = record.get('bag_interval', {})
        start, end, duration = (interval.get('start_epoch_s'), interval.get('end_epoch_s'),
                                interval.get('duration_s'))
        parent: dict[str, Any] | None = None
        overlap_fraction = 0.0
        if all(isinstance(value, (int, float)) and value > 0.0 for value in (start, end, duration)):
            for previous in independent:
                prior = previous.get('bag_interval', {})
                p_start, p_end = prior.get('start_epoch_s'), prior.get('end_epoch_s')
                if not all(isinstance(value, (int, float)) and value > 0.0 for value in (p_start, p_end)):
                    continue
                overlap = max(0.0, min(float(end), float(p_end)) - max(float(start), float(p_start)))
                fraction = overlap / float(duration)
                if fraction >= 0.95:
                    parent = previous
                    overlap_fraction = fraction
                    break
        if parent is None:
            independent.append(record)
        else:
            record['duplicate_capture'] = {
                'of_run': parent['name'],
                'overlap_fraction': overlap_fraction,
            }
            duplicates.append(record)
    return independent, duplicates


def _pairwise_paths(records: list[dict[str, Any]]) -> dict[str, Any]:
    output: dict[str, Any] = {}
    for index, left in enumerate(records):
        for right in records[index + 1:]:
            output[f'{left["name"]}_vs_{right["name"]}'] = _path_comparison(
                left['directory'], right['directory'])
    return output


def _aeb_variability_accounting(records: list[dict[str, Any]]) -> dict[str, Any]:
    """Descriptively split observed elapsed time into AEB and non-AEB intervals.

    This is not a causal counterfactual: an active AEB may still permit turning.
    It does show whether the observed time range is dominated by the periods in
    which the safety layer was intervening.
    """
    rows = []
    for record in records:
        result = record['result']
        total, active = _number(result.get('total_time_s')), _number(result.get('aeb_active_time_s'))
        if total is not None and active is not None:
            rows.append({'run': record['name'], 'total_time_s': total, 'aeb_active_time_s': active,
                         'non_aeb_time_s': total - active})
    if len(rows) < 2:
        return {'status': 'insufficient_independent_runs', 'runs': rows}
    fastest, slowest = min(rows, key=lambda item: item['total_time_s']), max(rows, key=lambda item: item['total_time_s'])
    total_delta = slowest['total_time_s'] - fastest['total_time_s']
    aeb_delta = slowest['aeb_active_time_s'] - fastest['aeb_active_time_s']
    normal_delta = slowest['non_aeb_time_s'] - fastest['non_aeb_time_s']
    return {
        'status': 'descriptive_not_causal', 'runs': rows,
        'fastest_run': fastest['run'], 'slowest_run': slowest['run'],
        'total_time_range_s': total_delta, 'aeb_active_time_difference_s': aeb_delta,
        'non_aeb_time_difference_s': normal_delta,
        'aeb_fraction_of_total_range_percent': None if total_delta == 0.0 else 100.0 * aeb_delta / total_delta,
        'non_aeb_fraction_of_total_range_percent': None if total_delta == 0.0 else 100.0 * normal_delta / total_delta,
    }


def _automatic_against_manual(automatic: list[dict[str, Any]], manual: list[dict[str, Any]]) -> dict[str, Any]:
    manual_values = [_number(record['result'].get('total_time_s')) for record in manual]
    values = [value for value in manual_values if value is not None]
    if not values:
        return {'status': 'no_manual_reference'}
    mean = statistics.mean(values)
    sample_std = statistics.stdev(values) if len(values) > 1 else None
    comparisons = []
    for record in automatic:
        total = _number(record['result'].get('total_time_s'))
        if total is None:
            continue
        comparisons.append({
            'run': record['name'], 'total_time_s': total, 'difference_from_manual_mean_s': total - mean,
            'z_from_manual_mean': None if not sample_std else (total - mean) / sample_std,
            'inside_observed_manual_range': min(values) <= total <= max(values),
            'inside_two_manual_sample_std': None if not sample_std else abs(total - mean) <= 2.0 * sample_std,
        })
    return {
        'status': 'provisional_insufficient_manual_n' if len(values) < 3 else 'evaluated',
        'manual_n': len(values), 'manual_mean_total_time_s': mean,
        'manual_sample_std_total_time_s': sample_std, 'comparisons': comparisons,
    }


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    columns = list(rows[0]) if rows else []
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def _write_markdown(destination: Path, analysis: dict[str, Any]) -> None:
    records = analysis['records']
    lines = [
        '# Baseline reproducibility analysis', '',
        'This report is observational. It changes no controller, planner, AEB, EKF, AMCL, TF, map or world.', '',
        '## Two-lap metrics', '',
        '| Run | Mode | Complete | Laps | Total s | Lap 1 s | Lap 2 s | Avg v | Max v | RMSE | Max error | AEB events | AEB s | Longest AEB s | Front min m |',
        '|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|',
    ]
    for row in records:
        lines.append(
            f'| {row["run"]} | {row["mode"]} | {row["completed"]} | {row["laps"]} | '
            f'{_format(row["total_time_s"], 3)} | {_format(row["lap_1_time_s"], 3)} | '
            f'{_format(row["lap_2_time_s"], 3)} | {_format(row["average_velocity_m_s"], 3)} | '
            f'{_format(row["maximum_velocity_m_s"], 3)} | {_format(row["lateral_rmse_m"], 4)} | '
            f'{_format(row["lateral_max_error_m"], 4)} | {row["aeb_events"]} | '
            f'{_format(row["aeb_active_time_s"], 3)} | {_format(row["aeb_longest_intervention_s"], 3)} | '
            f'{_format(row["minimum_front_distance_m"], 4)} |')
    lines.extend(['', '## Planned-path equivalence', ''])
    for name, comparison in analysis['path_comparisons'].items():
        lines.append(
            f'- `{name}`: `{comparison.get("status")}`; poses '
            f'`{comparison.get("left_points")}` / `{comparison.get("right_points")}`; '
            f'XY RMS/max `{_format(comparison.get("xy_rms_m"), 6)}` / '
            f'`{_format(comparison.get("xy_max_m"), 6)}` m; heading RMS/max '
            f'`{_format(comparison.get("heading_rms_rad"), 6)}` / '
            f'`{_format(comparison.get("heading_max_rad"), 6)}` rad.')
    lines.extend(['', '## Manual captures retained', '',
                  '| Run | Included in independent statistics | Complete | Total s | Lap 1 s | Lap 2 s | RMSE | AEB s | AEB events | Front min m |',
                  '|---|---|---|---:|---:|---:|---:|---:|---:|---:|'])
    duplicate_names = {item['run'] for item in analysis['manual_repeatability'].get('duplicate_captures', [])}
    for row in analysis['manual_records']:
        included = 'no (concurrent duplicate)' if row['run'] in duplicate_names else 'yes'
        lines.append(
            f'| {row["run"]} | {included} | {row["completed"]} | {_format(row["total_time_s"], 3)} | '
            f'{_format(row["lap_1_time_s"], 3)} | {_format(row["lap_2_time_s"], 3)} | '
            f'{_format(row["lateral_rmse_m"], 4)} | {_format(row["aeb_active_time_s"], 3)} | '
            f'{row["aeb_events"]} | {_format(row["minimum_front_distance_m"], 4)} |')
    lines.extend(['', 'Manual planned-path comparisons, including the concurrent capture check:', ''])
    for name, comparison in analysis['manual_path_comparisons'].items():
        lines.append(
            f'- `{name}`: `{comparison.get("status")}`; XY RMS/max '
            f'`{_format(comparison.get("xy_rms_m"), 6)}` / `{_format(comparison.get("xy_max_m"), 6)}` m; '
            f'heading RMS/max `{_format(comparison.get("heading_rms_rad"), 6)}` / '
            f'`{_format(comparison.get("heading_max_rad"), 6)}` rad.')
    lines.extend(['', '## Reference of failed run_006', '',
                  'The failed reference is retained only as diagnostic evidence; it is not part of repeatability statistics.',
                  f'- Its minimum front distance occurred at path coordinate `s={_format(analysis["failed_reference"].get("path_s_m"), 3)} m`.',
                  '- The complete same-progress comparison—including nav, mux and controller commands, '
                  'lookahead point/index, actual twist, AEB and front distance—is in '
                  '`baseline_reproducibility_zone_diagnostics.csv`.',
                  '', '## Narrow-zone comparison', '',
                  'Rows are the minimum front-distance sample inside the successful-route window `s=253.8–255.0 m`.', '',
                  '| Run | x | y | yaw | closest | target | target v | target omega | controller v | actual v | actual omega | AEB | blocked | front m | lateral error | progress % |',
                  '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|---:|---:|---:|'])
    for name, row in analysis['narrow_zone'].items():
        lines.append(
            f'| {name} | {_format(row.get("x_m"), 4)} | {_format(row.get("y_m"), 4)} | '
            f'{_format(row.get("yaw_rad"), 4)} | {row.get("closest_index", "—")} | '
            f'{row.get("target_index", "—")} | {_format(row.get("target_v_m_s"), 3)} | '
            f'{_format(row.get("target_omega_rad_s"), 3)} | {_format(row.get("output_v_m_s"), 3)} | '
            f'{_format(row.get("actual_v_m_s"), 3)} | {_format(row.get("actual_omega_rad_s"), 3)} | '
            f'{row.get("aeb_active", "—")} | {row.get("aeb_command_blocked", "—")} | '
            f'{_format(row.get("aeb_critical_distance_m"), 4)} | {_format(row.get("lateral_error_m"), 4)} | '
            f'{_format(row.get("progress_percent"), 3)} |')
    lines.extend(['', '## Narrow-zone: all manual captures', '',
                  'Each row is the minimum front-clearance sample in `s=253.8–255.0 m`; the adjacent YAML and diagnostic CSV retain all command and lookahead fields.', '',
                  '| Run | x | y | yaw | closest | target | lookahead x,y | lateral error | nav v | nav omega | AEB/blocked | front m |',
                  '|---|---:|---:|---:|---:|---:|---|---:|---:|---:|---|---:|'])
    for name, row in analysis['manual_narrow_zone'].items():
        lines.append(
            f'| {name} | {_format(row.get("x_m"), 4)} | {_format(row.get("y_m"), 4)} | '
            f'{_format(row.get("yaw_rad"), 4)} | {row.get("closest_index", "—")} | '
            f'{row.get("target_index", "—")} | {_format(row.get("lookahead_x_m"), 4)}, '
            f'{_format(row.get("lookahead_y_m"), 4)} | {_format(row.get("lateral_error_m"), 4)} | '
            f'{_format(row.get("target_v_m_s"), 3)} | {_format(row.get("target_omega_rad_s"), 3)} | '
            f'{row.get("aeb_active", "—")}/{row.get("aeb_command_blocked", "—")} | '
            f'{_format(row.get("aeb_critical_distance_m"), 4)} |')
    lines.extend(['',
                  'The diagnostic CSV also contains the exact row at `run_006`\'s failed path coordinate '
                  'and the minimum-front-distance row in this successful-route narrow-zone window for every run.',
                  '', '## Manual repeatability', ''])
    manual_stats = analysis['manual_repeatability']
    lines.append(f'- Independent completed manual runs included in statistics: `{manual_stats.get("n", 0)}` '
                 f'of `{manual_stats.get("captured_manual_runs", 0)}` recorder captures.')
    for duplicate in manual_stats.get('duplicate_captures', []):
        lines.append(f'- Excluded `{duplicate["run"]}` as a duplicate observer capture of '
                     f'`{duplicate["of_run"]}`: rosbag overlap '
                     f'`{_format(100.0 * duplicate["overlap_fraction"], 3)} %`.')
    for key in METRICS:
        metric = manual_stats.get(key)
        if metric:
            lines.append(f'- `{key}`: mean `{_format(metric.get("mean"), 4)}`, sample std '
                         f'`{_format(metric.get("sample_std"), 4)}`, CV '
                         f'`{_format(metric.get("coefficient_of_variation_percent"), 3)} %`.')
    attribution = analysis['aeb_variability_accounting']
    lines.extend(['', '## AEB contribution to observed manual time variation', ''])
    if attribution.get('status') == 'descriptive_not_causal':
        lines.append(
            f'- Slowest vs fastest independent run: `{attribution["slowest_run"]}` vs '
            f'`{attribution["fastest_run"]}`; total difference `{_format(attribution.get("total_time_range_s"), 3)}` s.')
        lines.append(
            f'- Difference in AEB-active time: `{_format(attribution.get("aeb_active_time_difference_s"), 3)}` s '
            f'(`{_format(attribution.get("aeb_fraction_of_total_range_percent"), 2)} %` of the total difference); '
            f'non-AEB time difference: `{_format(attribution.get("non_aeb_time_difference_s"), 3)}` s.')
        lines.append('- This is descriptive accounting, not a causal estimate: AEB-active samples can retain angular motion.')
    else:
        lines.append('- Insufficient independent manual runs for an AEB variation accounting.')
    automatic = analysis['automatic_against_manual']
    lines.extend(['', '## Corrected automatic baselines versus manual reference', ''])
    lines.append(f'- Assessment status: `{automatic.get("status")}`; independent manual n=`{automatic.get("manual_n", 0)}`.')
    for item in automatic.get('comparisons', []):
        lines.append(f'- `{item["run"]}`: difference from manual mean `{_format(item.get("difference_from_manual_mean_s"), 3)}` s; '
                     f'z `{_format(item.get("z_from_manual_mean"), 3)}`; inside observed range '
                     f'`{item.get("inside_observed_manual_range")}`, inside ±2 sample SD '
                     f'`{item.get("inside_two_manual_sample_std")}`.')
    lines.extend(['', 'Full startup snapshots (Gazebo, EKF, AMCL covariance and TF) and all requested trace fields are in the adjacent YAML artifact.', ''])
    destination.write_text('\n'.join(lines), encoding='utf-8')


def _diagnostic_row(zone: str, run: str, row: dict[str, Any]) -> dict[str, Any]:
    return {'zone': zone, 'run': run, 'status': row.get('status'),
            'window_path_s_m': row.get('window_path_s_m'),
            'reference_path_s_m': row.get('reference_path_s_m'),
            **{field: row.get(field) for field in TRACE_FIELDS}}


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-root', type=Path, default=Path('optimization_results'))
    parser.add_argument('--golden', default='run_012_golden_manual_baseline')
    parser.add_argument('--automatic', nargs='*', default=DEFAULT_AUTOMATIC)
    parser.add_argument('--failed-reference', default=DEFAULT_FAILED_REFERENCE)
    parser.add_argument('--narrow-window', nargs=2, type=float, default=(253.8, 255.0))
    parser.add_argument('--output-prefix', default='baseline_reproducibility')
    parsed = parser.parse_args(args)
    root = parsed.results_root.resolve()
    selected_names = [parsed.golden, *parsed.automatic]
    records = [_run(root / name) for name in selected_names]
    missing = [record['name'] for record in records if not record['directory'].is_dir()]
    if missing:
        raise SystemExit(f'Missing run directories: {", ".join(missing)}')
    failed = _run(root / parsed.failed_reference)
    failed_reference = _minimum_front_in_window(failed['trace'], -math.inf, math.inf)
    lower, upper = parsed.narrow_window
    same_progress = {
        record['name']: _nearest_path_coordinate(record['trace'], _number(failed_reference.get('path_s_m')) or 0.0)
        for record in records
    }
    narrow_zone = {record['name']: _minimum_front_in_window(record['trace'], lower, upper)
                   for record in records}
    diagnostics = [_diagnostic_row('run_006_failed_lock', failed['name'], failed_reference)]
    diagnostics.extend(_diagnostic_row('run_006_same_path_coordinate', name, row)
                       for name, row in same_progress.items())
    diagnostics.extend(_diagnostic_row('successful_route_narrow_zone', name, row)
                       for name, row in narrow_zone.items())
    all_manual = _manual_records(root)
    independent_manual, duplicate_manual = _independent_manual_records(all_manual)
    manual_narrow_zone = {record['name']: _minimum_front_in_window(record['trace'], lower, upper)
                          for record in all_manual}
    manual_repeatability = _stats(independent_manual)
    manual_repeatability['captured_manual_runs'] = len(all_manual)
    manual_repeatability['duplicate_captures'] = [
        {
            'run': record['name'],
            'of_run': record['duplicate_capture']['of_run'],
            'overlap_fraction': record['duplicate_capture']['overlap_fraction'],
        }
        for record in duplicate_manual
    ]
    analysis = {
        'runs': selected_names,
        'records': [_metric_row(record) for record in records],
        'manual_records': [_metric_row(record) for record in all_manual],
        'startup': {record['name']: _startup(record) for record in records},
        'path_comparisons': _pairwise_paths(records),
        'manual_path_comparisons': _pairwise_paths(all_manual),
        'failed_reference': {'run': failed['name'], **failed_reference},
        'failed_reference_same_progress': same_progress,
        'narrow_zone': narrow_zone,
        'manual_narrow_zone': manual_narrow_zone,
        'zone_diagnostics': diagnostics,
        'manual_repeatability': manual_repeatability,
        'aeb_variability_accounting': _aeb_variability_accounting(independent_manual),
        'automatic_against_manual': _automatic_against_manual(
            [record for record in records
             if record['metadata'].get('capture_mode') != 'passive_manual_observation'],
            independent_manual,
        ),
        'automatic_repeatability': _stats([record for record in records
                                            if record['metadata'].get('capture_mode') != 'passive_manual_observation']),
    }
    prefix = root / parsed.output_prefix
    _write_csv(prefix.with_suffix('.csv'), analysis['records'])
    _write_csv(prefix.with_name(prefix.name + '_zone_diagnostics').with_suffix('.csv'), diagnostics)
    with prefix.with_suffix('.yaml').open('w', encoding='utf-8') as stream:
        yaml.safe_dump(analysis, stream, sort_keys=False)
    _write_markdown(prefix.with_suffix('.md'), analysis)
    print(f'Wrote {prefix.with_suffix(".csv")}, {prefix.with_name(prefix.name + "_zone_diagnostics").with_suffix(".csv")}, '
          f'{prefix.with_suffix(".yaml")} and {prefix.with_suffix(".md")}')


if __name__ == '__main__':
    main()
