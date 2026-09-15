#!/usr/bin/env python3
"""Create the comparative CSV, figures, and report from experiment folders."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import statistics
from typing import Any

import matplotlib.pyplot as plt
import yaml


SUMMARY_COLUMNS = [
    'run', 'group', 'controller', 'completed', 'valid', 'reason', 'total_time_s',
    'lap_1_time_s', 'lap_2_time_s', 'average_velocity_m_s', 'maximum_velocity_m_s',
    'lateral_rmse_m', 'lateral_max_error_m', 'aeb_events', 'aeb_active_time_s',
    'aeb_longest_intervention_s', 'aeb_active_fraction', 'aeb_command_blocked_time_s',
    'minimum_lidar_distance_m',
    'omega_saturated_time_fraction', 'wheel_limit_exceeded_time_fraction', 'score',
]


def _load(path: Path) -> dict[str, Any]:
    with path.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream) or {}


def _number(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _fmt(value: Any, digits: int = 3) -> str:
    numeric = _number(value)
    return '—' if numeric is None else f'{numeric:.{digits}f}'


def _read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def _records(root: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for results_path in sorted(root.glob('run_*/results.yaml')):
        run_dir = results_path.parent
        result_file = _load(results_path)
        metadata = _load(run_dir / 'metadata.yaml') if (run_dir / 'metadata.yaml').exists() else {}
        parameters = _load(run_dir / 'parameters.yaml') if (run_dir / 'parameters.yaml').exists() else {}
        result = result_file.get('result', {})
        records.append({
            'run': run_dir.name,
            'group': str(metadata.get('group', metadata.get('name', run_dir.name))),
            'controller': str(metadata.get('controller', result_file.get('controller', 'unknown'))),
            'result': result,
            'safety': result_file.get('safety', {}),
            'metadata': metadata,
            'parameters': parameters,
            'run_dir': run_dir,
        })
    return records


def _baseline(records: list[dict[str, Any]]) -> dict[str, Any] | None:
    for record in records:
        if record['metadata'].get('name') == 'baseline' and record['result'].get('completed'):
            return record
    return next((record for record in records if record['result'].get('completed')), None)


def _limits(baseline: dict[str, Any] | None, rmse: float | None, maximum: float | None) -> tuple[float, float]:
    if rmse is not None and maximum is not None:
        return rmse, maximum
    if baseline is None:
        return 0.10, 0.30
    result = baseline['result']
    base_rmse = _number(result.get('lateral_rmse_m')) or 0.05
    base_max = _number(result.get('lateral_max_error_m')) or 0.15
    # Baseline-relative tolerances prevent a faster but visibly degraded path
    # from winning, while absolute floors avoid pathological near-zero noise.
    return max(0.05, 1.25 * base_rmse + 0.01), max(0.15, 1.25 * base_max + 0.02)


def _decorate(records: list[dict[str, Any]], rmse_limit: float, max_error_limit: float,
              weights: dict[str, float]) -> None:
    for record in records:
        result = record['result']
        complete = bool(result.get('completed', False)) and int(result.get('laps', 0)) == \
            int(result.get('expected_laps', 2))
        total_time = _number(result.get('total_time_s'))
        rmse = _number(result.get('lateral_rmse_m'))
        max_error = _number(result.get('lateral_max_error_m'))
        wheel_excess = _number(result.get('wheel_limit_exceeded_time_fraction')) or 0.0
        collision_detected = record.get('safety', {}).get('collision_detected')
        valid = bool(complete and total_time is not None and rmse is not None and max_error is not None and
                     rmse <= rmse_limit and max_error <= max_error_limit and wheel_excess <= 1e-6 and
                     collision_detected is not True)
        score = None
        if valid:
            score = total_time + weights['rmse'] * rmse + weights['max_error'] * max_error + \
                weights['aeb'] * (_number(result.get('aeb_active_time_s')) or 0.0) + \
                weights['saturation'] * (_number(result.get('omega_saturated_time_fraction')) or 0.0)
        record['valid'] = valid
        record['score'] = score


def _summary_row(record: dict[str, Any]) -> dict[str, Any]:
    result = record['result']
    lap1, lap2 = result.get('lap_1', {}), result.get('lap_2', {})
    return {
        'run': record['run'], 'group': record['group'], 'controller': record['controller'],
        'completed': bool(result.get('completed', False)), 'valid': record.get('valid', False),
        'reason': result.get('reason', ''), 'total_time_s': result.get('total_time_s'),
        'lap_1_time_s': lap1.get('time_s'), 'lap_2_time_s': lap2.get('time_s'),
        'average_velocity_m_s': result.get('average_velocity_m_s'),
        'maximum_velocity_m_s': result.get('maximum_velocity_m_s'),
        'lateral_rmse_m': result.get('lateral_rmse_m'),
        'lateral_max_error_m': result.get('lateral_max_error_m'),
        'aeb_events': result.get('aeb_events'), 'aeb_active_time_s': result.get('aeb_active_time_s'),
        'aeb_longest_intervention_s': result.get('aeb_longest_intervention_s'),
        'aeb_active_fraction': result.get('aeb_active_fraction'),
        'aeb_command_blocked_time_s': result.get('aeb_command_blocked_time_s'),
        'minimum_lidar_distance_m': result.get('minimum_lidar_distance_m'),
        'omega_saturated_time_fraction': result.get('omega_saturated_time_fraction'),
        'wheel_limit_exceeded_time_fraction': result.get('wheel_limit_exceeded_time_fraction'),
        'score': record.get('score'),
    }


def _trace(record: dict[str, Any]) -> list[dict[str, str]]:
    return _read_csv(record['run_dir'] / 'monitor_trace.csv')


def _series(rows: list[dict[str, str]], field: str) -> list[float]:
    result = []
    for row in rows:
        value = _number(row.get(field))
        if value is not None:
            result.append(value)
    return result


def _time(rows: list[dict[str, str]]) -> list[float]:
    values = _series(rows, 'time_s')
    return [] if not values else [value - values[0] for value in values]


def _plot_path_profile(record: dict[str, Any], plots: Path) -> None:
    rows = _read_csv(record['run_dir'] / 'path_profile.csv')
    if not rows:
        return
    s, curvature, heading = _series(rows, 's_m'), _series(rows, 'curvature_1_m'), _series(rows, 'heading_rad')
    if len(s) != len(curvature) or len(s) != len(heading):
        return
    figure, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    axes[0].plot(s, curvature, color='tab:purple')
    axes[0].set(ylabel='curvature [1/m]', title='Hybrid A* path geometry')
    axes[1].plot(s, heading, color='tab:gray')
    axes[1].set(xlabel='arc length s [m]', ylabel='heading [rad]')
    for axis in axes:
        axis.grid(alpha=0.3)
    figure.tight_layout()
    figure.savefig(plots / 'path_profile_curvature_heading.png', dpi=180)
    plt.close(figure)


def _truth(value: Any) -> bool:
    return str(value).strip().lower() in {'true', '1', 'yes'}


def _plot_aeb_intervention_window(record: dict[str, Any], plots: Path) -> None:
    """Show whether the longest AEB intervention retained motion or progress."""
    rows = _trace(record)
    intervals = record.get('safety', {}).get('aeb_interventions', [])
    if not rows or not isinstance(intervals, list):
        return
    valid_intervals = [item for item in intervals if isinstance(item, dict) and
                       _number(item.get('duration_s')) is not None]
    if not valid_intervals:
        return
    interval = max(valid_intervals, key=lambda item: _number(item.get('duration_s')) or 0.0)
    origin = _number(rows[0].get('time_s'))
    start = _number(interval.get('start_time_s'))
    end = _number(interval.get('end_time_s'))
    if origin is None or start is None or end is None:
        return
    lower, upper = origin + max(0.0, start - 5.0), origin + end + 5.0
    window = [row for row in rows if (value := _number(row.get('time_s'))) is not None and lower <= value <= upper]
    if not window:
        return
    times = [(_number(row.get('time_s')) or origin) - origin for row in window]
    figure, axes = plt.subplots(4, 1, sharex=True, figsize=(11, 10))
    axes[0].step(times, [1.0 if _truth(row.get('aeb_active')) else 0.0 for row in window],
                 where='post', label='AEB active', color='tab:red')
    axes[0].step(times, [1.0 if _truth(row.get('aeb_command_blocked')) else 0.0 for row in window],
                 where='post', label='forward command blocked', color='tab:orange')
    axes[0].set(ylabel='state', title='Longest AEB intervention: recovery evidence')
    axes[0].set_yticks([0, 1]); axes[0].grid(alpha=0.3); axes[0].legend(loc='upper right')
    distances = [_number(row.get('aeb_critical_distance_m')) for row in window]
    axes[1].plot(times, [value if value is not None else math.nan for value in distances], color='tab:purple')
    axes[1].axhline(0.60, color='tab:red', linestyle='--', linewidth=1, label='configured AEB secure distance')
    axes[1].set(ylabel='front lidar min [m]'); axes[1].grid(alpha=0.3); axes[1].legend(loc='upper right')
    for field, label, color in [
            ('target_v_m_s', 'nav command', 'tab:blue'),
            ('mux_v_m_s', 'mux command', 'tab:green'),
            ('output_v_m_s', 'controller command', 'tab:red'),
            ('actual_v_m_s', 'actual speed', 'black')]:
        axes[2].plot(times, [_number(row.get(field)) or 0.0 for row in window], label=label, color=color)
    axes[2].set(ylabel='linear speed [m/s]'); axes[2].grid(alpha=0.3); axes[2].legend(ncol=2, fontsize=8)
    axes[3].plot(times, [_number(row.get('actual_omega_rad_s')) or 0.0 for row in window],
                 label='actual omega', color='tab:green')
    progress_axis = axes[3].twinx()
    progress_axis.plot(times, [(_number(row.get('progress_percent')) or 0.0) / 100.0 for row in window],
                       label='path progress', color='tab:blue')
    axes[3].set(xlabel='mission time [s]', ylabel='actual omega [rad/s]')
    progress_axis.set_ylabel('path progress [0–1]')
    axes[3].grid(alpha=0.3)
    lines, labels = axes[3].get_legend_handles_labels()
    right_lines, right_labels = progress_axis.get_legend_handles_labels()
    axes[3].legend(lines + right_lines, labels + right_labels, loc='upper right')
    figure.tight_layout()
    figure.savefig(plots / 'baseline_aeb_intervention_window.png', dpi=180)
    plt.close(figure)


def _plot_time(records: list[dict[str, Any]], plots: Path) -> None:
    ordered = sorted(records, key=lambda item: (_number(item['result'].get('total_time_s')) is None,
                                                 _number(item['result'].get('total_time_s')) or math.inf))
    labels = [item['run'].replace('run_', '') for item in ordered]
    values = [_number(item['result'].get('total_time_s')) for item in ordered]
    figure, axis = plt.subplots(figsize=(max(8, 0.7 * len(labels)), 4.5))
    bars = axis.bar(labels, [value or 0.0 for value in values],
                    color=['tab:green' if item.get('valid') else 'tab:red' for item in ordered])
    for bar, value in zip(bars, values):
        if value is None:
            axis.text(bar.get_x() + bar.get_width() / 2.0, 0.02, 'FAILED', ha='center', rotation=90)
    axis.set(xlabel='candidate', ylabel='two-lap time [s]', title='Total time by candidate')
    axis.tick_params(axis='x', rotation=45)
    axis.grid(axis='y', alpha=0.3)
    figure.tight_layout()
    figure.savefig(plots / 'total_time_vs_candidate.png', dpi=180)
    plt.close(figure)


def _plot_baseline_vs_winner(baseline: dict[str, Any], winner: dict[str, Any], plots: Path) -> None:
    baseline_rows, winner_rows = _trace(baseline), _trace(winner)
    if not baseline_rows or not winner_rows:
        return
    figure, axis = plt.subplots(figsize=(10, 4.5))
    for label, rows, color in [('baseline', baseline_rows, 'tab:blue'), ('winner', winner_rows, 'tab:orange')]:
        times, values = _time(rows), _series(rows, 'actual_v_m_s')
        if len(times) == len(values):
            axis.plot(times, values, label=label, color=color)
    axis.set(xlabel='time [s]', ylabel='actual linear speed [m/s]', title='Measured speed: baseline vs winner')
    axis.grid(alpha=0.3); axis.legend(); figure.tight_layout()
    figure.savefig(plots / 'linear_speed_baseline_vs_winner.png', dpi=180)
    plt.close(figure)

    times = _time(winner_rows)
    figure, axis = plt.subplots(figsize=(10, 4.5))
    for field, label, color in [('target_v_m_s', 'target speed', 'tab:orange'),
                                ('actual_v_m_s', 'actual speed', 'tab:blue')]:
        values = _series(winner_rows, field)
        if len(times) == len(values):
            axis.plot(times, values, label=label, color=color)
    axis.set(xlabel='time [s]', ylabel='speed [m/s]', title='Winner: target vs actual speed')
    axis.grid(alpha=0.3); axis.legend(); figure.tight_layout()
    figure.savefig(plots / 'winner_target_vs_actual_speed.png', dpi=180)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(10, 4.5))
    for field, label, color in [('lateral_error_m', 'baseline', 'tab:blue'),
                                ('lateral_error_m', 'winner', 'tab:orange')]:
        rows = baseline_rows if label == 'baseline' else winner_rows
        times, values = _time(rows), _series(rows, field)
        if len(times) == len(values):
            axis.plot(times, values, label=label, color=color)
    axis.set(xlabel='time [s]', ylabel='lateral error [m]', title='Lateral tracking error')
    axis.grid(alpha=0.3); axis.legend(); figure.tight_layout()
    figure.savefig(plots / 'lateral_error_baseline_vs_winner.png', dpi=180)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(7, 5))
    profile = _read_csv(baseline['run_dir'] / 'path_profile.csv')
    if profile:
        axis.plot(_series(profile, 'x_m'), _series(profile, 'y_m'), 'k--', label='planned path')
    for label, rows, color in [('baseline', baseline_rows, 'tab:blue'), ('winner', winner_rows, 'tab:orange')]:
        axis.plot(_series(rows, 'x_m'), _series(rows, 'y_m'), color=color, label=label)
    axis.set(xlabel='x [m]', ylabel='y [m]', title='Path XY')
    axis.set_aspect('equal'); axis.grid(alpha=0.3); axis.legend(); figure.tight_layout()
    figure.savefig(plots / 'path_xy_baseline_vs_winner.png', dpi=180)
    plt.close(figure)


def _plot_winner_adaptation(winner: dict[str, Any], plots: Path) -> None:
    rows = _trace(winner)
    if not rows:
        return
    times = _time(rows)
    figure, axis = plt.subplots(figsize=(9, 4.5))
    curvature, velocity = _series(rows, 'future_curvature_1_m'), _series(rows, 'actual_v_m_s')
    if len(curvature) == len(velocity):
        axis.scatter(curvature, velocity, s=8, alpha=0.45)
    axis.set(xlabel='preview curvature |κ| [1/m]', ylabel='actual speed [m/s]',
             title='Winner: speed reduction in curves')
    axis.grid(alpha=0.3); figure.tight_layout()
    figure.savefig(plots / 'winner_curvature_vs_speed.png', dpi=180)
    plt.close(figure)

    values = _series(rows, 'lookahead_m')
    figure, axis = plt.subplots(figsize=(10, 4.5))
    if len(times) == len(values):
        axis.plot(times, values, color='tab:green')
    axis.set(xlabel='time [s]', ylabel='lookahead [m]', title='Winner: adaptive lookahead')
    axis.grid(alpha=0.3); figure.tight_layout()
    figure.savefig(plots / 'winner_lookahead_vs_time.png', dpi=180)
    plt.close(figure)


def _plot_campaign(records: list[dict[str, Any]], plots: Path) -> None:
    valid = [item for item in records if item.get('valid')]
    figure, axis = plt.subplots(figsize=(7, 5))
    for item in valid:
        result = item['result']
        x, y = _number(result.get('total_time_s')), _number(result.get('lateral_rmse_m'))
        if x is not None and y is not None:
            axis.scatter(x, y, label=item['run'])
    axis.set(xlabel='two-lap time [s]', ylabel='lateral RMSE [m]', title='Speed-accuracy trade-off')
    axis.grid(alpha=0.3)
    if valid:
        axis.legend(fontsize=7)
    figure.tight_layout()
    figure.savefig(plots / 'rmse_vs_total_time.png', dpi=180)
    plt.close(figure)

    labels = [item['run'].replace('run_', '') for item in records]
    values = [_number(item['result'].get('aeb_active_time_s')) or 0.0 for item in records]
    figure, axis = plt.subplots(figsize=(max(8, 0.7 * len(labels)), 4.5))
    axis.bar(labels, values, color='tab:red')
    axis.set(xlabel='candidate', ylabel='AEB active time [s]', title='AEB interventions by candidate')
    axis.tick_params(axis='x', rotation=45); axis.grid(axis='y', alpha=0.3); figure.tight_layout()
    figure.savefig(plots / 'aeb_active_time_vs_candidate.png', dpi=180)
    plt.close(figure)

    selected = sorted(valid, key=lambda item: item['score'])[:5]
    if selected:
        figure, axis = plt.subplots(figsize=(8, 4.5))
        labels = [item['run'].replace('run_', '') for item in selected]
        lap1 = [_number(item['result'].get('lap_1', {}).get('time_s')) or 0.0 for item in selected]
        lap2 = [_number(item['result'].get('lap_2', {}).get('time_s')) or 0.0 for item in selected]
        positions = list(range(len(labels)))
        axis.bar(positions, lap1, label='lap 1')
        axis.bar(positions, lap2, bottom=lap1, label='lap 2')
        axis.set_xticks(positions, labels, rotation=45)
        axis.set(ylabel='time [s]', title='Lap times of best valid candidates')
        axis.grid(axis='y', alpha=0.3); axis.legend(); figure.tight_layout()
        figure.savefig(plots / 'lap_times_best_candidates.png', dpi=180)
        plt.close(figure)


def _repeatability(records: list[dict[str, Any]]) -> dict[str, dict[str, float | int]]:
    groups: dict[str, list[dict[str, Any]]] = {}
    for record in records:
        if record.get('valid'):
            groups.setdefault(record['group'], []).append(record)
    output: dict[str, dict[str, float | int]] = {}
    for group, items in groups.items():
        times = [_number(item['result'].get('total_time_s')) for item in items]
        rmses = [_number(item['result'].get('lateral_rmse_m')) for item in items]
        times = [item for item in times if item is not None]
        rmses = [item for item in rmses if item is not None]
        output[group] = {
            'n': len(items),
            'mean_total_time_s': statistics.mean(times),
            'std_total_time_s': statistics.stdev(times) if len(times) > 1 else 0.0,
            'mean_rmse_m': statistics.mean(rmses),
            'std_rmse_m': statistics.stdev(rmses) if len(rmses) > 1 else 0.0,
        }
    return output


def _write_report(root: Path, records: list[dict[str, Any]], baseline: dict[str, Any] | None,
                  winner: dict[str, Any] | None, rmse_limit: float, max_error_limit: float,
                  weights: dict[str, float]) -> None:
    lines = ['# Pure Pursuit optimization report', '', '## Methodology', '',
             'Each run is stored independently with its rosbag, parameter snapshot, monitor trace, '
             'and `results.yaml`. The runner restarts the configured simulation/localization stack per run. '
             'The baseline controller is not modified; the monitor is observational. AEB state is '
             'recorded as safety evidence and never causes failure by itself.', '',
             'Mission failure due to immobilization requires a full configurable window with no path '
             'progress, displacement, closest-index advance, linear motion, or recovery rotation while '
             'the controller is requesting motion.', '',
             'The recorded lidar minimum is the `lidar_data` front sector (±20°), not a global scan '
             'minimum and not a collision label. Collision is reported as uninstrumented until a Gazebo '
             'contact topic is added.', '',
             'A run is valid only when it completes the requested two laps, remains within the '
             'baseline-derived tracking limits, and has no command beyond the coupled wheel envelope.', '',
             f'- Lateral RMSE limit: `{rmse_limit:.3f} m`',
             f'- Max lateral error limit: `{max_error_limit:.3f} m`',
             f'- Score: `T + {weights["rmse"]:.1f}·RMSE + {weights["max_error"]:.1f}·Emax '
             f'+ {weights["aeb"]:.1f}·AEB_time + {weights["saturation"]:.1f}·omega_saturation_fraction`.', '',
             '## Candidate table', '',
             '| Run | Controller | Total time s | Lap 1 | Lap 2 | Avg v | Max v | RMSE | Max error | AEB events | AEB time | Longest AEB | Min front lidar | Valid | Score |',
             '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|']
    ordered = sorted(records, key=lambda item: (item.get('score') is None, item.get('score') or math.inf))
    for item in ordered:
        result = item['result']
        lines.append(
            f'| {item["run"]} | {item["controller"]} | {_fmt(result.get("total_time_s"))} | '
            f'{_fmt(result.get("lap_1", {}).get("time_s"))} | {_fmt(result.get("lap_2", {}).get("time_s"))} | '
            f'{_fmt(result.get("average_velocity_m_s"))} | {_fmt(result.get("maximum_velocity_m_s"))} | '
            f'{_fmt(result.get("lateral_rmse_m"))} | {_fmt(result.get("lateral_max_error_m"))} | '
            f'{result.get("aeb_events", "—")} | {_fmt(result.get("aeb_active_time_s"))} | '
            f'{_fmt(result.get("aeb_longest_intervention_s"))} | {_fmt(result.get("minimum_lidar_distance_m"))} | '
            f'{"YES" if item.get("valid") else "FAILED"} | {_fmt(item.get("score"))} |')
    lines.extend(['', '## Baseline vs selected candidate', ''])
    if baseline is None:
        lines.append('No completed baseline is available yet; no performance claim is made.')
    elif winner is None:
        lines.append('No valid optimized candidate is available yet; baseline remains the reference.')
    else:
        base, best = baseline['result'], winner['result']
        base_time = _number(base.get('total_time_s'))
        best_time = _number(best.get('total_time_s'))
        improvement = None if not base_time or best_time is None else 100.0 * (base_time - best_time) / base_time
        lines.extend([
            '| Metric | Baseline | Selected |', '|---|---:|---:|',
            f'| Two-lap time [s] | {_fmt(base_time)} | {_fmt(best_time)} |',
            f'| Improvement | — | {_fmt(improvement, 2)} % |',
            f'| Average velocity [m/s] | {_fmt(base.get("average_velocity_m_s"))} | {_fmt(best.get("average_velocity_m_s"))} |',
            f'| Lateral RMSE [m] | {_fmt(base.get("lateral_rmse_m"))} | {_fmt(best.get("lateral_rmse_m"))} |',
            f'| Max lateral error [m] | {_fmt(base.get("lateral_max_error_m"))} | {_fmt(best.get("lateral_max_error_m"))} |',
            f'| AEB events | {base.get("aeb_events", "—")} | {best.get("aeb_events", "—")} |',
        ])
    repeatability = _repeatability(records)
    lines.extend(['', '## Repeatability', '', '| Group | n | Mean time s | SD time s | Mean RMSE m | SD RMSE m |',
                  '|---|---:|---:|---:|---:|---:|'])
    for group, values in sorted(repeatability.items()):
        lines.append(f'| {group} | {values["n"]} | {values["mean_total_time_s"]:.3f} | '
                     f'{values["std_total_time_s"]:.3f} | {values["mean_rmse_m"]:.4f} | '
                     f'{values["std_rmse_m"]:.4f} |')
    lines.extend(['', 'A recommended winner requires at least three valid repetitions of the same '
                  'configuration. Until then, the fastest row is explicitly provisional.', ''])
    (root / 'optimization_report.md').write_text('\n'.join(lines), encoding='utf-8')


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-root', type=Path, default=Path('optimization_results'))
    parser.add_argument('--rmse-limit', type=float)
    parser.add_argument('--max-error-limit', type=float)
    parsed = parser.parse_args(args)
    root = parsed.results_root.resolve()
    records = _records(root)
    if not records:
        raise SystemExit(f'No results.yaml found below {root}')
    baseline = _baseline(records)
    rmse_limit, max_error_limit = _limits(baseline, parsed.rmse_limit, parsed.max_error_limit)
    weights = {'rmse': 20.0, 'max_error': 10.0, 'aeb': 3.0, 'saturation': 5.0}
    _decorate(records, rmse_limit, max_error_limit, weights)
    rows = [_summary_row(record) for record in records]
    with (root / 'optimization_summary.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=SUMMARY_COLUMNS)
        writer.writeheader(); writer.writerows(rows)
    plots = root / 'plots'
    plots.mkdir(exist_ok=True)
    _plot_time(records, plots)
    _plot_campaign(records, plots)
    if baseline is not None:
        _plot_path_profile(baseline, plots)
        _plot_aeb_intervention_window(baseline, plots)
    else:
        # A failed baseline observation can still be valuable evidence when
        # diagnosing safety/recovery behavior before a valid reference exists.
        observed = next((record for record in records
                         if record['metadata'].get('name') == 'baseline' and
                         record.get('safety', {}).get('aeb_interventions')), None)
        if observed is not None:
            _plot_aeb_intervention_window(observed, plots)
    valid = [record for record in records if record.get('valid')]
    winner = min(valid, key=lambda item: item['score']) if valid else None
    if baseline is not None and winner is not None:
        _plot_baseline_vs_winner(baseline, winner, plots)
        _plot_winner_adaptation(winner, plots)
    _write_report(root, records, baseline, winner, rmse_limit, max_error_limit, weights)
    print(f'Wrote {root / "optimization_summary.csv"}, plots/, and optimization_report.md')


if __name__ == '__main__':
    main()
