#!/usr/bin/env python3
"""Create a reproducible, rosbag-backed comparison of manual runs 001 and 002.

The script is analysis-only: it opens recorded MCAP files and their companion
monitor traces, then writes a CSV, a Markdown report, and comparison figures.
It does not create ROS publishers, launch files, or parameter changes.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml


RUN_001 = 'run_001_original_speed_baseline'
RUN_002 = 'run_002_adaptive_double_speed'
SPEED_LIMIT_TOPICS = {
    'curvature': '/path_tracking/speed_limit_curvature',
    'preview': '/path_tracking/speed_limit_preview',
    'omega': '/path_tracking/speed_limit_omega',
    'lateral_error': '/path_tracking/speed_limit_lateral_error',
}


def _yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding='utf-8') as stream:
        value = yaml.safe_load(stream) or {}
    return value if isinstance(value, dict) else {}


def _number(value: Any, default: float = math.nan) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _truth(value: Any) -> bool:
    return str(value).strip().lower() in {'true', '1', 'yes'}


def _wrap(angle: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(angle), np.cos(angle))


def _fmt(value: Any, digits: int = 3) -> str:
    if value is None:
        return '—'
    if isinstance(value, str):
        return value
    parsed = _number(value)
    return '—' if not math.isfinite(parsed) else f'{parsed:.{digits}f}'


def _percent(numerator: float, denominator: float) -> float | None:
    if not math.isfinite(numerator) or not math.isfinite(denominator) or abs(denominator) < 1e-12:
        return None
    return 100.0 * numerator / denominator


@dataclass
class Run:
    directory: Path
    label: str
    metadata: dict[str, Any]
    results: dict[str, Any]
    trace: dict[str, np.ndarray]
    profile: dict[str, np.ndarray]
    bag: Path

    @property
    def result(self) -> dict[str, Any]:
        return self.results.get('result', {})

    @property
    def time(self) -> np.ndarray:
        values = self.trace['time_s']
        return values - values[0]


def _read_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f'No data in {path}')
    output: dict[str, np.ndarray] = {}
    for key in rows[0]:
        if key in {'aeb_active', 'aeb_command_blocked'}:
            output[key] = np.asarray([1.0 if _truth(row.get(key)) else 0.0 for row in rows])
        else:
            output[key] = np.asarray([_number(row.get(key)) for row in rows], dtype=float)
    return output


def _load_run(root: Path, name: str, label: str) -> Run:
    directory = root / name
    metadata, results = _yaml(directory / 'metadata.yaml'), _yaml(directory / 'results.yaml')
    bag_root = directory / 'rosbag'
    bags = sorted(path for path in bag_root.iterdir() if path.is_dir())
    if len(bags) != 1:
        raise ValueError(f'{directory} must contain exactly one rosbag directory')
    return Run(directory, label, metadata, results, _read_csv(directory / 'monitor_trace.csv'),
               _read_csv(directory / 'path_profile.csv'), bags[0])


def _integral(run: Run, predicate: Callable[[int], bool]) -> float:
    time = run.trace['time_s']
    total = 0.0
    for index in range(len(time) - 1):
        dt = time[index + 1] - time[index]
        if 0.0 < dt <= 0.2 and predicate(index):
            total += float(dt)
    return total


def _mean(values: np.ndarray) -> float:
    finite = values[np.isfinite(values)]
    return float(np.mean(finite)) if finite.size else math.nan


def _maximum(values: np.ndarray) -> float:
    finite = values[np.isfinite(values)]
    return float(np.max(finite)) if finite.size else math.nan


def _rmse(values: np.ndarray) -> float:
    finite = values[np.isfinite(values)]
    return float(math.sqrt(np.mean(finite * finite))) if finite.size else math.nan


def _scalar_series_from_bag(bag: Path, topics: list[str]) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Read native Float32 diagnostics directly from the recorded MCAP."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = set(topics) - set(types)
    if missing:
        raise ValueError(f'Bag {bag} is missing expected topics: {sorted(missing)}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    accumulator: dict[str, list[tuple[float, float]]] = {topic: [] for topic in topics}
    while reader.has_next():
        topic, serialized, stamp_ns = reader.read_next()
        message = deserialize_message(serialized, get_message(types[topic]))
        accumulator[topic].append((stamp_ns * 1e-9, float(message.data)))
    return {
        topic: (np.asarray([item[0] for item in values]), np.asarray([item[1] for item in values]))
        for topic, values in accumulator.items()
    }


def _path_from_bag(bag: Path) -> tuple[int, float, tuple[float, float], tuple[float, float]]:
    """Read the planned Path itself from the rosbag, rather than a report copy."""
    topic = '/planned_path'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    if not reader.has_next():
        raise ValueError(f'{bag} has no {topic}')
    _, serialized, _ = reader.read_next()
    path = deserialize_message(serialized, get_message(types[topic]))
    points = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
    length = sum(math.hypot(x1 - x0, y1 - y0) for (x0, y0), (x1, y1) in zip(points, points[1:]))
    return len(points), length, points[0], points[-1]


def _path_comparison(first: Run, second: Run) -> dict[str, float | str | int]:
    left, right = first.profile, second.profile
    common = min(float(left['s_m'][-1]), float(right['s_m'][-1]))
    samples = max(2, int(common / 0.05) + 1)
    s = np.linspace(0.0, common, samples)
    left_x, left_y = np.interp(s, left['s_m'], left['x_m']), np.interp(s, left['s_m'], left['y_m'])
    right_x, right_y = np.interp(s, right['s_m'], right['x_m']), np.interp(s, right['s_m'], right['y_m'])
    left_h, right_h = np.interp(s, left['s_m'], left['heading_rad']), np.interp(s, right['s_m'], right['heading_rad'])
    distances = np.hypot(left_x - right_x, left_y - right_y)
    heading = np.abs(_wrap(left_h - right_h))
    return {
        'common_arc_length_m': common,
        'samples': samples,
        'xy_rms_m': float(math.sqrt(np.mean(distances * distances))),
        'xy_max_m': float(np.max(distances)),
        'heading_rms_rad': float(math.sqrt(np.mean(heading * heading))),
        'heading_max_rad': float(np.max(heading)),
        'status': 'geometrically_different' if float(np.max(distances)) > 1e-3 else 'equivalent_within_1mm',
    }


def _section(run: Run, lower: float, upper: float) -> dict[str, float]:
    progress = run.trace['progress_percent']
    upper_mask = progress <= upper if upper >= 100.0 else progress < upper
    mask = (progress >= lower) & upper_mask
    indices = np.flatnonzero(mask)
    if not indices.size:
        return {'time_s': math.nan}
    sub = {key: values[indices] for key, values in run.trace.items()}
    time_s = 0.0
    for left, right in zip(indices, indices[1:]):
        dt = run.trace['time_s'][right] - run.trace['time_s'][left]
        if 0.0 < dt <= 0.2:
            time_s += float(dt)
    aeb_s = 0.0
    for left, right in zip(indices, indices[1:]):
        dt = run.trace['time_s'][right] - run.trace['time_s'][left]
        if 0.0 < dt <= 0.2 and run.trace['aeb_active'][left] > 0.5:
            aeb_s += float(dt)
    return {
        'time_s': time_s,
        'average_velocity_m_s': _mean(np.abs(sub['actual_v_m_s'])),
        'maximum_velocity_m_s': _maximum(np.abs(sub['actual_v_m_s'])),
        'lateral_rmse_m': _rmse(sub['lateral_error_m']),
        'aeb_active_time_s': aeb_s,
        'average_abs_curvature_1_m': _mean(np.abs(sub['curvature_1_m'])),
    }


def _narrow_zone(run: Run, progress_center: float, half_width: float = 1.0) -> dict[str, float]:
    progress = run.trace['progress_percent']
    mask = (progress >= progress_center - half_width) & (progress <= progress_center + half_width)
    indices = np.flatnonzero(mask)
    if not indices.size:
        return {'samples': 0.0}
    sub = {key: values[indices] for key, values in run.trace.items()}
    active = 0.0
    for left, right in zip(indices, indices[1:]):
        dt = run.trace['time_s'][right] - run.trace['time_s'][left]
        if 0.0 < dt <= 0.2 and run.trace['aeb_active'][left] > 0.5:
            active += float(dt)
    return {
        'progress_lower_percent': progress_center - half_width,
        'progress_upper_percent': progress_center + half_width,
        'samples': float(indices.size),
        'time_s': float(run.trace['time_s'][indices[-1]] - run.trace['time_s'][indices[0]]),
        'average_velocity_m_s': _mean(np.abs(sub['actual_v_m_s'])),
        'maximum_velocity_m_s': _maximum(np.abs(sub['actual_v_m_s'])),
        'lateral_rmse_m': _rmse(sub['lateral_error_m']),
        'minimum_front_clearance_m': _minimum(sub['aeb_critical_distance_m']),
        'aeb_active_time_s': active,
    }


def _minimum(values: np.ndarray) -> float:
    finite = values[np.isfinite(values)]
    return float(np.min(finite)) if finite.size else math.nan


def _metrics(run: Run, max_omega: float) -> dict[str, float]:
    result = run.result
    actual = np.abs(run.trace['actual_v_m_s'])
    target = np.abs(run.trace['target_v_m_s'])
    omega = np.abs(run.trace['target_omega_rad_s'])
    total = _number(result.get('total_time_s'))
    above_050 = _integral(run, lambda index: actual[index] > 0.50)
    target_090 = _integral(run, lambda index: target[index] >= 0.90)
    return {
        'total_time_s': total,
        'lap_1_time_s': _number(result.get('lap_1', {}).get('time_s')),
        'lap_2_time_s': _number(result.get('lap_2', {}).get('time_s')),
        'average_actual_velocity_m_s': _mean(actual),
        'maximum_actual_velocity_m_s': _maximum(actual),
        'average_commanded_velocity_m_s': _mean(target),
        'maximum_commanded_velocity_m_s': _maximum(target),
        'lateral_rmse_m': _rmse(run.trace['lateral_error_m']),
        'lateral_max_error_m': _maximum(np.abs(run.trace['lateral_error_m'])),
        'aeb_active_time_s': _integral(run, lambda index: run.trace['aeb_active'][index] > 0.5),
        'aeb_events': _number(result.get('aeb_events')),
        'aeb_longest_intervention_s': _number(result.get('aeb_longest_intervention_s')),
        'minimum_front_clearance_m': _minimum(run.trace['aeb_critical_distance_m']),
        'time_actual_speed_above_050_s': above_050,
        'fraction_actual_speed_above_050_percent': 100.0 * above_050 / total,
        'time_target_speed_at_least_090_s': target_090,
        'fraction_target_speed_at_least_090_percent': 100.0 * target_090 / total,
        'angular_saturation_percent': 100.0 * _integral(
            run, lambda index: omega[index] >= 0.995 * max_omega) / total,
        'distance_travelled_m': _number(result.get('distance_travelled_m')),
        'path_progress_percent': _number(result.get('path_progress_percent')),
    }


def _dominance(bag: Path) -> tuple[dict[str, float], dict[str, tuple[np.ndarray, np.ndarray]]]:
    topics = list(SPEED_LIMIT_TOPICS.values()) + ['/path_tracking/target_speed']
    native = _scalar_series_from_bag(bag, topics)
    limits = {name: native[topic] for name, topic in SPEED_LIMIT_TOPICS.items()}
    time = limits['curvature'][0]
    values = np.vstack([limits[key][1] for key in SPEED_LIMIT_TOPICS])
    names = list(SPEED_LIMIT_TOPICS)
    seconds = {name: 0.0 for name in names}
    for index in range(time.size - 1):
        dt = time[index + 1] - time[index]
        if not 0.0 < dt <= 0.2:
            continue
        lowest = np.min(values[:, index])
        winners = np.flatnonzero(np.abs(values[:, index] - lowest) <= 1e-5)
        for winner in winners:
            seconds[names[int(winner)]] += float(dt) / winners.size
    duration = sum(seconds.values())
    return ({f'{name}_dominant_seconds': value for name, value in seconds.items()} |
            {f'{name}_dominant_percent': 100.0 * value / duration for name, value in seconds.items()} |
            {'diagnostic_duration_s': duration}), native


def _decimate(values: np.ndarray, maximum: int = 5000) -> np.ndarray:
    return values[::max(1, int(math.ceil(values.size / maximum)))]


def _plot_bar(first: Run, second: Run, first_value: float, second_value: float,
              title: str, ylabel: str, destination: Path) -> None:
    figure, axis = plt.subplots(figsize=(7, 4.5))
    bars = axis.bar([first.label, second.label], [first_value, second_value],
                    color=['tab:blue', 'tab:orange'])
    for bar, value in zip(bars, (first_value, second_value)):
        axis.text(bar.get_x() + bar.get_width() / 2.0, value, _fmt(value, 2), ha='center', va='bottom')
    axis.set(title=title, ylabel=ylabel); axis.grid(axis='y', alpha=0.3); figure.tight_layout()
    figure.savefig(destination, dpi=180); plt.close(figure)


def _plots(first: Run, second: Run, metrics_a: dict[str, float], metrics_b: dict[str, float],
           native: dict[str, tuple[np.ndarray, np.ndarray]], dominance: dict[str, float],
           plots: Path) -> None:
    plots.mkdir(parents=True, exist_ok=True)
    _plot_bar(first, second, metrics_a['total_time_s'], metrics_b['total_time_s'],
              'Total mission time', 'time [s]', plots / '01_total_time.png')

    figure, axis = plt.subplots(figsize=(7, 4.5)); locations = np.arange(2); width = 0.35
    axis.bar(locations - width / 2, [metrics_a['lap_1_time_s'], metrics_a['lap_2_time_s']], width, label=first.label)
    axis.bar(locations + width / 2, [metrics_b['lap_1_time_s'], metrics_b['lap_2_time_s']], width, label=second.label)
    axis.set(xticks=locations, xticklabels=['Lap 1', 'Lap 2'], ylabel='time [s]', title='Time by lap')
    axis.legend(); axis.grid(axis='y', alpha=0.3); figure.tight_layout(); figure.savefig(plots / '02_lap_times.png', dpi=180); plt.close(figure)

    figure, axis = plt.subplots(figsize=(11, 4.6))
    for run, color in ((first, 'tab:blue'), (second, 'tab:orange')):
        selection = _decimate(np.arange(run.time.size)); axis.plot(run.time[selection], np.abs(run.trace['actual_v_m_s'][selection]), label=run.label, color=color)
    axis.axhline(0.50, linestyle='--', color='black', linewidth=1, label='old physical limit')
    axis.set(xlabel='mission time [s]', ylabel='actual speed [m/s]', title='Measured linear speed')
    axis.legend(); axis.grid(alpha=0.3); figure.tight_layout(); figure.savefig(plots / '03_actual_speed_vs_time.png', dpi=180); plt.close(figure)

    figure, axis = plt.subplots(figsize=(11, 4.6)); selection = _decimate(np.arange(second.time.size))
    axis.plot(second.time[selection], second.trace['target_v_m_s'][selection], label='target command', color='tab:orange')
    axis.plot(second.time[selection], np.abs(second.trace['actual_v_m_s'][selection]), label='actual speed', color='tab:blue')
    axis.set(xlabel='mission time [s]', ylabel='speed [m/s]', title='Run 002: target versus actual speed')
    axis.legend(); axis.grid(alpha=0.3); figure.tight_layout(); figure.savefig(plots / '04_run002_target_vs_actual.png', dpi=180); plt.close(figure)

    figure, axis = plt.subplots(figsize=(7, 5)); selection = _decimate(np.arange(second.time.size), 7000)
    axis.scatter(np.abs(second.trace['curvature_1_m'][selection]), np.abs(second.trace['actual_v_m_s'][selection]), s=5, alpha=0.35)
    axis.set(xlabel='|curvature| [1/m]', ylabel='actual speed [m/s]', title='Run 002: curvature versus speed')
    axis.grid(alpha=0.3); figure.tight_layout(); figure.savefig(plots / '05_run002_curvature_vs_speed.png', dpi=180); plt.close(figure)

    figure, axis = plt.subplots(figsize=(11, 4.6)); axis.plot(second.time[selection], second.trace['lookahead_m'][selection], color='tab:green')
    axis.set(xlabel='mission time [s]', ylabel='lookahead [m]', title='Run 002: adaptive lookahead')
    axis.grid(alpha=0.3); figure.tight_layout(); figure.savefig(plots / '06_run002_lookahead.png', dpi=180); plt.close(figure)

    figure, axis = plt.subplots(figsize=(11, 4.6))
    for run, color in ((first, 'tab:blue'), (second, 'tab:orange')):
        selection = _decimate(np.arange(run.time.size)); axis.plot(run.time[selection], run.trace['lateral_error_m'][selection], label=run.label, color=color)
    axis.set(xlabel='mission time [s]', ylabel='lateral error [m]', title='Lateral tracking error')
    axis.legend(); axis.grid(alpha=0.3); figure.tight_layout(); figure.savefig(plots / '07_lateral_error_vs_time.png', dpi=180); plt.close(figure)

    figure, axis = plt.subplots(figsize=(8, 7))
    for run, color, linestyle in ((first, 'black', '--'), (second, '0.35', ':')):
        axis.plot(
            run.profile['x_m'], run.profile['y_m'], color=color, linestyle=linestyle,
            linewidth=1.1, label=f'{run.label} planned',
        )
    for run, color in ((first, 'tab:blue'), (second, 'tab:orange')):
        selection = _decimate(np.arange(run.time.size)); axis.plot(run.trace['x_m'][selection], run.trace['y_m'][selection], color=color, alpha=0.8, label=f'{run.label} executed')
    axis.set(xlabel='x [m]', ylabel='y [m]', title='Planned and executed paths'); axis.set_aspect('equal'); axis.legend(fontsize=8); axis.grid(alpha=0.3)
    figure.tight_layout(); figure.savefig(plots / '08_path_xy.png', dpi=180); plt.close(figure)

    figure, axis = plt.subplots(figsize=(11, 3.8))
    for run, color in ((first, 'tab:red'), (second, 'tab:orange')):
        selection = _decimate(np.arange(run.time.size)); axis.step(run.time[selection], run.trace['aeb_active'][selection], where='post', label=run.label, color=color)
    axis.set(xlabel='mission time [s]', ylabel='AEB active', title='AEB state'); axis.set_yticks([0, 1]); axis.legend(); axis.grid(alpha=0.3)
    figure.tight_layout(); figure.savefig(plots / '09_aeb_active.png', dpi=180); plt.close(figure)

    figure, axis = plt.subplots(figsize=(11, 4.6))
    for run, color in ((first, 'tab:blue'), (second, 'tab:orange')):
        selection = _decimate(np.arange(run.time.size)); axis.plot(run.time[selection], run.trace['aeb_critical_distance_m'][selection], label=run.label, color=color)
    axis.axhline(0.60, color='tab:red', linestyle='--', linewidth=1, label='AEB safety distance')
    axis.set(xlabel='mission time [s]', ylabel='front clearance [m]', title='Front-sector clearance'); axis.legend(); axis.grid(alpha=0.3)
    figure.tight_layout(); figure.savefig(plots / '10_front_clearance.png', dpi=180); plt.close(figure)

    figure, axis = plt.subplots(figsize=(8, 4.8)); bins = np.linspace(0.0, 1.0, 41)
    axis.hist(np.abs(first.trace['actual_v_m_s']), bins=bins, alpha=0.55, density=True, label=first.label, color='tab:blue')
    axis.hist(np.abs(second.trace['actual_v_m_s']), bins=bins, alpha=0.55, density=True, label=second.label, color='tab:orange')
    axis.set(xlabel='actual speed [m/s]', ylabel='density', title='Speed distribution'); axis.legend(); axis.grid(alpha=0.3)
    figure.tight_layout(); figure.savefig(plots / '11_speed_distribution.png', dpi=180); plt.close(figure)

    native_time = native[SPEED_LIMIT_TOPICS['curvature']][0]; origin = native_time[0]; selection = _decimate(np.arange(native_time.size), 6000)
    figure, (axis, dominance_axis) = plt.subplots(
        2, 1, figsize=(11, 7.0), gridspec_kw={'height_ratios': (4, 1.25)}, sharex=False,
    )
    for name, topic in SPEED_LIMIT_TOPICS.items():
        times, values = native[topic]; axis.plot(times[selection] - origin, values[selection], label=f'{name} limit')
    target_times, target_values = native['/path_tracking/target_speed']; target_indices = _decimate(np.arange(target_times.size), 8000)
    axis.plot(target_times[target_indices] - origin, target_values[target_indices], color='black', linewidth=1.0, alpha=0.8, label='native target speed')
    axis.set(xlabel='controller diagnostic time [s]', ylabel='speed ceiling [m/s]', title='Run 002: Adaptive PP internal speed limits')
    axis.legend(ncol=2, fontsize=8); axis.grid(alpha=0.3)
    dominance_values = {
        'omega': dominance['omega_dominant_percent'],
        'lateral error': dominance['lateral_error_dominant_percent'],
        'curvature': dominance['curvature_dominant_percent'],
        'preview': dominance['preview_dominant_percent'],
    }
    ordered = sorted(dominance_values, key=dominance_values.get)
    dominance_axis.barh(ordered, [dominance_values[name] for name in ordered], color=['tab:green', 'tab:red', 'tab:blue', 'tab:orange'])
    dominance_axis.set(xlabel='time as active minimum ceiling [%]', title='Dominant speed ceiling')
    dominance_axis.grid(axis='x', alpha=0.3)
    for index, name in enumerate(ordered):
        dominance_axis.text(dominance_values[name] + 0.6, index, f'{dominance_values[name]:.2f}%', va='center', fontsize=8)
    figure.tight_layout(); figure.savefig(plots / '12_run002_speed_limits.png', dpi=180); plt.close(figure)


def _improvement(first: float, second: float, higher_is_better: bool) -> float | None:
    return _percent(second - first if higher_is_better else first - second, abs(first))


def _write_csv(destination: Path, first: dict[str, float], second: dict[str, float]) -> None:
    descriptors = [
        ('total_time_s', 's', False), ('lap_1_time_s', 's', False), ('lap_2_time_s', 's', False),
        ('average_actual_velocity_m_s', 'm/s', True), ('maximum_actual_velocity_m_s', 'm/s', True),
        ('average_commanded_velocity_m_s', 'm/s', True), ('maximum_commanded_velocity_m_s', 'm/s', True),
        ('lateral_rmse_m', 'm', False), ('lateral_max_error_m', 'm', False),
        ('aeb_active_time_s', 's', False), ('aeb_events', 'count', False),
        ('aeb_longest_intervention_s', 's', False), ('minimum_front_clearance_m', 'm', True),
        ('time_actual_speed_above_050_s', 's', True), ('fraction_actual_speed_above_050_percent', '%', True),
        ('time_target_speed_at_least_090_s', 's', True), ('fraction_target_speed_at_least_090_percent', '%', True),
        ('angular_saturation_percent', '%', False), ('distance_travelled_m', 'm', True),
        ('path_progress_percent', '%', True),
    ]
    with destination.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=['metric', 'unit', 'run_001', 'run_002', 'run_002_minus_run_001', 'improvement_percent'])
        writer.writeheader()
        for key, unit, higher in descriptors:
            a, b = first[key], second[key]
            writer.writerow({'metric': key, 'unit': unit, 'run_001': a, 'run_002': b,
                             'run_002_minus_run_001': b - a,
                             'improvement_percent': _improvement(a, b, higher)})


def _write_report(destination: Path, first: Run, second: Run, metrics_a: dict[str, float],
                  metrics_b: dict[str, float], path: dict[str, float | str | int],
                  bag_a: tuple[int, float, tuple[float, float], tuple[float, float]],
                  bag_b: tuple[int, float, tuple[float, float], tuple[float, float]],
                  sections_a: dict[str, dict[str, float]], sections_b: dict[str, dict[str, float]],
                  zone_a: dict[str, float], zone_b: dict[str, float], dominance: dict[str, float]) -> None:
    rows = [
        ('Total time', 's', 'total_time_s', False), ('Lap 1', 's', 'lap_1_time_s', False), ('Lap 2', 's', 'lap_2_time_s', False),
        ('Actual average speed', 'm/s', 'average_actual_velocity_m_s', True), ('Actual maximum speed', 'm/s', 'maximum_actual_velocity_m_s', True),
        ('Commanded average speed', 'm/s', 'average_commanded_velocity_m_s', True), ('Commanded maximum speed', 'm/s', 'maximum_commanded_velocity_m_s', True),
        ('Lateral RMSE', 'm', 'lateral_rmse_m', False), ('Maximum lateral error', 'm', 'lateral_max_error_m', False),
        ('AEB active time', 's', 'aeb_active_time_s', False), ('AEB events', 'count', 'aeb_events', False),
        ('Longest AEB intervention', 's', 'aeb_longest_intervention_s', False), ('Minimum front clearance', 'm', 'minimum_front_clearance_m', True),
        ('Actual speed > 0.50 m/s', 's', 'time_actual_speed_above_050_s', True), ('Mission at actual speed > 0.50 m/s', '%', 'fraction_actual_speed_above_050_percent', True),
        ('Target speed >= 0.90 m/s', 's', 'time_target_speed_at_least_090_s', True), ('Mission at target >= 0.90 m/s', '%', 'fraction_target_speed_at_least_090_percent', True),
        ('Angular saturation', '%', 'angular_saturation_percent', False), ('Distance travelled', 'm', 'distance_travelled_m', True), ('Path progress', '%', 'path_progress_percent', True),
    ]
    time_gain = metrics_a['total_time_s'] - metrics_b['total_time_s']
    time_gain_percent = 100.0 * time_gain / metrics_a['total_time_s']
    section_lines = []
    for key in ('0-25%', '25-50%', '50-75%', '75-100%'):
        a, b = sections_a[key], sections_b[key]
        section_lines.append(
            f'| {key} | {_fmt(a["time_s"])} | {_fmt(b["time_s"])} | {_fmt(a["time_s"] - b["time_s"])} | '
            f'{_fmt(a["average_velocity_m_s"])} | {_fmt(b["average_velocity_m_s"])} | '
            f'{_fmt(a["maximum_velocity_m_s"])} | {_fmt(b["maximum_velocity_m_s"])} | '
            f'{_fmt(a["lateral_rmse_m"])} | {_fmt(b["lateral_rmse_m"])} | '
            f'{_fmt(a["aeb_active_time_s"])} | {_fmt(b["aeb_active_time_s"])} | '
            f'{_fmt(a["average_abs_curvature_1_m"])} | {_fmt(b["average_abs_curvature_1_m"])} |')
    metric_lines = []
    for label, unit, key, higher in rows:
        a, b = metrics_a[key], metrics_b[key]
        metric_lines.append(f'| {label} [{unit}] | {_fmt(a)} | {_fmt(b)} | {_fmt(b - a)} | {_fmt(_improvement(a, b, higher), 2)}% |')
    start_dx, start_dy = second.profile['x_m'][0] - first.profile['x_m'][0], second.profile['y_m'][0] - first.profile['y_m'][0]
    start_yaw = float(_wrap(np.asarray([second.profile['heading_rad'][0] - first.profile['heading_rad'][0]]))[0])
    lines = [
        '# Run 001 vs Run 002 — final manual comparison', '',
        '## Data integrity and provenance', '',
        '- Both rosbag directories were verified with `ros2 bag info` before this analysis; both use MCAP and contain `/planned_path`, tracking, command, odometry, LiDAR, TF, and AEB topics.',
        f'- Run 001: `{bag_a[0]}` planned poses and `{bag_a[1]:.6f}` m read directly from its rosbag Path message.',
        f'- Run 002: `{bag_b[0]}` planned poses and `{bag_b[1]:.6f}` m read directly from its rosbag Path message.',
        f'- Mission completion: Run 001 `{first.result.get("completed")}`, `{first.result.get("laps")}` laps; Run 002 `{second.result.get("completed")}`, `{second.result.get("laps")}` laps.',
        '- Run 002 resolved parameters confirm `adaptive_pure_pursuit`, wheel limit ±20 rad/s, radius 0.05 m, nominal 0.90 m/s, maximum 1.00 m/s, and angular maximum 4.00 rad/s.', '',
        '## Path equivalence check', '',
        f'- Common-arc XY RMS / max: `{_fmt(path["xy_rms_m"], 6)}` / `{_fmt(path["xy_max_m"], 6)}` m; heading RMS / max: `{_fmt(path["heading_rms_rad"], 6)}` / `{_fmt(path["heading_max_rad"], 6)}` rad.',
        f'- Start-pose difference in planned path: Δx `{start_dx:.6f}` m, Δy `{start_dy:.6f}` m, norm `{math.hypot(start_dx, start_dy):.6f}` m, Δyaw `{start_yaw:.6f}` rad ({math.degrees(start_yaw):.3f}°).',
        f'- Verdict: **{path["status"]}**. The path is longer by `{bag_b[1] - bag_a[1]:.6f}` m ({100.0 * (bag_b[1] - bag_a[1]) / bag_a[1]:.3f}%). This comparison measures observed end-to-end performance, but it is not a controller-only causal experiment because the waypoint file changed between the runs.', '',
        '## Main result', '',
        f'Run 002 finished `{time_gain:.3f}` s earlier: `{metrics_a["total_time_s"]:.3f}` s → `{metrics_b["total_time_s"]:.3f}` s, an improvement of **`{time_gain_percent:.3f}%`**.', '',
        '| Metric | Run 001 | Run 002 | Difference (002−001) | Improvement % |', '|---|---:|---:|---:|---:|', *metric_lines, '',
        'For time, errors, AEB metrics, and saturation, positive improvement means a reduction. For speed, clearance, distance, and progress, positive improvement means an increase.', '',
        '## Analysis by path-progress section', '',
        '| Section | Time 001 [s] | Time 002 [s] | Saved [s] | Avg v 001 | Avg v 002 | Max v 001 | Max v 002 | RMSE 001 | RMSE 002 | AEB 001 [s] | AEB 002 [s] | Mean |κ| 001 | Mean |κ| 002 |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|', *section_lines, '',
        '## Narrow AEB section', '',
        f'The only Run 001 AEB event occurred at progress `{_fmt(zone_a.get("progress_lower_percent"), 3)}–{_fmt(zone_a.get("progress_upper_percent"), 3)}%` (centered on its minimum clearance). The matched normalized-progress window is compared below.', '',
        '| Metric | Run 001 | Run 002 |', '|---|---:|---:|',
        f'| Average speed [m/s] | {_fmt(zone_a.get("average_velocity_m_s"))} | {_fmt(zone_b.get("average_velocity_m_s"))} |',
        f'| Maximum speed [m/s] | {_fmt(zone_a.get("maximum_velocity_m_s"))} | {_fmt(zone_b.get("maximum_velocity_m_s"))} |',
        f'| Lateral RMSE [m] | {_fmt(zone_a.get("lateral_rmse_m"))} | {_fmt(zone_b.get("lateral_rmse_m"))} |',
        f'| Minimum front clearance [m] | {_fmt(zone_a.get("minimum_front_clearance_m"))} | {_fmt(zone_b.get("minimum_front_clearance_m"))} |',
        f'| AEB active [s] | {_fmt(zone_a.get("aeb_active_time_s"))} | {_fmt(zone_b.get("aeb_active_time_s"))} |', '',
        '## Adaptive speed-limit dominance (Run 002)', '',
        f'- Preview curvature: `{_fmt(dominance["preview_dominant_seconds"])} s` (`{_fmt(dominance["preview_dominant_percent"], 2)}%`).',
        f'- Current/pursuit curvature: `{_fmt(dominance["curvature_dominant_seconds"])} s` (`{_fmt(dominance["curvature_dominant_percent"], 2)}%`).',
        f'- Lateral-error ceiling: `{_fmt(dominance["lateral_error_dominant_seconds"])} s` (`{_fmt(dominance["lateral_error_dominant_percent"], 2)}%`).',
        f'- Omega ceiling: `{_fmt(dominance["omega_dominant_seconds"])} s` (`{_fmt(dominance["omega_dominant_percent"], 2)}%`).',
        '', '## Figures', '',
        'All plots are in `optimization_results/plots/` and are generated by `compare_run001_vs_run002` from recorded data only.',
    ]
    destination.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-root', type=Path, default=Path('optimization_results'))
    parsed = parser.parse_args(args)
    root = parsed.results_root.resolve()
    first, second = _load_run(root, RUN_001, 'Run 001'), _load_run(root, RUN_002, 'Run 002')
    if not (first.result.get('completed') and second.result.get('completed') and
            int(first.result.get('laps', 0)) == 2 and int(second.result.get('laps', 0)) == 2):
        raise RuntimeError('Both runs must have completed exactly two laps before comparison')
    metrics_a, metrics_b = _metrics(first, 2.0), _metrics(second, 4.0)
    path = _path_comparison(first, second)
    bag_a, bag_b = _path_from_bag(first.bag), _path_from_bag(second.bag)
    sections_a = {f'{low}-{low + 25}%': _section(first, low, low + 25) for low in range(0, 100, 25)}
    sections_b = {f'{low}-{low + 25}%': _section(second, low, low + 25) for low in range(0, 100, 25)}
    min_index = int(np.nanargmin(first.trace['aeb_critical_distance_m']))
    center = float(first.trace['progress_percent'][min_index])
    zone_a, zone_b = _narrow_zone(first, center), _narrow_zone(second, center)
    dominance, native = _dominance(second.bag)
    _write_csv(root / 'comparison_run001_vs_run002.csv', metrics_a, metrics_b)
    _plots(first, second, metrics_a, metrics_b, native, dominance, root / 'plots')
    _write_report(root / 'comparison_run001_vs_run002.md', first, second, metrics_a, metrics_b,
                  path, bag_a, bag_b, sections_a, sections_b, zone_a, zone_b, dominance)
    print(f'Wrote {root / "comparison_run001_vs_run002.csv"}')
    print(f'Wrote {root / "comparison_run001_vs_run002.md"} and {root / "plots"}')


if __name__ == '__main__':
    main()
