#!/usr/bin/env python3
"""Passively record a manually launched two-lap baseline.

This program owns only its monitor and rosbag child processes.  It does not
launch, stop, reset, configure, or publish to Gazebo, AMCL, EKF, AEB, the
planner, tracker, or velocity command chain.  It is therefore safe to start in
an extra terminal before the usual manual localization / navigation sequence.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import fcntl
from pathlib import Path
import signal
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory

from .run_optimization import (
    DEFAULT_BAG_TOPICS,
    _dump_yaml,
    _format_value,
    _gazebo_pose_snapshot,
    _load_yaml,
    _parameter_dump,
    _provenance_snapshot,
    _readiness_command,
    _start,
    _stop,
    _write_failure,
)


def _next_run_id(root: Path) -> int:
    values: list[int] = []
    for directory in root.glob('run_*'):
        parts = directory.name.split('_', 2)
        if len(parts) >= 2 and parts[1].isdigit():
            values.append(int(parts[1]))
    return max(values, default=0) + 1


def _run_name(run_id: int, name: str) -> str:
    normalized = ''.join(char if char.isalnum() or char in '_-' else '_' for char in name)
    return f'run_{run_id:03d}_{normalized.strip("_") or "manual_baseline"}'


def _acquire_recorder_lock(root: Path):
    """Allow only one passive manual recorder for a physical mission.

    This is an advisory OS lock, released automatically if the recorder exits
    unexpectedly.  It protects the statistical meaning of a manual repeat
    while leaving the external ROS/Gazebo stack completely untouched.
    """
    lock_file = (root / '.manual_recorder.lock').open('w', encoding='utf-8')
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        lock_file.close()
        raise RuntimeError(
            'Another record_manual_baseline instance is already observing a mission. '
            'Do not start a second recorder for the same physical repetition.'
        ) from error
    return lock_file


def record(config: dict[str, Any], root: Path, run_id: int, name: str,
           timeout_s: float, controller: str, controller_node: str) -> bool:
    run_directory = root / _run_name(run_id, name)
    if run_directory.exists():
        raise FileExistsError(f'{run_directory} already exists; manual evidence is never overwritten.')
    run_directory.mkdir(parents=True)
    (run_directory / 'rosbag').mkdir()
    logs = run_directory / 'logs'
    mission = dict(config.get('mission', {}))
    runtime = config.get('runtime', {})
    monitor_cfg = config.get('monitor', {})
    bag_topics = config.get('bag_topics', DEFAULT_BAG_TOPICS)
    if not isinstance(bag_topics, list) or not all(isinstance(item, str) for item in bag_topics):
        raise ValueError('bag_topics must be a list of topic names')

    result_file = run_directory / 'results.yaml'
    trace_file = run_directory / 'monitor_trace.csv'
    initial_state_file = run_directory / 'initial_state.yaml'
    readiness_file = run_directory / 'readiness_localized.yaml'
    monitor_command = [
        'ros2', 'run', 'path_tracker_2602_hotel', 'optimization_monitor', '--ros-args',
        '-p', f'use_sim_time:={_format_value(monitor_cfg.get("use_sim_time", True))}',
        '-p', f'run_id:={run_id}',
        '-p', f'controller:={controller}',
        '-p', f'results_file:={result_file}',
        '-p', f'trace_file:={trace_file}',
        '-p', f'initial_state_file:={initial_state_file}',
    ]
    for key, value in monitor_cfg.items():
        if key != 'use_sim_time':
            monitor_command.extend(['-p', f'{key}:={_format_value(value)}'])
    bag_command = ['ros2', 'bag', 'record', '-o', str(run_directory / 'rosbag' / name), *bag_topics]
    readiness_command, _ = _readiness_command(runtime, 'localized', readiness_file)
    metadata = {
        'run_id': run_id,
        'name': name,
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'controller': controller,
        'mission': mission,
        'capture_mode': 'passive_manual_observation',
        'safety_note': 'AEB remains enabled and is observed; this recorder publishes no commands.',
        'external_stack_actions': [],
        'rosbag_topics': bag_topics,
        'provenance': _provenance_snapshot(config.get('provenance_files', {})),
        'manual_sequence_reconstructed_from_history': [
            'ros2 launch hotel_bringup gz_spawn.launch.py',
            'ros2 launch hotel_bringup amcl_localization.launch.py map:=.../map_nuevo.yaml',
            'publish /initialpose in map, then launch navigation_2602_hotel with the selected tracker',
        ],
    }
    _dump_yaml(run_directory / 'metadata.yaml', metadata)
    _dump_yaml(run_directory / 'parameters.yaml', {
        'controller': controller,
        'measurement_mode': 'passive_manual_observation',
        'monitor': monitor_cfg,
        'physical_limits': config.get('physical_limits', {}),
        'note': 'Controller parameters are dumped at the end when the manual tracker node is available.',
    })
    _dump_yaml(run_directory / 'rosbag_topics.yaml', {'topics': bag_topics})
    _dump_yaml(run_directory / 'commands.yaml', {
        'monitor_command': monitor_command,
        'bag_command': bag_command,
        'readiness_observer_command': readiness_command,
        'explicit_non_actions': [
            'no Gazebo launch/reset', 'no AMCL/EKF restart', 'no /initialpose publication',
            'no parameter change', 'no cmd_vel publication', 'no navigation launch',
        ],
    })

    monitor_process = None
    bag_process = None
    readiness_process = None
    passed = False
    failure = 'manual_recorder_interrupted_before_result'
    gazebo_snapshot_recorded = False
    try:
        monitor_process = _start(monitor_command, logs / 'monitor.log')
        bag_process = _start(bag_command, logs / 'rosbag.log')
        # This node only subscribes to the same health signals recorded in the
        # bag.  It neither waits on nor reconfigures the external stack.
        readiness_process = _start(readiness_command, logs / 'readiness_localized.log')
        print(f'Passive recorder active: {run_directory}', flush=True)
        print('Proceed with the normal manual startup. This recorder will not alter the ROS stack.', flush=True)
        deadline = time.monotonic() + max(1.0, timeout_s)
        while time.monotonic() < deadline:
            if readiness_file.exists() and not gazebo_snapshot_recorded:
                # Both Gazebo interfaces below are read-only.  Capturing after
                # readiness makes the physical reference contemporaneous with
                # the AMCL/TF snapshot, without publishing any input pose.
                _gazebo_pose_snapshot(runtime, run_directory / 'gazebo_ground_truth.yaml',
                                      logs / 'gazebo_pose.log')
                gazebo_snapshot_recorded = True
            if result_file.exists():
                result = _load_yaml(result_file).get('result', {})
                passed = bool(result.get('completed', False))
                failure = str(result.get('reason', 'not_completed'))
                break
            if monitor_process.poll() is not None:
                failure = 'monitor_process_exited_before_result'
                break
            time.sleep(0.25)
        else:
            failure = 'manual_capture_timeout_waiting_for_result'
        _parameter_dump(controller_node, run_directory / 'resolved_controller_parameters.yaml')
        if not result_file.exists():
            _write_failure(result_file, {'run_id': run_id, 'controller': controller,
                                         'expected_laps': int(monitor_cfg.get('expected_laps', 2))}, failure)
    finally:
        # Do not stop anything except this recorder's own observer processes.
        _stop(readiness_process)
        _stop(bag_process)
        _stop(monitor_process)
    return passed


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    default_config = Path(get_package_share_directory('path_tracker_2602_hotel')) / \
        'config' / 'optimization_baseline.yaml'
    parser.add_argument('--config', type=Path, default=default_config)
    parser.add_argument('--results-root', type=Path)
    parser.add_argument('--run-id', type=int)
    parser.add_argument('--name', default='golden_manual_baseline')
    parser.add_argument('--timeout-s', type=float, default=1100.0)
    parser.add_argument('--controller-node')
    parsed = parser.parse_args(args)
    config = _load_yaml(parsed.config.resolve())
    root = parsed.results_root or Path(config.get('results_root', 'optimization_results'))
    if not root.is_absolute():
        root = Path.cwd() / root
    root.mkdir(parents=True, exist_ok=True)
    run_id = _next_run_id(root) if parsed.run_id is None else parsed.run_id
    controller = str(config.get('controller', 'pure_pursuit_standard'))
    default_nodes = {
        'pure_pursuit_standard': '/pure_pursuit_pt_2602_hotel',
        'adaptive_pure_pursuit': '/adaptive_pure_pursuit',
    }
    controller_node = parsed.controller_node or str(
        config.get('controller_node', default_nodes.get(controller, '')))
    previous_sigint = signal.signal(signal.SIGINT, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    recorder_lock = None
    try:
        recorder_lock = _acquire_recorder_lock(root)
        success = record(
            config, root, run_id, parsed.name, parsed.timeout_s, controller, controller_node)
        raise SystemExit(0 if success else 1)
    except KeyboardInterrupt:
        print('Manual recorder interrupted; it did not modify the external stack.', flush=True)
        raise SystemExit(130)
    finally:
        if recorder_lock is not None:
            fcntl.flock(recorder_lock.fileno(), fcntl.LOCK_UN)
            recorder_lock.close()
        signal.signal(signal.SIGINT, previous_sigint)


if __name__ == '__main__':
    main()
