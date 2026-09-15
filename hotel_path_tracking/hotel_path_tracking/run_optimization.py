#!/usr/bin/env python3
"""
Run reproducible ROS 2 path-tracking experiments with one rosbag per run.

The runner restarts the simulator/localization stack for every candidate by
default.  This avoids the common but invalid comparison where later candidates
start from the previous run's final robot pose.  Commands live in YAML so the
world, map and reset protocol are explicit evidence, not shell history.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time
from typing import Any
import re

import yaml
from ament_index_python.packages import get_package_share_directory


DEFAULT_BAG_TOPICS = [
    '/planned_path',
    '/path_tracking/executed_path',
    '/path_tracking/closest_point',
    '/path_tracking/lookahead_point',
    '/path_tracking/goal',
    '/path_tracking/current_speed',
    '/path_tracking/target_speed',
    '/path_tracking/speed_limit_curvature',
    '/path_tracking/speed_limit_preview',
    '/path_tracking/speed_limit_omega',
    '/path_tracking/speed_limit_lateral_error',
    '/path_tracking/lookahead_distance',
    '/path_tracking/curvature',
    '/path_tracking/future_curvature',
    '/path_tracking/lateral_error',
    '/path_tracking/heading_error',
    '/path_tracking/path_progress',
    '/path_tracking/lap',
    '/path_tracking/closest_index',
    '/path_tracking/target_index',
    '/path_tracking/mission_status',
    '/cmd_vel_nav',
    '/cmd_vel_mux',
    '/diffdrive_controller/cmd_vel',
    '/diffdrive_controller/odom',
    '/ekf/odometry',
    '/amcl_pose',
    '/initialpose',
    '/scan',
    '/imu',
    '/tf',
    '/tf_static',
    '/aeb/active',
    '/aeb/command_blocked',
    '/aeb/ttc',
    '/aeb/critical_distance',
    '/lidar/d_min',
    '/lidar/vctrl',
    '/dist_min',
]


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding='utf-8') as stream:
        value = yaml.safe_load(stream) or {}
    if not isinstance(value, dict):
        raise ValueError(f'{path} must contain a YAML mapping')
    return value


def _dump_yaml(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(value, stream, sort_keys=False)


def _command(value: str | list[str], extra: list[str] | None = None) -> list[str]:
    command = shlex.split(value) if isinstance(value, str) else [str(item) for item in value]
    return command + (extra or [])


def _start(command: list[str], log: Path) -> subprocess.Popen:
    log.parent.mkdir(parents=True, exist_ok=True)
    stream = log.open('w', encoding='utf-8')
    environment = os.environ.copy()
    # Keep launch logs beside the experimental evidence.  It also makes the
    # runner usable in restricted/containerized environments with no writable
    # ~/.ros while preserving the command's ROS environment.
    ros_log_dir = log.parent / 'ros'
    ros_log_dir.mkdir(exist_ok=True)
    environment['ROS_LOG_DIR'] = str(ros_log_dir)
    return subprocess.Popen(
        command,
        stdout=stream,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        env=environment,
    )


def _run_once(command: list[str], log: Path, timeout_s: float = 20.0) -> bool:
    """Run a short setup action (currently AMCL initial pose) with a run log."""
    log.parent.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    ros_log_dir = log.parent / 'ros'
    ros_log_dir.mkdir(exist_ok=True)
    environment['ROS_LOG_DIR'] = str(ros_log_dir)
    with log.open('w', encoding='utf-8') as stream:
        try:
            return subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT,
                                  timeout=timeout_s, check=False, env=environment).returncode == 0
        except (OSError, subprocess.TimeoutExpired):
            return False


def _controller_active(controller_name: str, log: Path) -> bool:
    """Read controller-manager state before a run is allowed to start."""
    environment = os.environ.copy()
    ros_log_dir = log.parent / 'ros'
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    environment['ROS_LOG_DIR'] = str(ros_log_dir)
    command = [
        'ros2', 'service', 'call', '/controller_manager/list_controllers',
        'controller_manager_msgs/srv/ListControllers', '{}',
    ]
    try:
        completed = subprocess.run(
            command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            timeout=5.0, check=False, env=environment,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    with log.open('a', encoding='utf-8') as stream:
        stream.write('\n$ ' + ' '.join(command) + '\n' + completed.stdout)
    active_state = 'state: active' in completed.stdout or "state='active'" in completed.stdout
    return completed.returncode == 0 and controller_name in completed.stdout and active_state


def _lifecycle_active(node_name: str, log: Path) -> bool:
    """Verify the AMCL lifecycle state without changing the node."""
    command = ['ros2', 'lifecycle', 'get', node_name]
    try:
        completed = subprocess.run(
            command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            timeout=5.0, check=False, env=os.environ.copy(),
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    with log.open('a', encoding='utf-8') as stream:
        stream.write('\n$ ' + ' '.join(command) + '\n' + completed.stdout)
    return completed.returncode == 0 and re.search(r'\bactive\b', completed.stdout, re.IGNORECASE) is not None


def _readiness_command(runtime: dict[str, Any], phase: str, output_file: Path) -> tuple[list[str], float]:
    """Build an observer-only readiness command and its wall-clock timeout."""
    readiness = runtime.get('readiness', {})
    if not isinstance(readiness, dict):
        readiness = {}
    timeout_key = 'bootstrap_timeout_s' if phase == 'bootstrap' else 'localized_timeout_s'
    timeout_s = max(5.0, float(readiness.get(timeout_key, 75.0)))
    command = [
        'ros2', 'run', 'path_tracker_2602_hotel', 'system_readiness', '--ros-args',
        '-p', 'use_sim_time:=true',
        '-p', f'phase:={phase}',
        '-p', f'output_file:={output_file}',
        '-p', f'timeout_s:={timeout_s}',
    ]
    allowed = {
        'stable_window_s', 'sample_frequency_hz', 'odom_topic', 'scan_topic',
        'amcl_pose_topic', 'initial_pose_topic', 'map_frame', 'odom_frame', 'base_frame',
        'minimum_odom_samples', 'minimum_scan_samples', 'minimum_tf_samples',
        'minimum_amcl_samples', 'minimum_odom_rate_hz', 'minimum_scan_rate_hz',
        'maximum_amcl_x_covariance_m2', 'maximum_amcl_y_covariance_m2',
        'maximum_amcl_yaw_covariance_rad2',
    }
    for key, value in readiness.items():
        if key in allowed:
            command.extend(['-p', f'{key}:={_format_value(value)}'])
    return command, timeout_s


def _readiness_passed(runtime: dict[str, Any], phase: str, output_file: Path, log: Path) -> bool:
    command, timeout_s = _readiness_command(runtime, phase, output_file)
    if not _run_once(command, log, timeout_s=timeout_s + 10.0):
        return False
    try:
        result = _load_yaml(output_file)
    except (OSError, ValueError, yaml.YAMLError):
        return False
    return bool(result.get('ready', False))


def _readiness_file_passed(output_file: Path) -> bool:
    try:
        result = _load_yaml(output_file)
    except (OSError, ValueError, yaml.YAMLError):
        return False
    return bool(result.get('ready', False))


def _gazebo_pose_snapshot(runtime: dict[str, Any], destination: Path, log: Path) -> None:
    """Capture physical Gazebo pose using only read-only Gazebo interfaces.

    Some Gazebo worlds expose ``/get_pose`` but do not answer it while the
    simulation is starting.  Their standard pose-info topic is nevertheless
    available, so use it as a documented fallback rather than treating the
    requested spawn pose as a measured ground-truth pose.
    """
    world_name = str(runtime.get('gazebo_world_name', 'empty_world'))
    model_name = str(runtime.get('gazebo_model_name', 'diffbot'))
    command = [
        'gz', 'service', '-s', f'/world/{world_name}/get_pose',
        '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Pose', '--timeout', '1500',
        '--req', f'name: "{model_name}"',
    ]
    content: dict[str, Any] = {
        'status': 'unavailable', 'world_name': world_name, 'model_name': model_name,
        'command': command,
    }
    try:
        completed = subprocess.run(
            command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            timeout=3.0, check=False, env=os.environ.copy(),
        )
        output = completed.stdout
        with log.open('a', encoding='utf-8') as stream:
            stream.write('\n$ ' + ' '.join(command) + '\n' + output)
        content['raw_response'] = output
        if completed.returncode == 0:
            def number(field: str) -> float | None:
                match = re.search(rf'\b{field}:\s*([-+0-9.eE]+)', output)
                return None if match is None else float(match.group(1))
            x, y = number('x'), number('y')
            # Gazebo represents orientation in a nested quaternion; retain raw
            # response if its exact schema changes instead of guessing a yaw.
            z_match = re.search(r'orientation\s*\{[^}]*\bz:\s*([-+0-9.eE]+)', output, re.DOTALL)
            w_match = re.search(r'orientation\s*\{[^}]*\bw:\s*([-+0-9.eE]+)', output, re.DOTALL)
            content['status'] = 'available' if x is not None and y is not None else 'response_unparsed'
            if x is not None and y is not None:
                content.update({'x_m': x, 'y_m': y})
            if z_match is not None and w_match is not None:
                z, w = float(z_match.group(1)), float(w_match.group(1))
                content['yaw_rad'] = 2.0 * math.atan2(z, w)
    except (OSError, subprocess.TimeoutExpired, ValueError) as error:
        content['error'] = str(error)

    if content.get('status') != 'available':
        topic = f'/world/{world_name}/pose/info'
        topic_command = ['gz', 'topic', '-e', '-t', topic, '-n', '1']
        content['topic_fallback_command'] = topic_command
        try:
            completed = subprocess.run(
                topic_command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                timeout=6.0, check=False, env=os.environ.copy(),
            )
            output = completed.stdout
            with log.open('a', encoding='utf-8') as stream:
                stream.write('\n$ ' + ' '.join(topic_command) + '\n' + output)
            content['topic_fallback_raw_response'] = output

            lines = output.splitlines()
            target_index = next((index for index, line in enumerate(lines)
                                 if f'name: "{model_name}"' in line), None)
            if target_index is not None:
                start_index = next((index for index in range(target_index, -1, -1)
                                    if lines[index].strip() == 'pose {'), None)
                if start_index is not None:
                    depth = 0
                    end_index = None
                    for index in range(start_index, len(lines)):
                        depth += lines[index].count('{') - lines[index].count('}')
                        if depth == 0:
                            end_index = index
                            break
                    block = '\n'.join(lines[start_index:(end_index or len(lines)) + 1])
                    position = re.search(r'position\s*\{([^{}]*)\}', block, re.DOTALL)
                    orientation = re.search(r'orientation\s*\{([^{}]*)\}', block, re.DOTALL)

                    def field(section: re.Match[str] | None, key: str) -> float | None:
                        if section is None:
                            return None
                        match = re.search(rf'\b{key}:\s*([-+0-9.eE]+)', section.group(1))
                        return None if match is None else float(match.group(1))

                    x, y, z = field(position, 'x'), field(position, 'y'), field(position, 'z')
                    qx, qy = field(orientation, 'x'), field(orientation, 'y')
                    qz, qw = field(orientation, 'z'), field(orientation, 'w')
                    if None not in (x, y, z, qx, qy, qz, qw):
                        content.update({
                            'status': 'available_topic_fallback',
                            'source_topic': topic,
                            'x_m': x, 'y_m': y, 'z_m': z,
                            'quaternion': {'x': qx, 'y': qy, 'z': qz, 'w': qw},
                            'yaw_rad': math.atan2(2.0 * (qw * qz + qx * qy),
                                                  1.0 - 2.0 * (qy * qy + qz * qz)),
                        })
        except (OSError, subprocess.TimeoutExpired, ValueError) as error:
            content['topic_fallback_error'] = str(error)
    _dump_yaml(destination, content)


def _stop(process: subprocess.Popen | None, timeout_s: float = 12.0) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=timeout_s)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=4.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def _format_value(value: Any) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def _run_name(candidate: dict[str, Any]) -> str:
    run_id = int(candidate['run_id'])
    label = ''.join(char if char.isalnum() or char in '_-' else '_' for char in str(candidate['name']))
    return f'run_{run_id:03d}_{label.strip("_") or "candidate"}'


def _write_failure(results_file: Path, candidate: dict[str, Any], reason: str) -> None:
    _dump_yaml(results_file, {
        'run_id': int(candidate['run_id']),
        'controller': candidate.get('controller', 'unknown'),
        'result': {
            'completed': False,
            'reason': reason,
            'laps': 0,
            'expected_laps': int(candidate.get('expected_laps', 2)),
        },
    })


def _parameter_dump(node_name: str, destination: Path) -> None:
    if not node_name:
        return
    try:
        completed = subprocess.run(
            ['ros2', 'param', 'dump', node_name],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            timeout=12.0, check=False, env=os.environ.copy(),
        )
    except (OSError, subprocess.TimeoutExpired):
        return
    if completed.returncode == 0 and completed.stdout.strip():
        destination.write_text(completed.stdout, encoding='utf-8')


def _provenance_snapshot(files: Any) -> dict[str, dict[str, str | bool]]:
    """Fingerprint fixed experimental inputs without copying mutable source trees."""
    if not isinstance(files, dict):
        return {}
    snapshot: dict[str, dict[str, str | bool]] = {}
    for label, value in files.items():
        path = Path(str(value)).expanduser().resolve()
        item: dict[str, str | bool] = {'path': str(path), 'exists': path.is_file()}
        if path.is_file():
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                    digest.update(chunk)
            item['sha256'] = digest.hexdigest()
        snapshot[str(label)] = item
    return snapshot


def _candidate_launch_args(candidate: dict[str, Any], mission: dict[str, Any]) -> list[str]:
    output = [f'{key}:={_format_value(value)}' for key, value in mission.items()]
    output.extend(f'{key}:={_format_value(value)}' for key, value in candidate.get('launch_args', {}).items())
    output.extend(f'{key}:={_format_value(value)}' for key, value in candidate.get('parameters', {}).items())
    return output


def run_candidate(config: dict[str, Any], candidate: dict[str, Any], root: Path, dry_run: bool) -> bool:
    run_directory = root / _run_name(candidate)
    if run_directory.exists():
        raise FileExistsError(
            f'{run_directory} already exists. Refusing to overwrite an experimental record.')
    run_directory.mkdir(parents=True)
    (run_directory / 'rosbag').mkdir()
    logs = run_directory / 'logs'
    runtime = config.get('runtime', {})
    mission = config.get('mission', {})
    monitor_cfg = config.get('monitor', {})
    bag_topics = config.get('bag_topics', DEFAULT_BAG_TOPICS)
    if not isinstance(bag_topics, list) or not all(isinstance(item, str) for item in bag_topics):
        raise ValueError('bag_topics must be a list of topic names')
    metadata = {
        'run_id': int(candidate['run_id']),
        'name': candidate['name'],
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'controller': candidate.get('controller', 'standard'),
        'mission': mission,
        'notes': candidate.get('notes', []),
        'rosbag_topics': bag_topics,
        'reset_policy': runtime.get('reset_policy', 'restart_stack_per_run'),
        'initial_pose': runtime.get('initial_pose', {}),
        'common_start_commands': runtime.get('common_start_commands', []),
        'navigation_command': runtime.get('navigation_command', ''),
        'readiness_protocol': runtime.get('readiness', {}),
        'provenance': _provenance_snapshot(config.get('provenance_files', {})),
    }
    _dump_yaml(run_directory / 'metadata.yaml', metadata)
    _dump_yaml(run_directory / 'parameters.yaml', {
        'controller': candidate.get('controller', 'standard'),
        'launch_args': candidate.get('launch_args', {}),
        'parameters': candidate.get('parameters', {}),
        'monitor': monitor_cfg,
        'physical_limits': config.get('physical_limits', {}),
    })
    _dump_yaml(run_directory / 'rosbag_topics.yaml', {'topics': bag_topics})

    common_commands = runtime.get('common_start_commands', [])
    navigation_command = runtime.get('navigation_command')
    if not navigation_command:
        raise ValueError('runtime.navigation_command is required')
    launch_command = _command(navigation_command, _candidate_launch_args(candidate, mission))
    result_file = run_directory / 'results.yaml'
    trace_file = run_directory / 'monitor_trace.csv'
    initial_state_file = run_directory / 'initial_state.yaml'
    monitor_command = [
        'ros2', 'run', 'path_tracker_2602_hotel', 'optimization_monitor', '--ros-args',
        '-p', f'use_sim_time:={_format_value(monitor_cfg.get("use_sim_time", True))}',
        '-p', f'run_id:={int(candidate["run_id"])}',
        '-p', f'controller:={candidate.get("controller", "standard")}',
        '-p', f'results_file:={result_file}',
        '-p', f'trace_file:={trace_file}',
        '-p', f'initial_state_file:={initial_state_file}',
    ]
    merged_monitor = {**monitor_cfg, **candidate.get('monitor_parameters', {})}
    for key, value in merged_monitor.items():
        if key != 'use_sim_time':
            monitor_command.extend(['-p', f'{key}:={_format_value(value)}'])
    bag_command = ['ros2', 'bag', 'record', '-o', str(run_directory / 'rosbag' / candidate['name']), *bag_topics]
    manifest = {
        'common_start_commands': [_command(value) for value in common_commands],
        'navigation_command': launch_command,
        'monitor_command': monitor_command,
        'bag_command': bag_command,
        'readiness_bootstrap_command': _readiness_command(
            runtime, 'bootstrap', run_directory / 'readiness_bootstrap.yaml')[0],
        'readiness_localized_command': _readiness_command(
            runtime, 'localized', run_directory / 'readiness_localized.yaml')[0],
    }
    _dump_yaml(run_directory / 'commands.yaml', manifest)
    if dry_run:
        print(f'[dry-run] {run_directory.name}')
        return True

    common_processes: list[subprocess.Popen] = []
    monitor_process: subprocess.Popen | None = None
    bag_process: subprocess.Popen | None = None
    localized_readiness_process: subprocess.Popen | None = None
    navigation_process: subprocess.Popen | None = None
    passed = False
    failure = 'runner_interrupted'
    startup_wall = time.monotonic()
    timeline: list[dict[str, Any]] = []

    def mark(event: str, **details: Any) -> None:
        timeline.append({
            'event': event,
            'elapsed_wall_s': time.monotonic() - startup_wall,
            **details,
        })

    try:
        for index, command in enumerate(common_commands):
            common_processes.append(_start(_command(command), logs / f'common_{index + 1}.log'))
        mark('common_stack_started')
        required_controller = str(runtime.get('required_controller', '')).strip()
        if required_controller:
            controller_deadline = time.monotonic() + max(
                1.0, float(runtime.get('controller_ready_timeout_s', 60.0)))
            controller_ready = False
            while time.monotonic() < controller_deadline:
                if _controller_active(required_controller, logs / 'controller_readiness.log'):
                    controller_ready = True
                    break
                if any(process.poll() is not None for process in common_processes):
                    break
                time.sleep(0.5)
            if not controller_ready:
                failure = f'required_controller_not_active:{required_controller}'
                _write_failure(result_file, candidate, failure)
                return False
            mark('diffdrive_controller_active', controller=required_controller)

        # Start observation before initialpose and before navigation.  These
        # processes subscribe only; they never reset, move, or configure the
        # simulation.  Starting them here preserves volatile /initialpose in
        # the automatic bag just as the manual recorder does.
        monitor_process = _start(monitor_command, logs / 'monitor.log')
        bag_process = _start(bag_command, logs / 'rosbag.log')
        mark('monitor_and_rosbag_started')

        bootstrap_file = run_directory / 'readiness_bootstrap.yaml'
        if not _readiness_passed(runtime, 'bootstrap', bootstrap_file, logs / 'readiness_bootstrap.log'):
            failure = 'bootstrap_readiness_failed'
            _write_failure(result_file, candidate, failure)
            return False
        mark('bootstrap_readiness_passed', snapshot=str(bootstrap_file))

        amcl_node = str(runtime.get('amcl_node', '/amcl'))
        amcl_deadline = time.monotonic() + max(1.0, float(runtime.get('amcl_ready_timeout_s', 45.0)))
        while time.monotonic() < amcl_deadline:
            if _lifecycle_active(amcl_node, logs / 'amcl_readiness.log'):
                mark('amcl_lifecycle_active', node=amcl_node)
                break
            if any(process.poll() is not None for process in common_processes):
                break
            time.sleep(0.5)
        else:
            failure = f'amcl_not_active:{amcl_node}'
            _write_failure(result_file, candidate, failure)
            return False
        if not timeline or timeline[-1].get('event') != 'amcl_lifecycle_active':
            failure = f'amcl_not_active:{amcl_node}'
            _write_failure(result_file, candidate, failure)
            return False

        localized_file = run_directory / 'readiness_localized.yaml'
        localized_command, localized_timeout_s = _readiness_command(runtime, 'localized', localized_file)
        # AMCL pose and /initialpose are volatile.  The local readiness
        # observer must therefore subscribe *before* the runner publishes the
        # initial pose that prompts AMCL's first scan update.
        localized_readiness_process = _start(localized_command, logs / 'readiness_localized.log')
        mark('localized_readiness_observer_started', snapshot=str(localized_file))

        initial_pose = runtime.get('initial_pose')
        if runtime.get('reset_policy', 'restart_stack_per_run') == 'restart_stack_per_run':
            if not isinstance(initial_pose, dict):
                failure = 'initial_pose_not_configured'
                _write_failure(result_file, candidate, failure)
                return False
            required = ('x', 'y', 'yaw')
            if any(key not in initial_pose for key in required):
                failure = 'initial_pose_missing_x_y_or_yaw'
                _write_failure(result_file, candidate, failure)
                return False
            pose_command = [
                'ros2', 'run', 'path_tracker_2602_hotel', 'publish_initial_pose', '--ros-args',
                '-p', f'x:={_format_value(initial_pose["x"])}',
                '-p', f'y:={_format_value(initial_pose["y"])}',
                '-p', f'yaw:={_format_value(initial_pose["yaw"])}',
                '-p', f'frame_id:={_format_value(initial_pose.get("frame_id", "map"))}',
                # AMCL, the localized observer and the monitor / recorder
                # should be connected before any volatile pose is sent.
                '-p', 'minimum_subscribers:=3',
                '-p', 'subscriber_wait_timeout_s:=8.0',
            ]
            if not _run_once(pose_command, logs / 'initial_pose.log', timeout_s=20.0):
                failure = 'initial_pose_publication_failed'
                _write_failure(result_file, candidate, failure)
                return False
            mark('initialpose_published', pose=initial_pose)

        try:
            localized_returncode = localized_readiness_process.wait(timeout=localized_timeout_s + 10.0)
        except subprocess.TimeoutExpired:
            localized_returncode = None
        if localized_returncode != 0 or not _readiness_file_passed(localized_file):
            failure = 'localized_readiness_failed'
            _write_failure(result_file, candidate, failure)
            return False
        mark('localized_readiness_passed', snapshot=str(localized_file))
        _gazebo_pose_snapshot(runtime, run_directory / 'gazebo_ground_truth.yaml', logs / 'gazebo_pose.log')
        mark('gazebo_ground_truth_queried')

        navigation_process = _start(launch_command, logs / 'navigation.log')
        mark('navigation_started')
        timeout_s = float(monitor_cfg.get('maximum_duration_s', 900.0)) + float(runtime.get('start_grace_s', 90.0))
        deadline = time.monotonic() + max(1.0, timeout_s)
        while time.monotonic() < deadline:
            if result_file.exists():
                result = _load_yaml(result_file).get('result', {})
                passed = bool(result.get('completed', False))
                failure = str(result.get('reason', 'not_completed'))
                break
            if navigation_process.poll() is not None:
                failure = 'navigation_process_exited_before_monitor_result'
                break
            if monitor_process.poll() is not None:
                failure = 'monitor_process_exited_before_result'
                break
            time.sleep(0.25)
        else:
            failure = 'runner_timeout_waiting_for_result'
        _parameter_dump(str(candidate.get('controller_node', '')), run_directory / 'resolved_controller_parameters.yaml')
        if not result_file.exists():
            _write_failure(result_file, candidate, failure)
    finally:
        # A SIGINT/SIGTERM can arrive while a mission is running (for example
        # from a batch scheduler).  Preserve an explicit failed record rather
        # than leaving an orphan rosbag with no experimental outcome.
        if not result_file.exists():
            _write_failure(result_file, candidate, failure)
        _stop(navigation_process)
        _stop(bag_process)
        _stop(monitor_process)
        _stop(localized_readiness_process)
        for process in reversed(common_processes):
            _stop(process)
        _dump_yaml(run_directory / 'startup_timeline.yaml', {
            'started_utc': metadata['created_utc'],
            'events': timeline,
        })
    return passed


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    default_config = Path(get_package_share_directory('path_tracker_2602_hotel')) / \
        'config' / 'optimization_baseline.yaml'
    parser.add_argument(
        '--config', type=Path,
        default=default_config,
        help='experiment YAML (default: baseline configuration installed with the package)',
    )
    parser.add_argument('--candidate', action='append', default=[], help='run only this candidate name; repeatable')
    parser.add_argument('--dry-run', action='store_true', help='write reproducible run metadata without starting ROS')
    parsed = parser.parse_args(args)
    config = _load_yaml(parsed.config.resolve())
    root = Path(config.get('results_root', 'optimization_results')).expanduser()
    if not root.is_absolute():
        root = Path.cwd() / root
    candidates = config.get('candidates', [])
    if not isinstance(candidates, list) or not candidates:
        raise ValueError('configuration needs a non-empty candidates list')
    selected = [candidate for candidate in candidates if not parsed.candidate or candidate.get('name') in parsed.candidate]
    if not selected:
        raise ValueError('no candidates matched --candidate')
    def interrupt_handler(_signum: int, _frame: object) -> None:
        raise KeyboardInterrupt

    previous_sigint = signal.signal(signal.SIGINT, interrupt_handler)
    previous_sigterm = signal.signal(signal.SIGTERM, interrupt_handler)
    try:
        outcome = True
        for candidate in selected:
            if not isinstance(candidate, dict) or 'run_id' not in candidate or 'name' not in candidate:
                raise ValueError('each candidate needs run_id and name')
            print(f'=== Starting {_run_name(candidate)} ===', flush=True)
            outcome = run_candidate(config, candidate, root, parsed.dry_run) and outcome
        raise SystemExit(0 if outcome else 1)
    except KeyboardInterrupt:
        print('Optimization runner interrupted; current run was recorded as FAILED.', file=sys.stderr)
        raise SystemExit(130)
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == '__main__':
    main()
