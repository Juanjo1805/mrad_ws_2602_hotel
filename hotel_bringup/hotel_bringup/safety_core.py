"""Validated RC TTC controller and hysteresis, owned by PC bringup."""

from dataclasses import dataclass
import math

from ybeb_2602_zulu.safety_core import clamp, CommandWatchdog
from ybeb_2602_zulu.scan_support import projected_ttc


@dataclass(frozen=True)
class AEBConfig:
    """Calibration and timing; speed bounds are in m/s, never throttle units."""

    calibration_confirmed: bool = False
    ttc_threshold: float = 0.45
    release_ttc_threshold: float = 0.65
    release_hold_time: float = 0.3
    release_scan_count: int = 3
    scan_timeout: float = 0.3
    command_timeout: float = 0.3
    footprint_radius_m: float = 0.45
    forward_speed_bound_mps: float = 2.0
    reverse_speed_bound_mps: float = 1.0
    coast_timeout: float = 1.0
    front_fov_deg: float = 180.0
    rear_fov_deg: float = 180.0
    lidar_yaw_deg: float = 0.0
    min_valid_readings: int = 5
    min_valid_fraction: float = 0.3
    max_invalid_gap_deg: float = 10.0
    min_closing_speed_mps: float = 0.01
    brake_steering_mode: str = 'hold'
    fail_safe_steering: float = 0.0

    def __post_init__(self):
        """Fail on nonfinite, incoherent or unsafe parameter combinations."""
        for name, value in vars(self).items():
            if isinstance(value, (float, int)) and not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        positive = ('ttc_threshold', 'scan_timeout', 'command_timeout',
                    'footprint_radius_m', 'forward_speed_bound_mps',
                    'reverse_speed_bound_mps', 'coast_timeout', 'release_hold_time',
                    'min_valid_readings', 'release_scan_count', 'max_invalid_gap_deg',
                    'min_closing_speed_mps')
        if any(getattr(self, name) <= 0 for name in positive):
            raise ValueError('timing, geometry, speed bounds and counts must be positive')
        if self.release_ttc_threshold <= self.ttc_threshold:
            raise ValueError('release_ttc_threshold must exceed ttc_threshold')
        if not 0 < self.min_valid_fraction <= 1:
            raise ValueError('min_valid_fraction must be in (0, 1]')
        if not all(0 < fov <= 180 for fov in (self.front_fov_deg, self.rear_fov_deg)):
            raise ValueError('directional FOV must be in (0, 180] degrees')
        if self.brake_steering_mode not in ('hold', 'center'):
            raise ValueError('brake_steering_mode must be hold or center')
        if abs(self.fail_safe_steering) > 0.5:
            raise ValueError('fail_safe_steering exceeds RC contract')


@dataclass(frozen=True)
class Decision:
    """Final bounded actuator command plus diagnostic safety state."""

    throttle: float
    steering: float
    active: bool
    reason: str
    ttc: float = math.inf


class AEBController:
    """Timer-driven AEB with expiry, hysteresis and fresh-scan release counting."""

    def __init__(self, config, limits):
        """Start fail closed until a command, valid scan and calibration exist."""
        self.config = config
        self.limits = limits
        self.watchdog = CommandWatchdog(limits, config.command_timeout)
        self.scan = None
        self.scan_deadline = -math.inf
        self.scan_error = 'no_scan'
        self.scan_sequence = 0
        self.latched = False
        self.risk_directions = set()
        self.last_motion = {}
        self.clear_since = None
        self.clear_count = 0
        self.last_clear_sequence = -1

    def update_scan(self, scan, now, valid=True, remaining=None):
        """Invalid samples immediately invalidate previously clear evidence."""
        self.scan_sequence += 1
        lifetime = self.config.scan_timeout if remaining is None else remaining
        self.scan = scan
        self.scan_error = 'ok' if valid and lifetime > 0 else 'invalid_scan'
        self.scan_deadline = now + min(self.config.scan_timeout, lifetime)

    def _stop(self, reason, command=(0.0, 0.0), ttc=math.inf):
        steering = self.config.fail_safe_steering
        if reason == 'collision' and self.config.brake_steering_mode == 'hold':
            steering = command[1]
        return Decision(0.0, clamp(steering, self.limits.steering_limit), True, reason, ttc)

    def _reset_release(self):
        self.clear_since = None
        self.clear_count = 0
        self.last_clear_sequence = -1

    def evaluate(self, now):
        """Produce a safe result on every tick even when all inputs disappear."""
        command, reason = self.watchdog.sample(now)
        fault = None
        if not self.config.calibration_confirmed:
            fault = 'calibration_required'
        elif reason != 'ok':
            fault = reason
        elif self.scan is None or self.scan_error != 'ok':
            fault = self.scan_error
        elif now >= self.scan_deadline:
            fault = 'scan_timeout'
        if fault:
            self._reset_release()
            return self._stop(fault)
        directions = {d for d, t in self.last_motion.items()
                      if now - t < self.config.coast_timeout}
        if command[0] != 0:
            directions.add(1 if command[0] > 0 else -1)
        directions.update(self.risk_directions)
        velocities = [d * (self.config.forward_speed_bound_mps if d > 0
                           else self.config.reverse_speed_bound_mps) for d in directions]
        # Still verify a usable scan at standstill; zero velocity has infinite TTC.
        if not velocities:
            velocities = [0.0]
        ttc = math.inf
        for velocity in velocities:
            value, quality = projected_ttc(self.scan, velocity, self.config)
            if quality != 'ok':
                self._reset_release()
                return self._stop(quality)
            ttc = min(ttc, value)
        if ttc <= self.config.ttc_threshold:
            self.latched = True
            self.risk_directions.update(directions)
            self._reset_release()
        elif self.latched:
            if ttc >= self.config.release_ttc_threshold:
                if self.scan_sequence != self.last_clear_sequence:
                    self.last_clear_sequence = self.scan_sequence
                    self.clear_count += 1
                    if self.clear_since is None:
                        self.clear_since = now
                if (self.clear_count >= self.config.release_scan_count
                        and now - self.clear_since >= self.config.release_hold_time):
                    self.latched = False
                    self.risk_directions.clear()
                    self._reset_release()
            else:
                self._reset_release()
        if self.latched:
            return self._stop('collision', command, ttc)
        if command[0] != 0:
            self.last_motion[1 if command[0] > 0 else -1] = now
        return Decision(*command, False, 'clear', ttc)
