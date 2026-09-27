"""Validated RC gap geometry, with normalized and bounded commands."""

from dataclasses import dataclass
import math

from ybeb_2602_zulu.safety_core import clamp, Limits
from ybeb_2602_zulu.scan_support import projected_ttc, ScanQuality


@dataclass(frozen=True)
class GapConfig:
    """Conservative normalized commands and untuned geometric parameters."""

    throttle: float = 0.08
    steering_limit: float = 0.25
    steering_gain: float = 0.7
    steering_sign: float = 1.0
    lidar_yaw_deg: float = 0.0
    gap_fov_deg: float = 120.0
    gap_min_depth: float = 1.5
    gap_min_width_deg: float = 10.0
    bubble_radius: float = 0.3

    def __post_init__(self):
        """Reject unsafe scales and undefined geometry before node startup."""
        Limits(self.throttle, self.steering_limit)
        numeric = [v for v in vars(self).values() if isinstance(v, (int, float))]
        if not all(math.isfinite(v) for v in numeric):
            raise ValueError('all reactive parameters must be finite')
        if self.steering_sign not in (-1.0, 1.0):
            raise ValueError('steering_sign must be -1 or 1 after physical verification')
        if not (0 < self.gap_fov_deg <= 180
                and 0 < self.gap_min_width_deg < self.gap_fov_deg):
            raise ValueError('invalid reactive angular geometry')
        if min(self.steering_gain, self.gap_min_depth, self.bubble_radius) <= 0:
            raise ValueError('reactive gains and distances must be positive')


def gap_command(scan, config):
    """Return bounded normalized throttle/steering, or neutral if geometry fails."""
    quality = ScanQuality(lidar_yaw_deg=config.lidar_yaw_deg)
    if projected_ttc(scan, 0.0, quality)[1] != 'ok':
        return 0.0, 0.0
    rays = []
    for i, distance in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment + math.radians(config.lidar_yaw_deg)
        angle = math.atan2(math.sin(angle), math.cos(angle))
        valid = math.isfinite(distance) and scan.range_min <= distance <= scan.range_max
        rays.append((angle, distance if valid else None))
    rays.sort()
    half = math.radians(config.gap_fov_deg) / 2
    front = [(angle, distance) for angle, distance in rays if abs(angle) <= half + 1e-8]
    finite = [(angle, distance) for angle, distance in front if distance is not None]
    if not finite:
        return 0.0, 0.0
    nearest_angle, nearest = min(finite, key=lambda ray: ray[1])
    bubble = math.atan2(config.bubble_radius, max(nearest, 1e-3))
    gaps, current = [], []
    for angle, distance in front:
        blocked = nearest < config.gap_min_depth and abs(angle - nearest_angle) <= bubble
        if distance is not None and distance >= config.gap_min_depth and not blocked:
            current.append((angle, distance))
        elif current:
            gaps.append(current)
            current = []
    if current:
        gaps.append(current)
    gaps = [gap for gap in gaps if gap[-1][0] - gap[0][0] >=
            math.radians(config.gap_min_width_deg)]
    if not gaps:
        return 0.0, 0.0
    best = max(gaps, key=lambda gap: (
        (gap[-1][0] - gap[0][0]) * min(d for _, d in gap),
        -abs((gap[0][0] + gap[-1][0]) / 2)))
    error = (best[0][0] + best[-1][0]) / 2
    steering = clamp(config.steering_sign * config.steering_gain * error,
                     config.steering_limit)
    return config.throttle, steering
