"""Validated RC wall geometry, with normalized and bounded commands."""

from dataclasses import dataclass
import math

from ybeb_2602_zulu.safety_core import clamp, Limits
from ybeb_2602_zulu.scan_support import projected_ttc, ScanQuality


@dataclass(frozen=True)
class WallConfig:
    """Conservative normalized commands and untuned geometric parameters."""

    throttle: float = 0.08
    steering_limit: float = 0.25
    steering_gain: float = 0.7
    steering_sign: float = 1.0
    lidar_yaw_deg: float = 0.0
    desired_distance: float = 0.72
    theta_deg: float = 48.0
    lookahead_dist: float = 0.3

    def __post_init__(self):
        """Reject unsafe scales and undefined geometry before node startup."""
        Limits(self.throttle, self.steering_limit)
        numeric = [v for v in vars(self).values() if isinstance(v, (int, float))]
        if not all(math.isfinite(v) for v in numeric):
            raise ValueError('all reactive parameters must be finite')
        if self.steering_sign not in (-1.0, 1.0):
            raise ValueError('steering_sign must be -1 or 1 after physical verification')
        if not 0 < self.theta_deg < 90:
            raise ValueError('invalid reactive angular geometry')
        if min(self.steering_gain, self.desired_distance, self.lookahead_dist) <= 0:
            raise ValueError('reactive gains and distances must be positive')


def wall_command(scan, config):
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

    # Hotel right-wall geometry: b at -90 degrees, a at -90 + theta.
    def sample(target):
        angle, value = min(rays, key=lambda ray: abs(ray[0] - target))
        return value if abs(angle - target) <= 1.5 * abs(scan.angle_increment) else None

    theta = math.radians(config.theta_deg)
    a, b = sample(-math.pi / 2 + theta), sample(-math.pi / 2)
    if a is None or b is None:
        return 0.0, 0.0
    alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
    future = b * math.cos(alpha) + config.lookahead_dist * math.sin(alpha)
    error = config.desired_distance - future
    steering = clamp(config.steering_sign * config.steering_gain * error,
                     config.steering_limit)
    return config.throttle, steering
