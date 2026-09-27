"""Shared LaserScan value object, quality gate and approaching-ray projection."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ScanQuality:
    """Default scan quality envelope used by the optional RC sources."""

    footprint_radius_m: float = 0.45
    front_fov_deg: float = 180.0
    rear_fov_deg: float = 180.0
    lidar_yaw_deg: float = 0.0
    min_valid_readings: int = 5
    min_valid_fraction: float = 0.3
    max_invalid_gap_deg: float = 10.0
    min_closing_speed_mps: float = 0.01


@dataclass(frozen=True)
class Scan:
    """LaserScan geometry copied into a ROS-independent value object."""

    ranges: tuple
    angle_min: float
    angle_increment: float
    angle_max: float
    range_min: float
    range_max: float


def projected_ttc(scan, velocity, config):
    """
    Return (TTC, quality reason) for one direction of longitudinal travel.

    TTC = max(0, range - footprint_radius) / (velocity * cos(body_angle)).
    Only approaching rays participate. The retained Hotel circular envelope
    is conservative; steering is not misinterpreted as an angular velocity.
    Positive infinity is unknown/no return, not evidence of a clear corridor.
    """
    metadata = (scan.angle_min, scan.angle_increment, scan.angle_max,
                scan.range_min, scan.range_max)
    if (not scan.ranges or not all(math.isfinite(v) for v in metadata)
            or scan.angle_increment == 0 or scan.range_min < 0
            or scan.range_max <= scan.range_min):
        return math.inf, 'invalid_scan_metadata'
    step = abs(scan.angle_increment)
    expected = scan.angle_min + (len(scan.ranges) - 1) * scan.angle_increment
    if abs(expected - scan.angle_max) > max(1e-5, 1.5 * step):
        return math.inf, 'invalid_scan_angles'
    direction = 1 if velocity >= 0 else -1
    center = 0.0 if direction > 0 else math.pi
    half = math.radians(config.front_fov_deg if direction > 0 else config.rear_fov_deg) / 2
    yaw = math.radians(config.lidar_yaw_deg)
    rays = []
    for i, distance in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment + yaw
        offset = math.atan2(math.sin(angle - center), math.cos(angle - center))
        if abs(offset) <= half + 1e-8:
            valid = math.isfinite(distance) and scan.range_min <= distance <= scan.range_max
            rays.append((offset, distance, angle, valid))
    rays.sort()
    valid_count = sum(ray[3] for ray in rays)
    if (valid_count < config.min_valid_readings or not rays
            or valid_count / len(rays) < config.min_valid_fraction):
        return math.inf, 'insufficient_scan_returns'
    if rays[0][0] > -half + 1.5 * step or rays[-1][0] < half - 1.5 * step:
        return math.inf, 'incomplete_scan_fov'
    gap = 0.0
    best = math.inf
    for _, distance, angle, valid in rays:
        if not valid:
            gap += step
            if gap > math.radians(config.max_invalid_gap_deg):
                return math.inf, 'scan_blind_sector'
            continue
        gap = 0.0
        closing = velocity * math.cos(angle)
        if closing > config.min_closing_speed_mps:
            best = min(best, max(0.0, distance - config.footprint_radius_m) / closing)
    return best, 'ok'
