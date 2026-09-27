"""Shared RC command contract and the independent hardware watchdog; no I/O."""

from dataclasses import dataclass
import math


def clamp(value, limit):
    """Clamp a finite scalar symmetrically."""
    return max(-limit, min(limit, value))


@dataclass(frozen=True)
class Limits:
    """Configurable limits within the immutable RC actuator contract."""

    throttle_limit: float = 0.4
    steering_limit: float = 0.5

    def __post_init__(self):
        """Reject configurations that could exceed the hardware contract."""
        for value, maximum in ((self.throttle_limit, 0.4), (self.steering_limit, 0.5)):
            if not math.isfinite(value) or not 0.0 < value <= maximum:
                raise ValueError('RC limits must be finite, positive and within 0.4 / 0.5')


class CommandWatchdog:
    """Independent monotonic command expiry and fail-closed scalar validation."""

    def __init__(self, limits, timeout):
        """Start at neutral, without a valid command."""
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('command timeout must be positive and finite')
        self.limits = limits
        self.timeout = timeout
        self.command = (0.0, 0.0)
        self.deadline = -math.inf
        self.reason = 'no_command'

    def update(self, throttle, steering, now, valid=True, remaining=None):
        """Latch invalid inputs to neutral; valid oversized inputs are saturated."""
        if not valid or not all(math.isfinite(x) for x in (throttle, steering, now)):
            self.command = (0.0, 0.0)
            self.deadline = -math.inf
            self.reason = 'invalid_command'
            return
        lifetime = self.timeout if remaining is None else min(self.timeout, remaining)
        if not math.isfinite(lifetime) or lifetime <= 0:
            self.update(0.0, 0.0, now, valid=False)
            return
        self.command = (clamp(throttle, self.limits.throttle_limit),
                        clamp(steering, self.limits.steering_limit))
        self.deadline = now + lifetime
        self.reason = 'ok'

    def sample(self, now):
        """Return neutral once expired, including before the first command."""
        if now >= self.deadline:
            reason = 'command_timeout' if self.reason == 'ok' else self.reason
            return (0.0, 0.0), reason
        return self.command, 'ok'


def servo_values(throttle, steering, limits):
    """Preserve the original PWM calibration; never pass NaN/inf to Rosmaster."""
    if not all(math.isfinite(v) for v in (throttle, steering)):
        throttle, steering = 0.0, 0.0
    throttle = clamp(throttle, limits.throttle_limit)
    steering = clamp(steering, limits.steering_limit)
    return 91.0 + 27.0 * throttle, 127.5 + 105.0 * steering
