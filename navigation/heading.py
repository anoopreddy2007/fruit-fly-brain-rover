import math


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def update_heading(
    heading: float,
    left_speed: float,
    right_speed: float,
    wheel_base: float,
    dt: float,
) -> float:
    """Estimate heading using differential-drive wheel speeds."""
    if wheel_base <= 0 or dt < 0:
        raise ValueError("wheel_base must be positive and dt non-negative")

    angular_velocity = (right_speed - left_speed) / wheel_base
    return normalize_angle(heading + angular_velocity * dt)
