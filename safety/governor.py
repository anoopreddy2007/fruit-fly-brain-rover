from dataclasses import dataclass


@dataclass
class MotionCommand:
    speed: float
    turn_rate: float


def apply_safety(
    command: MotionCommand,
    front_distance: float,
    threshold: float = 0.7,
) -> MotionCommand:
    """Override forward movement when an obstacle is too close."""
    if front_distance <= threshold:
        return MotionCommand(speed=0.0, turn_rate=1.0)

    return command
