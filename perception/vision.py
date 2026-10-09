from dataclasses import dataclass


@dataclass
class SensorReading:
    front_distance: float
    max_range: float = 5.0


def obstacle_detected(
    reading: SensorReading,
    threshold: float = 0.7,
) -> bool:
    return 0 <= reading.front_distance <= threshold
