import math

from navigation.heading import normalize_angle
from navigation.home_vector import HomeVector
from perception.vision import SensorReading
from safety.governor import MotionCommand, apply_safety


class Rover:
    def __init__(self, x=0.0, y=0.0, heading=0.0):
        self.x = x
        self.y = y
        self.heading = heading
        self.home_vector = HomeVector()

    def step(self, world, dt=0.1):
        requested = MotionCommand(speed=1.0, turn_rate=0.0)

        reading = SensorReading(
            front_distance=world.front_distance(
                self.x, self.y, self.heading
            )
        )

        safe_command = apply_safety(
            requested,
            reading.front_distance,
        )

        self.heading = normalize_angle(
            self.heading + safe_command.turn_rate * dt
        )

        self.x += safe_command.speed * math.cos(self.heading) * dt
        self.y += safe_command.speed * math.sin(self.heading) * dt

        self.home_vector.update(
            self.heading,
            safe_command.speed,
            dt,
        )

        return reading, safe_command
