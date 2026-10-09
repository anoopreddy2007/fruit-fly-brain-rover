
import math

from navigation.heading import normalize_angle
from navigation.home_vector import HomeVector
from safety.governor import MotionCommand


class Rover:
    def __init__(self, x=0.0, y=0.0, heading=0.0):
        self.x = x
        self.y = y
        self.heading = heading

        self.home_vector = HomeVector()
        self.mode = "EXPLORE"

        # Obstacle avoidance state
        self.avoiding_obstacle = False
        self.turn_direction = 1.0
        self.avoidance_steps = 0

        # Controller parameters
        self.obstacle_threshold = 0.8
        self.clearance_threshold = 1.2
        self.turn_rate = 1.5
        self.heading_tolerance = 0.12
        self.home_radius = 0.05
        self.max_avoidance_steps = 100

    def return_home(self):
        self.mode = "RETURN_HOME"

    def step(self, world, dt=0.1):
        if dt <= 0:
            raise ValueError("dt must be positive")

        # Stop once home is reached during return.
        actual_home_distance = math.hypot(self.x, self.y)

        if self.mode == "RETURN_HOME" and actual_home_distance <= self.home_radius:
            return None, MotionCommand(speed=0.0, turn_rate=0.0)

        obstacle = world.front_obstacle(
            self.x,
            self.y,
            self.heading,
        )

        obstacle_distance = (
            obstacle["distance"]
            if obstacle is not None
            else world.max_sensor_range
        )

        # Start obstacle avoidance only when entering the danger zone.
        if not self.avoiding_obstacle:
            if obstacle_distance < self.obstacle_threshold:
                self.avoiding_obstacle = True
                self.avoidance_steps = 0

                # Choose a direction once and retain it during avoidance.
                if obstacle is not None and obstacle["angle"] > 0.05:
                    self.turn_direction = -1.0
                elif obstacle is not None and obstacle["angle"] < -0.05:
                    self.turn_direction = 1.0

        # Keep turning in the selected direction until there is clearance.
        if self.avoiding_obstacle:
            self.avoidance_steps += 1

            if obstacle_distance >= self.clearance_threshold:
                self.avoiding_obstacle = False
                self.avoidance_steps = 0

            elif self.avoidance_steps >= self.max_avoidance_steps:
                # Reverse the search direction if recovery takes too long.
                self.turn_direction *= -1.0
                self.avoidance_steps = 0

                command = MotionCommand(
                    speed=0.0,
                    turn_rate=self.turn_direction * self.turn_rate,
                )

                self._integrate(command, dt)
                return obstacle, command

            else:
                command = MotionCommand(
                    speed=0.0,
                    turn_rate=self.turn_direction * self.turn_rate,
                )

                self._integrate(command, dt)
                return obstacle, command

        # Normal navigation when the path is clear.
        if self.mode == "RETURN_HOME":
            target_heading = self.home_vector.direction_home()
            heading_error = normalize_angle(
                target_heading - self.heading
            )

            if abs(heading_error) > self.heading_tolerance:
                command = MotionCommand(
                    speed=0.0,
                    turn_rate=math.copysign(
                        self.turn_rate,
                        heading_error,
                    ),
                )
            else:
                command = MotionCommand(
                    speed=0.7,
                    turn_rate=0.0,
                )
        else:
            command = MotionCommand(
                speed=1.0,
                turn_rate=0.0,
            )

        self._integrate(command, dt)
        return obstacle, command

    def _integrate(self, command, dt):
        self.heading = normalize_angle(
            self.heading + command.turn_rate * dt
        )

        self.x += command.speed * math.cos(self.heading) * dt
        self.y += command.speed * math.sin(self.heading) * dt

        # Update the path integrator with actual commanded motion.
        self.home_vector.update(
            self.heading,
            command.speed,
            dt,
        )
