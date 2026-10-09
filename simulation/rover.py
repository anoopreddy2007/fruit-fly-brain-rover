
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

        # Obstacle avoidance
        self.avoiding_obstacle = False
        self.turn_direction = 1.0
        self.avoidance_steps = 0

        self.obstacle_threshold = 0.8
        self.clearance_threshold = 1.2
        self.turn_rate = 1.5
        self.heading_tolerance = 0.12
        self.home_radius = 0.05

        # Stuck detection
        self.max_avoidance_steps = 50
        self.last_position = (x, y)
        self.no_progress_steps = 0
        self.progress_epsilon = 0.05

    def return_home(self):
        self.mode = "RETURN_HOME"

    def _distance_at_heading(self, world, heading):
        obstacle = world.front_obstacle(
            self.x,
            self.y,
            heading,
        )

        if obstacle is None:
            return world.max_sensor_range

        return obstacle["distance"]

    def _choose_turn_direction(self, world):
        # Compare free space on both sides of the rover.
        left_heading = normalize_angle(
            self.heading + math.radians(45)
        )
        right_heading = normalize_angle(
            self.heading - math.radians(45)
        )

        left_clearance = self._distance_at_heading(
            world, left_heading
        )
        right_clearance = self._distance_at_heading(
            world, right_heading
        )

        if left_clearance > right_clearance:
            return 1.0

        if right_clearance > left_clearance:
            return -1.0

        # Deterministic tie-breaker.
        return self.turn_direction

    def step(self, world, dt=0.1):
        if dt <= 0:
            raise ValueError("dt must be positive")

        # Stop when the rover reaches home.
        actual_home_distance = math.hypot(self.x, self.y)

        if (
            self.mode == "RETURN_HOME"
            and actual_home_distance <= self.home_radius
        ):
            return None, MotionCommand(
                speed=0.0,
                turn_rate=0.0,
            )

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

        # Start avoiding an obstacle when it enters the danger zone.
        if (
            not self.avoiding_obstacle
            and obstacle_distance < self.obstacle_threshold
        ):
            self.avoiding_obstacle = True
            self.avoidance_steps = 0
            self.turn_direction = self._choose_turn_direction(world)

        # Continue turning until the path has adequate clearance.
        if self.avoiding_obstacle:
            self.avoidance_steps += 1

            if obstacle_distance >= self.clearance_threshold:
                self.avoiding_obstacle = False
                self.avoidance_steps = 0
                self.no_progress_steps = 0

            else:
                if self.avoidance_steps >= self.max_avoidance_steps:
                    # Re-evaluate the safer direction after prolonged turning.
                    self.turn_direction = self._choose_turn_direction(world)
                    self.avoidance_steps = 0

                command = MotionCommand(
                    speed=0.0,
                    turn_rate=self.turn_direction * self.turn_rate,
                )

                self._integrate(command, dt)
                self._update_progress()
                return obstacle, command

        # Navigate toward home when the path is clear.
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
        self._update_progress()

        return obstacle, command

    def _integrate(self, command, dt):
        self.heading = normalize_angle(
            self.heading + command.turn_rate * dt
        )

        self.x += command.speed * math.cos(self.heading) * dt
        self.y += command.speed * math.sin(self.heading) * dt

        self.home_vector.update(
            self.heading,
            command.speed,
            dt,
        )

    def _update_progress(self):
        current_position = (self.x, self.y)

        movement = math.hypot(
            current_position[0] - self.last_position[0],
            current_position[1] - self.last_position[1],
        )

        if movement < self.progress_epsilon:
            self.no_progress_steps += 1
        else:
            self.no_progress_steps = 0

        self.last_position = current_position
