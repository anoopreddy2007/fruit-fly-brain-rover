import math


class World:
    def __init__(self, obstacles=None, max_sensor_range=5.0):
        self.obstacles = obstacles or []
        self.max_sensor_range = max_sensor_range

    def front_obstacle(self, x, y, heading, half_angle=math.radians(55)):
        nearest = None
        nearest_distance = self.max_sensor_range

        for ox, oy, radius in self.obstacles:
            dx = ox - x
            dy = oy - y
            center_distance = math.hypot(dx, dy)
            clearance = center_distance - radius

            if center_distance <= radius:
                return {
                    "distance": 0.0,
                    "angle": 0.0,
                    "clearance": 0.0,
                }

            relative_angle = (
                math.atan2(dy, dx) - heading + math.pi
            ) % (2 * math.pi) - math.pi

            # Include the obstacle's angular size in the sensor cone.
            angular_radius = math.asin(
                min(1.0, radius / center_distance)
            )

            if abs(relative_angle) <= half_angle + angular_radius:
                if clearance < nearest_distance:
                    nearest_distance = clearance
                    nearest = {
                        "distance": max(0.0, clearance),
                        "angle": relative_angle,
                        "clearance": clearance,
                    }

        return nearest

    def front_distance(self, x, y, heading):
        obstacle = self.front_obstacle(x, y, heading)
        if obstacle is None:
            return self.max_sensor_range
        return obstacle["distance"]
