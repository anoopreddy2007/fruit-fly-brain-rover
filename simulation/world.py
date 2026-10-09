import math


class World:
    def __init__(self, obstacles=None, max_sensor_range=5.0):
        self.obstacles = obstacles or []
        self.max_sensor_range = max_sensor_range

    def front_distance(self, x, y, heading):
        nearest = self.max_sensor_range

        for ox, oy, radius in self.obstacles:
            dx = ox - x
            dy = oy - y
            distance = math.hypot(dx, dy)

            if distance <= radius:
                return 0.0

            relative_angle = math.atan2(dy, dx) - heading
            relative_angle = (relative_angle + math.pi) % (
                2 * math.pi
            ) - math.pi

            if abs(relative_angle) < math.radians(25):
                nearest = min(nearest, max(0.0, distance - radius))

        return nearest
