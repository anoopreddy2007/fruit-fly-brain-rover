import math

from navigation.heading import normalize_angle


class HomeVector:
    """Track displacement from the starting point and estimate home direction."""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0

    def update(self, heading: float, speed: float, dt: float) -> None:
        if dt < 0:
            raise ValueError("dt must be non-negative")

        self.x += speed * math.cos(heading) * dt
        self.y += speed * math.sin(heading) * dt

    def direction_home(self) -> float:
        if math.isclose(self.x, 0.0) and math.isclose(self.y, 0.0):
            return 0.0

        return normalize_angle(math.atan2(-self.y, -self.x))

    def distance_from_home(self) -> float:
        return math.hypot(self.x, self.y)
