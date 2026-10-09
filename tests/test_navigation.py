import math

from navigation.heading import update_heading
from navigation.home_vector import HomeVector


def test_equal_wheel_speeds_keep_heading():
    result = update_heading(0.5, 1.0, 1.0, 0.5, 1.0)
    assert math.isclose(result, 0.5)


def test_different_wheel_speeds_change_heading():
    result = update_heading(0.0, 0.0, 1.0, 0.5, 1.0)
    assert math.isclose(result, 2.0)


def test_home_vector_tracks_forward_motion():
    vector = HomeVector()
    vector.update(0.0, 1.0, 2.0)

    assert math.isclose(vector.x, 2.0)
    assert math.isclose(vector.y, 0.0)
    assert math.isclose(vector.distance_from_home(), 2.0)


def test_home_direction_points_backwards():
    vector = HomeVector()
    vector.update(0.0, 1.0, 2.0)

    assert math.isclose(abs(vector.direction_home()), math.pi)
