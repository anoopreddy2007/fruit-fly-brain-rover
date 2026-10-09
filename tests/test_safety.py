from safety.governor import MotionCommand, apply_safety


def test_safety_stops_forward_motion_near_obstacle():
    command = MotionCommand(speed=1.0, turn_rate=0.0)
    result = apply_safety(command, front_distance=0.3)

    assert result.speed == 0.0
    assert result.turn_rate > 0.0


def test_safety_allows_motion_when_path_is_clear():
    command = MotionCommand(speed=1.0, turn_rate=0.0)
    result = apply_safety(command, front_distance=2.0)

    assert result.speed == 1.0
    assert result.turn_rate == 0.0
