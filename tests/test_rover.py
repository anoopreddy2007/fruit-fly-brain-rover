from simulation.rover import Rover
from simulation.world import World


def test_rover_turns_when_obstacle_is_ahead():
    rover = Rover()
    world = World(obstacles=[(1.0, 0.0, 0.3)])

    _, command = rover.step(world)

    assert command.speed == 0.0
    assert command.turn_rate != 0.0


def test_rover_moves_when_path_is_clear():
    rover = Rover()
    world = World(obstacles=[])

    _, command = rover.step(world)

    assert command.speed > 0.0
    assert rover.x > 0.0


def test_return_home_mode_can_be_enabled():
    rover = Rover()
    rover.return_home()

    assert rover.mode == "RETURN_HOME"


def test_world_detects_obstacle_in_front():
    world = World(obstacles=[(2.0, 0.0, 0.5)])

    obstacle = world.front_obstacle(0.0, 0.0, 0.0)

    assert obstacle is not None
    assert obstacle["distance"] < 2.0
