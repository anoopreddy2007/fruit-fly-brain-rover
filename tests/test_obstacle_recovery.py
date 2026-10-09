from simulation.rover import Rover
from simulation.world import World


def test_rover_does_not_flip_turn_direction_each_step():
    rover = Rover()
    world = World(obstacles=[(1.0, 0.0, 0.3)])

    rover.step(world, dt=0.1)
    initial_direction = rover.turn_direction

    for _ in range(5):
        rover.step(world, dt=0.1)

    assert rover.turn_direction == initial_direction


def test_rover_recovers_from_obstacle_and_moves_again():
    rover = Rover()
    world = World(obstacles=[(1.0, 0.0, 0.3)])

    speeds = []

    for _ in range(100):
        _, command = rover.step(world, dt=0.1)
        speeds.append(command.speed)

    assert any(speed > 0 for speed in speeds)
