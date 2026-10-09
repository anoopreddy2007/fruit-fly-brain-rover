import math

from simulation.rover import Rover
from simulation.world import World


def test_rover_returns_home_without_obstacles():
    rover = Rover(heading=math.pi / 2)
    world = World(obstacles=[])

    for _ in range(20):
        rover.step(world, dt=0.1)

    initial_distance = math.hypot(rover.x, rover.y)
    rover.return_home()

    for _ in range(100):
        rover.step(world, dt=0.1)

    final_distance = math.hypot(rover.x, rover.y)

    assert final_distance < initial_distance
    assert final_distance < 0.1


def test_rover_enters_return_home_mode():
    rover = Rover()
    rover.return_home()

    assert rover.mode == "RETURN_HOME"
