from simulation.rover import Rover
from simulation.world import World


def main():
    world = World(obstacles=[(2.0, 0.0, 0.4)])
    rover = Rover()

    for step in range(50):
        reading, command = rover.step(world)

        print(
            f"Step {step:02d} | "
            f"Position=({rover.x:.2f}, {rover.y:.2f}) | "
            f"Heading={rover.heading:.2f} rad | "
            f"Front={reading.front_distance:.2f} m | "
            f"Speed={command.speed:.1f}"
        )


if __name__ == "__main__":
    main()
