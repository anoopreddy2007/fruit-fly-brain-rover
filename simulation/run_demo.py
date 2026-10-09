from simulation.rover import Rover
from simulation.world import World


def main():
    world = World(
        obstacles=[
            (2.0, 0.0, 0.4),
            (3.5, 1.5, 0.5),
            (1.0, 2.5, 0.4),
        ]
    )

    rover = Rover()

    for step in range(300):
        if step == 150:
            rover.return_home()
            print("\n--- RETURN HOME MODE ---\n")

        obstacle, command = rover.step(world, dt=0.1)

        distance = (
            obstacle["distance"]
            if obstacle is not None
            else world.max_sensor_range
        )

        if step % 10 == 0:
            print(
                f"Step {step:03d} | Mode={rover.mode:11s} | "
                f"Position=({rover.x:.2f}, {rover.y:.2f}) | "
                f"Heading={rover.heading:.2f} rad | "
                f"Obstacle={distance:.2f} m | "
                f"Speed={command.speed:.1f}"
            )


if __name__ == "__main__":
    main()
