
import math
import matplotlib.pyplot as plt

from simulation.rover import Rover
from simulation.world import World


def main():
    # Create the simulated world and obstacles
    world = World(
        obstacles=[
            (2.0, 0.0, 0.4),
            (3.5, 1.5, 0.5),
            (1.0, 2.5, 0.4),
        ]
    )

    rover = Rover()

    trajectory_x = [rover.x]
    trajectory_y = [rover.y]

    return_step = 150
    total_steps = 600
    return_position = None

    # Run exploration followed by return-home mode
    for step in range(total_steps):

        # Switch to return-home mode
        if step == return_step:
            rover.return_home()
            return_position = (rover.x, rover.y)

            print("\n--- RETURN HOME MODE ---")
            print(
                f"Return starts at: "
                f"({rover.x:.2f}, {rover.y:.2f})"
            )
            print(
                f"Distance from home: "
                f"{math.hypot(rover.x, rover.y):.2f} m"
            )

        # Execute one simulation step
        obstacle, command = rover.step(world, dt=0.1)

        trajectory_x.append(rover.x)
        trajectory_y.append(rover.y)

        # Log progress every 50 steps
        if step % 50 == 0:
            distance_home = math.hypot(rover.x, rover.y)

            obstacle_distance = (
                obstacle["distance"]
                if obstacle is not None
                else world.max_sensor_range
            )

            print(
                f"Step {step:03d} | "
                f"Mode={rover.mode:11s} | "
                f"Position=({rover.x:.2f}, {rover.y:.2f}) | "
                f"Distance home={distance_home:.2f} m | "
                f"Obstacle={obstacle_distance:.2f} m | "
                f"Speed={command.speed:.1f} | "
                f"Turn={command.turn_rate:.1f}"
            )

    # Calculate final distance from the origin
    final_distance = math.hypot(rover.x, rover.y)

    print("\n--- SIMULATION RESULTS ---")
    print(f"Final position: ({rover.x:.2f}, {rover.y:.2f})")
    print(f"Distance from home: {final_distance:.2f} m")

    if return_position is not None:
        start_distance = math.hypot(
            return_position[0],
            return_position[1],
        )

        reduction = start_distance - final_distance

        print(f"Return-start distance: {start_distance:.2f} m")
        print(f"Distance reduction: {reduction:.2f} m")

        if final_distance < 0.2:
            print("Result: HOME REACHED")
        elif final_distance < start_distance:
            print("Result: RETURN PROGRESS MADE, HOME NOT REACHED")
        else:
            print("Result: RETURN FAILED")

    # Plot exploration and return-home trajectories
    fig, ax = plt.subplots(figsize=(10, 7))

    ax.plot(
        trajectory_x[:return_step + 1],
        trajectory_y[:return_step + 1],
        label="Exploration",
    )

    ax.plot(
        trajectory_x[return_step:],
        trajectory_y[return_step:],
        label="Return-home phase",
    )

    # Mark home
    ax.scatter(
        0,
        0,
        marker="*",
        s=220,
        label="Home",
        zorder=5,
    )

    # Mark where return-home mode begins
    if return_position is not None:
        ax.scatter(
            return_position[0],
            return_position[1],
            marker="s",
            s=80,
            label="Return start",
            zorder=5,
        )

    # Mark final position
    ax.scatter(
        rover.x,
        rover.y,
        marker="o",
        s=80,
        label="Final position",
        zorder=5,
    )

    # Draw obstacles
    for x, y, radius in world.obstacles:
        obstacle_patch = plt.Circle(
            (x, y),
            radius,
            color="red",
            alpha=0.35,
        )
        ax.add_patch(obstacle_patch)

    ax.set_title("Fruit Fly Brain Rover — Simulation")
    ax.set_xlabel("X position (m)")
    ax.set_ylabel("Y position (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True)
    ax.legend()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
