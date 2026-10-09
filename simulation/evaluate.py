import math

from simulation.rover import Rover
from simulation.world import World


ROVER_RADIUS = 0.12
RETURN_STEP = 150
MAX_STEPS = 750
DT = 0.1


SCENARIOS = {
    "Original course": [
        (2.0, 0.0, 0.4),
        (3.5, 1.5, 0.5),
        (1.0, 2.5, 0.4),
    ],
    "Clear environment": [],
    "Single obstacle": [
        (2.0, 0.0, 0.4),
    ],
    "Offset obstacles": [
        (1.5, 0.5, 0.35),
        (3.0, 2.0, 0.45),
        (0.0, 3.0, 0.4),
    ],
    "Multiple obstacles": [
        (1.5, 0.0, 0.35),
        (2.5, 1.2, 0.4),
        (3.5, 2.0, 0.45),
        (0.5, 2.5, 0.35),
    ],
}


def point_segment_distance(px, py, ax, ay, bx, by):
    """Minimum distance from a point to a line segment."""
    dx = bx - ax
    dy = by - ay

    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)

    t = ((px - ax) * dx + (py - ay) * dy) / (
        dx * dx + dy * dy
    )
    t = max(0.0, min(1.0, t))

    closest_x = ax + t * dx
    closest_y = ay + t * dy

    return math.hypot(px - closest_x, py - closest_y)


def run_scenario(name, obstacles):
    world = World(obstacles=obstacles)
    rover = Rover()

    collision_count = 0
    return_start_distance = None
    arrival_step = None

    for step in range(MAX_STEPS):
        if step == RETURN_STEP:
            rover.return_home()
            return_start_distance = math.hypot(rover.x, rover.y)

        old_x, old_y = rover.x, rover.y

        rover.step(world, dt=DT)

        # Check the entire movement segment, not just the endpoint.
        for ox, oy, radius in obstacles:
            distance = point_segment_distance(
                ox, oy,
                old_x, old_y,
                rover.x, rover.y,
            )

            if distance < radius + ROVER_RADIUS:
                collision_count += 1

        distance_home = math.hypot(rover.x, rover.y)

        if (
            step >= RETURN_STEP
            and distance_home <= rover.home_radius
        ):
            arrival_step = step + 1
            break

    final_distance = math.hypot(rover.x, rover.y)

    return {
        "name": name,
        "collision_steps": collision_count,
        "arrival": arrival_step is not None,
        "arrival_step": arrival_step,
        "final_distance": final_distance,
        "start_distance": return_start_distance,
    }


def main():
    results = []

    print("\n=== FRUIT FLY BRAIN ROVER EVALUATION ===\n")

    for name, obstacles in SCENARIOS.items():
        result = run_scenario(name, obstacles)
        results.append(result)

        print(f"Scenario: {result['name']}")
        print(f"  Collision steps: {result['collision_steps']}")
        print(f"  Home reached: {'YES' if result['arrival'] else 'NO'}")
        print(f"  Final distance: {result['final_distance']:.2f} m")

        if result["arrival_step"] is not None:
            print(f"  Arrival step: {result['arrival_step']}")

        print()

    successes = sum(r["arrival"] for r in results)
    collisions = sum(r["collision_steps"] for r in results)

    print("=== SUMMARY ===")
    print(f"Scenarios passed: {successes}/{len(results)}")
    print(f"Scenarios with collision detections: "
          f"{sum(r['collision_steps'] > 0 for r in results)}")
    print(f"Total collision steps: {collisions}")

    success_rate = 100 * successes / len(results)
    print(f"Home-arrival rate: {success_rate:.1f}%")


if __name__ == "__main__":
    main()
