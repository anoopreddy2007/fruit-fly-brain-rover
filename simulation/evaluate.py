
import json
import math
import random
from pathlib import Path

from simulation.rover import Rover
from simulation.world import World


ROVER_RADIUS = 0.12
RETURN_STEP = 150
MAX_STEPS = 750
DT = 0.1
HOME_RADIUS = 0.05
RANDOM_SEED = 42

DEBUG_SCENARIOS = {
    "Generated 01",
    "Generated 16",
    "Generated 18",
}

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


def generate_extra_scenarios(count=20, seed=RANDOM_SEED):
    """Generate repeatable obstacle layouts for stress testing."""
    rng = random.Random(seed)
    scenarios = {}

    for index in range(count):
        obstacles = []
        obstacle_count = rng.randint(2, 6)

        for _ in range(obstacle_count):
            x = rng.uniform(0.7, 5.0)
            y = rng.uniform(-1.0, 5.0)
            radius = rng.uniform(0.25, 0.5)

            obstacles.append(
                (round(x, 2), round(y, 2), round(radius, 2))
            )

        scenarios[f"Generated {index + 1:02d}"] = obstacles

    return scenarios


def point_segment_distance(px, py, ax, ay, bx, by):
    """Minimum distance from a point to a movement segment."""
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
    """Run a scenario and collect performance and debug metrics."""
    world = World(obstacles=obstacles)
    rover = Rover()

    collision_steps = 0
    minimum_clearance = float("inf")
    return_start_distance = None
    arrival_step = None

    for step in range(MAX_STEPS):
        if step == RETURN_STEP:
            rover.return_home()
            return_start_distance = math.hypot(rover.x, rover.y)

        old_x, old_y = rover.x, rover.y

        rover.step(world, dt=DT)

        # Trace rover behavior during the return phase.
        if (
            name in DEBUG_SCENARIOS
            and step >= RETURN_STEP
            and step % 50 == 0
        ):
            print(
                f"TRACE | {name} | step={step} | "
                f"position=({rover.x:.3f}, {rover.y:.3f}) | "
                f"distance_home={math.hypot(rover.x, rover.y):.3f} m | "
                f"heading={rover.heading:.3f} rad | "
                f"avoiding={rover.avoiding_obstacle} | "
                f"avoidance_steps={rover.avoidance_steps}"
            )

        # Check the entire movement segment for obstacle collisions.
        step_collision = False

        for ox, oy, radius in obstacles:
            center_distance = point_segment_distance(
                ox, oy, old_x, old_y, rover.x, rover.y
            )

            clearance = (
                center_distance - radius - ROVER_RADIUS
            )

            minimum_clearance = min(
                minimum_clearance, clearance
            )

            if clearance < 0:
                step_collision = True

        if step_collision:
            collision_steps += 1

        distance_home = math.hypot(rover.x, rover.y)

        if step >= RETURN_STEP and distance_home <= HOME_RADIUS:
            arrival_step = step + 1
            break

    final_distance = math.hypot(rover.x, rover.y)

    if minimum_clearance == float("inf"):
        minimum_clearance = None

    return {
        "name": name,
        "obstacles": [
            {
                "x": ox,
                "y": oy,
                "radius": radius,
            }
            for ox, oy, radius in obstacles
        ],
        "obstacle_count": len(obstacles),
        "collision_steps": collision_steps,
        "arrival": arrival_step is not None,
        "arrival_step": arrival_step,
        "final_position_m": {
            "x": round(rover.x, 3),
            "y": round(rover.y, 3),
        },
        "final_heading_rad": round(rover.heading, 3),
        "final_distance_m": round(final_distance, 3),
        "return_start_distance_m": (
            round(return_start_distance, 3)
            if return_start_distance is not None
            else None
        ),
        "minimum_clearance_m": (
            round(minimum_clearance, 3)
            if minimum_clearance is not None
            else None
        ),
        "final_mode": str(rover.mode),
        "avoiding_obstacle": getattr(
            rover, "avoiding_obstacle", None
        ),
        "avoidance_steps": getattr(
            rover, "avoidance_steps", None
        ),
    }


def print_failed_diagnostics(failed_results):
    """Print layouts and final controller state for failed scenarios."""
    print("\n=== FAILED SCENARIO DIAGNOSTICS ===")

    if not failed_results:
        print("No failed scenarios.")
        return

    for result in failed_results:
        print(f"\n{result['name']}:")
        print(f"  Arrival: {result['arrival']}")
        print(
            f"  Final distance: "
            f"{result['final_distance_m']:.3f} m"
        )
        print(f"  Final position: {result['final_position_m']}")
        print(
            f"  Final heading: "
            f"{result['final_heading_rad']:.3f} rad"
        )
        print(f"  Final mode: {result['final_mode']}")
        print(
            f"  Avoiding obstacle: "
            f"{result['avoiding_obstacle']}"
        )
        print(
            f"  Avoidance steps: "
            f"{result['avoidance_steps']}"
        )
        print(f"  Collision steps: {result['collision_steps']}")
        print("  Obstacles (x, y, radius):")

        if not result["obstacles"]:
            print("    None")
        else:
            for obstacle in result["obstacles"]:
                print(
                    f"    ({obstacle['x']}, "
                    f"{obstacle['y']}, "
                    f"{obstacle['radius']})"
                )


def main():
    all_scenarios = dict(SCENARIOS)
    all_scenarios.update(generate_extra_scenarios())

    results = []

    print("\n=== FRUIT FLY BRAIN ROVER ROBUSTNESS EVALUATION ===")
    print(f"Scenarios: {len(all_scenarios)}")
    print(f"Random seed: {RANDOM_SEED}")
    print(f"Maximum steps per scenario: {MAX_STEPS}")
    print(f"Return begins at step: {RETURN_STEP}")
    print(f"Home acceptance radius: {HOME_RADIUS} m\n")

    for name, obstacles in all_scenarios.items():
        result = run_scenario(name, obstacles)
        results.append(result)

        status = (
            "PASS"
            if result["arrival"] and result["collision_steps"] == 0
            else "FAIL"
        )

        print(
            f"{status} | {result['name']} | "
            f"Obstacles={result['obstacle_count']} | "
            f"Collisions={result['collision_steps']} | "
            f"Home={'YES' if result['arrival'] else 'NO'} | "
            f"Final distance={result['final_distance_m']:.2f} m | "
            f"Clearance={result['minimum_clearance_m']}"
        )

        if name in DEBUG_SCENARIOS:
            print(f"\n--- DEBUG: {name} ---")
            print(f"Final position: {result['final_position_m']}")
            print(f"Final heading: {result['final_heading_rad']} rad")
            print(f"Final mode: {result['final_mode']}")
            print(
                f"Avoiding obstacle: "
                f"{result['avoiding_obstacle']}"
            )
            print(
                f"Avoidance steps: {result['avoidance_steps']}"
            )
            print(
                f"Final distance: "
                f"{result['final_distance_m']} m\n"
            )

    total = len(results)

    arrivals = sum(
        result["arrival"] for result in results
    )

    collision_scenarios = sum(
        result["collision_steps"] > 0
        for result in results
    )

    total_collision_steps = sum(
        result["collision_steps"] for result in results
    )

    fully_passed = sum(
        result["arrival"] and result["collision_steps"] == 0
        for result in results
    )

    arrival_rate = (
        100.0 * arrivals / total if total else 0.0
    )
    pass_rate = (
        100.0 * fully_passed / total if total else 0.0
    )

    failed_results = [
        result
        for result in results
        if not result["arrival"] or result["collision_steps"] > 0
    ]

    summary = {
        "seed": RANDOM_SEED,
        "scenario_count": total,
        "return_step": RETURN_STEP,
        "max_steps": MAX_STEPS,
        "time_step_seconds": DT,
        "home_radius_m": HOME_RADIUS,
        "rover_radius_m": ROVER_RADIUS,
        "home_arrivals": arrivals,
        "home_arrival_rate_percent": round(arrival_rate, 2),
        "fully_passed_scenarios": fully_passed,
        "pass_rate_percent": round(pass_rate, 2),
        "scenarios_with_collision": collision_scenarios,
        "total_collision_steps": total_collision_steps,
        "failed_scenario_names": [
            result["name"] for result in failed_results
        ],
        "results": results,
    }

    print("\n=== SUMMARY ===")
    print(f"Home arrivals: {arrivals}/{total} ({arrival_rate:.1f}%)")
    print(f"Fully passed: {fully_passed}/{total} ({pass_rate:.1f}%)")
    print(f"Scenarios with collisions: {collision_scenarios}")
    print(f"Total collision steps: {total_collision_steps}")

    print_failed_diagnostics(failed_results)

    output_path = Path("docs") / "evaluation_results.json"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(summary, indent=2),
        encoding="utf-8",
    )

    print(f"\nDetailed results saved to: {output_path}")


if __name__ == "__main__":
    main()
