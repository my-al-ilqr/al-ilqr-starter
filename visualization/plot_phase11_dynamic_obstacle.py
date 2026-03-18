#!/usr/bin/env python3

import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle


def load_csv(path: Path):
    with path.open("r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def main():
    if len(sys.argv) != 4:
        print(
            "Usage: python3 visualization/plot_phase11_dynamic_obstacle.py "
            "build/phase11_dynamic_initial_trajectory.csv "
            "build/phase11_dynamic_optimized_trajectory.csv "
            "build/phase11_dynamic_obstacle_prediction.csv"
        )
        sys.exit(1)

    initial_csv = Path(sys.argv[1])
    optimized_csv = Path(sys.argv[2])
    obstacle_csv = Path(sys.argv[3])

    initial_rows = load_csv(initial_csv)
    optimized_rows = load_csv(optimized_csv)
    obstacle_rows = load_csv(obstacle_csv)

    x_init = [float(row["x0"]) for row in initial_rows]
    y_init = [float(row["x1"]) for row in initial_rows]
    x_opt = [float(row["x0"]) for row in optimized_rows]
    y_opt = [float(row["x1"]) for row in optimized_rows]
    speed_opt = [float(row["x3"]) for row in optimized_rows]
    k_opt = [int(row["k"]) for row in optimized_rows]

    obstacle_x = [float(row["cx"]) for row in obstacle_rows]
    obstacle_y = [float(row["cy"]) for row in obstacle_rows]
    radius = float(obstacle_rows[0]["radius"])

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))

    ax = axes[0]
    ax.plot(x_init, y_init, linestyle="--", color="tab:gray", linewidth=2, label="initial")
    ax.plot(x_opt, y_opt, color="tab:blue", linewidth=2.5, label="optimized")
    ax.axhline(0.0, color="tab:cyan", linestyle=":", linewidth=1.4, label="reference line")
    ax.axhline(2.3, color="tab:olive", linestyle="--", linewidth=1.2, label="road boundary")
    ax.axhline(-2.3, color="tab:olive", linestyle="--", linewidth=1.2)

    sample_ids = sorted(set([0, len(obstacle_rows) // 4, len(obstacle_rows) // 2, 3 * len(obstacle_rows) // 4, len(obstacle_rows) - 1]))
    for idx in sample_ids:
        alpha = 0.12 + 0.12 * (idx / max(1, len(obstacle_rows) - 1))
        ax.add_patch(Circle((obstacle_x[idx], obstacle_y[idx]), radius, color="tab:red", alpha=alpha))
    ax.plot(obstacle_x, obstacle_y, color="tab:red", linewidth=1.5, label="predicted obstacle path")

    ax.set_title("Dynamic obstacle avoidance")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.legend(loc="upper right")

    axes[1].plot(k_opt, speed_opt, color="tab:green", linewidth=2)
    axes[1].set_title("Optimized speed profile")
    axes[1].set_xlabel("k")
    axes[1].set_ylabel("speed [m/s]")
    axes[1].grid(True)

    fig.tight_layout()
    output_path = optimized_csv.with_name("phase11_dynamic_obstacle.png")
    fig.savefig(output_path, dpi=160)
    print(f"saved plot to {output_path}")


if __name__ == "__main__":
    main()
