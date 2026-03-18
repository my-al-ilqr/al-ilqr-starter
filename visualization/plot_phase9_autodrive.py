#!/usr/bin/env python3

import csv
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon


ROAD_LOWER = -2.3
ROAD_UPPER = 2.3
OBSTACLE_X = 6.0
OBSTACLE_Y = 0.0
OBSTACLE_RADIUS = 0.8
VEHICLE_LENGTH = 4.6
VEHICLE_WIDTH = 1.9


def load_csv(path: Path):
    with path.open("r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def draw_vehicle(ax, x, y, yaw, color, alpha=0.18):
    half_l = VEHICLE_LENGTH / 2.0
    half_w = VEHICLE_WIDTH / 2.0
    corners = [
        (half_l, half_w),
        (half_l, -half_w),
        (-half_l, -half_w),
        (-half_l, half_w),
    ]
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    rotated = []
    for cx, cy in corners:
        rx = x + cos_yaw * cx - sin_yaw * cy
        ry = y + sin_yaw * cx + cos_yaw * cy
        rotated.append((rx, ry))
    ax.add_patch(Polygon(rotated, closed=True, facecolor=color, edgecolor=color, alpha=alpha))


def main():
    if len(sys.argv) != 4:
        print(
            "Usage: python3 visualization/plot_phase9_autodrive.py "
            "build/phase9_autodrive_initial_trajectory.csv "
            "build/phase9_autodrive_optimized_trajectory.csv "
            "build/phase9_autodrive_outer_log.csv"
        )
        sys.exit(1)

    initial_csv = Path(sys.argv[1])
    optimized_csv = Path(sys.argv[2])
    outer_log_csv = Path(sys.argv[3])

    initial_rows = load_csv(initial_csv)
    optimized_rows = load_csv(optimized_csv)
    outer_rows = load_csv(outer_log_csv)

    x_init = [float(row["x0"]) for row in initial_rows]
    y_init = [float(row["x1"]) for row in initial_rows]
    yaw_init = [float(row["x2"]) for row in initial_rows]

    x_opt = [float(row["x0"]) for row in optimized_rows]
    y_opt = [float(row["x1"]) for row in optimized_rows]
    yaw_opt = [float(row["x2"]) for row in optimized_rows]
    speed_opt = [float(row["x3"]) for row in optimized_rows]
    accel_opt = [float(row["u0"]) if row["u0"] else float("nan") for row in optimized_rows]
    steer_opt = [float(row["u1"]) if row["u1"] else float("nan") for row in optimized_rows]
    knot = [int(row["k"]) for row in optimized_rows]

    outer_iter = [int(row["outer_iteration"]) for row in outer_rows]
    base_cost = [float(row["base_cost"]) for row in outer_rows]
    violation = [float(row["max_violation"]) for row in outer_rows]
    penalty = [float(row["max_penalty"]) for row in outer_rows]

    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(2, 3, height_ratios=[1.2, 1.0])

    ax_traj = fig.add_subplot(gs[0, :])
    ax_speed = fig.add_subplot(gs[1, 0])
    ax_control = fig.add_subplot(gs[1, 1])
    ax_log = fig.add_subplot(gs[1, 2])

    ax_traj.plot(x_init, y_init, color="tab:gray", linestyle="--", linewidth=2, label="initial")
    ax_traj.plot(x_opt, y_opt, color="tab:blue", linewidth=2.5, label="optimized")
    ax_traj.axhline(0.0, color="tab:cyan", linestyle=":", linewidth=1.5, label="reference line")
    ax_traj.axhline(ROAD_UPPER, color="tab:olive", linestyle="--", linewidth=1.3, label="road boundary")
    ax_traj.axhline(ROAD_LOWER, color="tab:olive", linestyle="--", linewidth=1.3)
    ax_traj.add_patch(Circle((OBSTACLE_X, OBSTACLE_Y), OBSTACLE_RADIUS, color="tab:red", alpha=0.25))

    sample_ids = sorted(set([0, len(optimized_rows) // 4, len(optimized_rows) // 2, 3 * len(optimized_rows) // 4, len(optimized_rows) - 1]))
    for idx in sample_ids:
        draw_vehicle(ax_traj, x_opt[idx], y_opt[idx], yaw_opt[idx], "tab:blue")
    for idx in [0, len(initial_rows) // 2, len(initial_rows) - 1]:
        draw_vehicle(ax_traj, x_init[idx], y_init[idx], yaw_init[idx], "tab:gray", alpha=0.10)

    ax_traj.set_title("Autodrive AL-iLQR: initial vs optimized trajectory")
    ax_traj.set_xlabel("x [m]")
    ax_traj.set_ylabel("y [m]")
    ax_traj.set_aspect("equal", adjustable="box")
    ax_traj.grid(True)
    ax_traj.legend(loc="upper right")

    ax_speed.plot(knot, speed_opt, color="tab:green", label="speed")
    ax_speed.set_title("Optimized speed")
    ax_speed.set_xlabel("k")
    ax_speed.grid(True)
    ax_speed.legend()

    ax_control.plot(knot, accel_opt, color="tab:red", label="accel")
    ax_control.plot(knot, steer_opt, color="tab:purple", label="steering")
    ax_control.set_title("Optimized controls")
    ax_control.set_xlabel("k")
    ax_control.grid(True)
    ax_control.legend()

    ax_log.plot(outer_iter, base_cost, marker="o", color="tab:blue", label="base cost")
    ax_log.set_title("Outer-loop metrics")
    ax_log.set_xlabel("outer iter")
    ax_log.grid(True)
    ax_log2 = ax_log.twinx()
    ax_log2.plot(outer_iter, violation, marker="s", color="tab:red", label="violation")
    ax_log2.plot(outer_iter, penalty, marker="^", color="tab:orange", label="penalty")

    lines1, labels1 = ax_log.get_legend_handles_labels()
    lines2, labels2 = ax_log2.get_legend_handles_labels()
    ax_log2.legend(lines1 + lines2, labels1 + labels2, loc="upper right")

    fig.tight_layout()
    output_path = optimized_csv.with_name("phase9_autodrive_visualization.png")
    fig.savefig(output_path, dpi=160)
    print(f"saved plot to {output_path}")


if __name__ == "__main__":
    main()
