#!/usr/bin/env python3

import csv
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt


def load_csv(path: Path):
    with path.open("r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 visualization/plot_phase8_autodrive.py build/phase8_autodrive_trajectory.csv")
        sys.exit(1)

    csv_path = Path(sys.argv[1])
    rows = load_csv(csv_path)

    x = [float(row["x0"]) for row in rows]
    y = [float(row["x1"]) for row in rows]
    yaw = [float(row["x2"]) for row in rows]
    v = [float(row["x3"]) for row in rows]
    a = [float(row["u0"]) if row["u0"] else float("nan") for row in rows]
    delta = [float(row["u1"]) if row["u1"] else float("nan") for row in rows]
    k = [int(row["k"]) for row in rows]

    fig, axes = plt.subplots(2, 2, figsize=(11, 8))

    axes[0, 0].plot(x, y, marker="o", markersize=2, label="optimized")
    axes[0, 0].axhline(2.3, color="tab:gray", linestyle="--", label="road boundary")
    axes[0, 0].axhline(-2.3, color="tab:gray", linestyle="--")
    axes[0, 0].axhline(0.0, color="tab:blue", linestyle=":", label="reference line")
    obstacle = plt.Circle((6.0, 0.0), 0.8, color="tab:red", alpha=0.25, label="obstacle")
    axes[0, 0].add_patch(obstacle)
    axes[0, 0].set_aspect("equal", adjustable="box")
    axes[0, 0].set_xlabel("x")
    axes[0, 0].set_ylabel("y")
    axes[0, 0].set_title("Autodrive trajectory")
    axes[0, 0].grid(True)
    axes[0, 0].legend()

    axes[0, 1].plot(k, yaw, color="tab:orange")
    axes[0, 1].set_title("yaw")
    axes[0, 1].set_xlabel("k")
    axes[0, 1].grid(True)

    axes[1, 0].plot(k, v, color="tab:green", label="speed")
    axes[1, 0].plot(k, a, color="tab:red", label="accel")
    axes[1, 0].set_title("speed and acceleration")
    axes[1, 0].set_xlabel("k")
    axes[1, 0].grid(True)
    axes[1, 0].legend()

    axes[1, 1].plot(k, delta, color="tab:purple")
    axes[1, 1].set_title("steering")
    axes[1, 1].set_xlabel("k")
    axes[1, 1].grid(True)

    fig.tight_layout()
    output_path = csv_path.with_suffix(".png")
    fig.savefig(output_path, dpi=150)
    print(f"saved plot to {output_path}")


if __name__ == "__main__":
    main()
