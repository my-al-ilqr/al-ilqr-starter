#!/usr/bin/env python3

import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt


def load_csv(path: Path):
    with path.open("r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def main():
    if len(sys.argv) != 2:
        print("Usage: python visualization/plot_phase3_ilqr.py build/phase3_unicycle_ilqr_trajectory.csv")
        sys.exit(1)

    csv_path = Path(sys.argv[1])
    rows = load_csv(csv_path)

    x = [float(row["x0"]) for row in rows]
    y = [float(row["x1"]) for row in rows]
    yaw = [float(row["x2"]) for row in rows]
    u0 = [float(row["u0"]) if row["u0"] else float("nan") for row in rows]
    u1 = [float(row["u1"]) if row["u1"] else float("nan") for row in rows]
    k = [int(row["k"]) for row in rows]

    fig, axes = plt.subplots(2, 2, figsize=(10, 8))
    axes[0, 0].plot(x, y, marker="o", markersize=2)
    axes[0, 0].set_title("x-y trajectory")
    axes[0, 0].set_xlabel("x")
    axes[0, 0].set_ylabel("y")
    axes[0, 0].axis("equal")
    axes[0, 0].grid(True)

    axes[0, 1].plot(k, yaw)
    axes[0, 1].set_title("yaw")
    axes[0, 1].set_xlabel("k")
    axes[0, 1].grid(True)

    axes[1, 0].plot(k, u0, color="tab:green")
    axes[1, 0].set_title("control v")
    axes[1, 0].set_xlabel("k")
    axes[1, 0].grid(True)

    axes[1, 1].plot(k, u1, color="tab:red")
    axes[1, 1].set_title("control omega")
    axes[1, 1].set_xlabel("k")
    axes[1, 1].grid(True)

    fig.tight_layout()
    output_path = csv_path.with_suffix(".png")
    fig.savefig(output_path, dpi=150)
    print(f"saved plot to {output_path}")


if __name__ == "__main__":
    main()
