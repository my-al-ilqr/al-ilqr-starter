#!/usr/bin/env python3

import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt


def load_csv(path: Path):
    rows = []
    with path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def main():
    if len(sys.argv) != 2:
        print("Usage: python visualization/plot_phase2_lqr.py build/phase2_linear_lqr_trajectory.csv")
        sys.exit(1)

    csv_path = Path(sys.argv[1])
    rows = load_csv(csv_path)

    k = [int(row["k"]) for row in rows]
    x0 = [float(row["x0"]) for row in rows]
    x1 = [float(row["x1"]) for row in rows]
    u0 = [float(row["u0"]) if row["u0"] else float("nan") for row in rows]

    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    axes[0].plot(k, x0, label="position")
    axes[0].set_ylabel("x0")
    axes[0].grid(True)

    axes[1].plot(k, x1, label="velocity", color="tab:orange")
    axes[1].set_ylabel("x1")
    axes[1].grid(True)

    axes[2].plot(k, u0, label="control", color="tab:green")
    axes[2].set_ylabel("u0")
    axes[2].set_xlabel("k")
    axes[2].grid(True)

    fig.suptitle("Phase 2 Finite-Horizon LQR")
    fig.tight_layout()
    output_path = csv_path.with_suffix(".png")
    fig.savefig(output_path, dpi=150)
    print(f"saved plot to {output_path}")


if __name__ == "__main__":
    main()
