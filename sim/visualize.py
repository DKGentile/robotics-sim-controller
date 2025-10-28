#!/usr/bin/env python3
import argparse
import csv
import os
import sys
from pathlib import Path
from typing import Dict, List

import numpy as np
import matplotlib

def _should_use_agg() -> bool:
    if "--headless" in sys.argv:
        return True
    display = os.environ.get("DISPLAY")
    if display:
        return False
    if sys.platform.startswith("win"):
        return False
    return True

if _should_use_agg():
    matplotlib.use("Agg")

from matplotlib import pyplot as plt  # noqa: E402


def load_csv(csv_path: Path) -> Dict[str, np.ndarray]:
    with csv_path.open(newline="") as csvfile:
        reader = csv.DictReader(csvfile)
        if not reader.fieldnames:
            raise ValueError("CSV file has no header.")
        raw: Dict[str, List[float]] = {field: [] for field in reader.fieldnames}
        for row in reader:
            for key, value in row.items():
                raw[key].append(float(value))
    return {key: np.asarray(values) for key, values in raw.items()}


def plot_trajectory(data: Dict[str, np.ndarray], output_dir: Path) -> None:
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(data["x"], data["y"], label="trajectory")
    ax.set_title("Top-Down Trajectory")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.axis("equal")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "trajectory.png", dpi=150)
    plt.close(fig)


def plot_signals(data: Dict[str, np.ndarray], output_dir: Path) -> None:
    time = data["time"]
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 11))

    axes[0].plot(time, data["linear_setpoint"], label="linear setpoint")
    axes[0].plot(time, data["linear_measured"], label="linear measured")
    axes[0].set_ylabel("Speed [m/s]")
    axes[0].set_title("Speed Tracking")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(time, data["yaw_true"], label="true yaw")
    axes[1].plot(time, data["imu_yaw"], label="imu yaw")
    axes[1].set_ylabel("Yaw [rad]")
    axes[1].set_title("Heading")
    axes[1].grid(True)
    axes[1].legend()

    axes[2].plot(time, data["left_setpoint"], label="left setpoint")
    axes[2].plot(time, data["left_measured"], label="left measured")
    axes[2].plot(time, data["right_setpoint"], label="right setpoint")
    axes[2].plot(time, data["right_measured"], label="right measured")
    axes[2].set_ylabel("Wheel [m/s]")
    axes[2].set_title("Wheel Velocity Tracking")
    axes[2].grid(True)
    axes[2].legend(ncol=2)

    axes[3].plot(time, data["left_pwm"], label="left pwm")
    axes[3].plot(time, data["right_pwm"], label="right pwm")
    axes[3].set_ylabel("PWM")
    axes[3].set_xlabel("Time [s]")
    axes[3].set_title("Control Effort")
    axes[3].grid(True)
    axes[3].legend()

    fig.tight_layout()
    fig.savefig(output_dir / "signals.png", dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize robotics telemetry.")
    parser.add_argument("csv_path", type=Path, help="CSV file produced by robotics-sim.")
    parser.add_argument("--output-dir", type=Path, default=None, help="Directory for PNG output.")
    parser.add_argument("--headless", action="store_true", help="Force non-interactive backend.")
    parser.add_argument("--show", action="store_true", help="Display figures (if backend permits).")
    args = parser.parse_args()

    data = load_csv(args.csv_path)

    output_dir = args.output_dir or (args.csv_path.resolve().parent / "plots")
    output_dir.mkdir(parents=True, exist_ok=True)

    plot_trajectory(data, output_dir)
    plot_signals(data, output_dir)

    if args.show and not _should_use_agg():
        plt.show()


if __name__ == "__main__":
    main()