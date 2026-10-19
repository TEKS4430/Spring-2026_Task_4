"""
Plotter — provided, do not modify
====================================
Reads all CSV files from logs/ and plots error over time for each run.
Run after completing all three Task 2 runs (P_only, PD, PID).
The data_logger already does this on Ctrl+C; use this to regenerate.
Delete old CSVs in logs/ before a new tuning session.

Usage:
  ros2 run arm_control_pkg plotter
"""

import os
import csv
import glob
import matplotlib

matplotlib.use("Agg")  # no display needed — saves to file only
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.gridspec as gridspec  # noqa: E402


def main(args=None):
    log_dir = "/ros2_ws/src/arm_control_pkg/logs"
    files = sorted(glob.glob(os.path.join(log_dir, "*.csv")))

    if not files:
        print(f"No CSV files found in {log_dir}")
        print("Run task2_pid with data_logger first to generate data.")
        return

    fig = plt.figure(figsize=(12, 5))
    gs = gridspec.GridSpec(1, 2, width_ratios=[2, 1])

    ax_err = fig.add_subplot(gs[0])  # main: error over time
    ax_rms = fig.add_subplot(gs[1])  # bar: RMS error per run

    ax_err.axhline(
        0,
        color="black",
        linewidth=0.8,
        linestyle="--",
        label="Zero error (perfect tracking)",
    )

    labels, rms_values = [], []

    for filepath in files:
        # "P_only_1714740000.csv" -> "P_only"
        label = os.path.basename(filepath).rsplit("_", 1)[0]
        times, errors = [], []

        with open(filepath) as f:
            reader = csv.DictReader(f)
            for row in reader:
                times.append(float(row["time_s"]))
                errors.append(float(row["error_rad"]))

        if not errors:
            print(f"Skipping empty log: {filepath}")
            continue

        ax_err.plot(times, errors, label=label, linewidth=1.5)

        # RMS error — lower is better
        rms = (sum(e**2 for e in errors) / len(errors)) ** 0.5
        labels.append(label)
        rms_values.append(rms)

    ax_err.set_xlabel("Time (s)")
    ax_err.set_ylabel("Joint 2 error (rad)\ntarget − actual")
    ax_err.set_title(
        "Task 2 — PID Tuning: Tracking Error Over Time\n"
        "Smaller error = better tracking"
    )
    ax_err.legend()
    ax_err.grid(True, alpha=0.3)

    colors = ["#e74c3c", "#e67e22", "#2ecc71", "#3498db", "#9b59b6"]
    colors = [colors[i % len(colors)] for i in range(len(labels))]
    ax_rms.bar(labels, rms_values, color=colors, edgecolor="black", linewidth=0.8)
    ax_rms.set_ylabel("RMS error (rad)")
    ax_rms.set_title("Tracking quality\n(lower = better)")
    ax_rms.grid(True, alpha=0.3, axis="y")
    for i, v in enumerate(rms_values):
        ax_rms.text(i, v + 0.002, f"{v:.4f}", ha="center", fontsize=9)

    out = os.path.join(log_dir, "task2_plot.png")
    plt.tight_layout()
    plt.savefig(out, dpi=150)
    print(f"Plot saved to {out}")
    print("Open it from your host machine at: src/arm_control_pkg/logs/task2_plot.png")


if __name__ == "__main__":
    main()
