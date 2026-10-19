"""
Data Logger — provided, do not modify
=======================================
Subscribes to /arm/joint2_error, saves a CSV, and automatically
generates the comparison plot when you stop it with Ctrl+C.

Usage:
  ros2 run arm_control_pkg data_logger --ros-args -p label:=P_only

  Change label for each run: P_only, PD, PID

When you press Ctrl+C:
  - CSV is saved to  src/arm_control_pkg/logs/<label>_<timestamp>.csv
  - Plot PNG is saved to  src/arm_control_pkg/logs/task2_plot.png
  - Open the PNG on your host machine to view the comparison

The plot includes EVERY CSV in logs/. Delete old CSVs before a new
tuning session:  rm -f /ros2_ws/src/arm_control_pkg/logs/*.csv
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import csv
import os
import time
import glob
import matplotlib

matplotlib.use("Agg")  # no display needed — saves to file only
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.gridspec as gridspec  # noqa: E402

LOG_DIR = "/ros2_ws/src/arm_control_pkg/logs"


class DataLogger(Node):

    def __init__(self):
        super().__init__("data_logger")

        self.declare_parameter("label", "run")
        self.label = self.get_parameter("label").get_parameter_value().string_value
        self.start_time = time.time()

        os.makedirs(LOG_DIR, exist_ok=True)
        self.filename = os.path.join(
            LOG_DIR, f"{self.label}_{int(self.start_time)}.csv"
        )

        self.file = open(self.filename, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["time_s", "error_rad"])

        self.create_subscription(Float64, "/arm/joint2_error", self._callback, 10)

        self.get_logger().info(f'Logging "{self.label}" → {self.filename}')
        self.get_logger().info("Press Ctrl+C to stop and save the plot.")

    def _callback(self, msg: Float64):
        t = time.time() - self.start_time
        self.writer.writerow([round(t, 4), round(msg.data, 6)])
        self.file.flush()

    def save_plot(self):
        """Read all CSVs in logs/ and save a comparison plot."""
        files = sorted(glob.glob(os.path.join(LOG_DIR, "*.csv")))
        if not files:
            return

        fig = plt.figure(figsize=(12, 5))
        gs = gridspec.GridSpec(1, 2, width_ratios=[2, 1])
        ax_err = fig.add_subplot(gs[0])
        ax_rms = fig.add_subplot(gs[1])

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

            if not times:
                continue

            ax_err.plot(times, errors, label=label, linewidth=1.5)
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

        out = os.path.join(LOG_DIR, "task2_plot.png")
        plt.tight_layout()
        plt.savefig(out, dpi=150)
        plt.close()
        print(f"\nPlot saved → {out}")
        print("Open on your host: src/arm_control_pkg/logs/task2_plot.png")

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node.save_plot()  # always runs on exit — Ctrl+C or normal stop
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
