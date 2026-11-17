#!/usr/bin/env python3
"""Live plot of thunder_model_check: model vs sensor torque for all joints.

One subplot per joint with tau_model, tau_sensor and their residual over a rolling window.
"""

import argparse
import threading
from collections import deque

import matplotlib.pyplot as plt
import rclpy
from matplotlib.animation import FuncAnimation
from rclpy.node import Node
from sensor_msgs.msg import JointState

NUM_JOINTS = 7
TOPICS = {
    "model": "/thunder_model_check/tau_model",
    "sensor": "/thunder_model_check/tau_sensor",
    "residual": "/thunder_model_check/residual",
}


class ModelCheckPlotter(Node):
    def __init__(self, window, max_points):
        super().__init__("plot_model_check")
        self.window = window
        self.lock = threading.Lock()
        self.data = {
            key: (deque(maxlen=max_points), [deque(maxlen=max_points) for _ in range(NUM_JOINTS)])
            for key in TOPICS
        }
        for key, topic in TOPICS.items():
            self.create_subscription(
                JointState, topic, lambda msg, key=key: self.callback(key, msg), 100
            )

    def callback(self, key, msg):
        if len(msg.effort) < NUM_JOINTS:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        times, values = self.data[key]
        with self.lock:
            times.append(t)
            for i in range(NUM_JOINTS):
                values[i].append(msg.effort[i])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--window", type=float, default=10.0, help="time window [s]")
    parser.add_argument("--max-points", type=int, default=20000, help="samples kept per topic")
    parser.add_argument("--no-residual", action="store_true", help="hide the residual")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = ModelCheckPlotter(args.window, args.max_points)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    fig, axes = plt.subplots(4, 2, sharex=True, figsize=(13, 10))
    axes = axes.flatten()
    axes[-1].axis("off")
    styles = {"model": ("tau_model", "C0", "-"), "sensor": ("tau_sensor", "C1", "--")}
    if not args.no_residual:
        styles["residual"] = ("residual", "C3", ":")
    lines = {key: [] for key in styles}
    for i in range(NUM_JOINTS):
        ax = axes[i]
        for key, (label, color, ls) in styles.items():
            (line,) = ax.plot([], [], color=color, linestyle=ls, label=label)
            lines[key].append(line)
        ax.set_title(f"joint{i + 1}")
        ax.set_ylabel("torque [Nm]")
        ax.grid(True, alpha=0.3)
    axes[NUM_JOINTS - 1].set_xlabel("sim time [s]")
    axes[NUM_JOINTS - 2].set_xlabel("sim time [s]")
    axes[0].legend(loc="upper right")
    fig.tight_layout()

    def update(_):
        with node.lock:
            snapshot = {
                key: (list(node.data[key][0]), [list(v) for v in node.data[key][1]])
                for key in styles
            }
        t_last = max((times[-1] for times, _ in snapshot.values() if times), default=None)
        if t_last is None:
            return []
        for i in range(NUM_JOINTS):
            ymin, ymax = float("inf"), float("-inf")
            for key in styles:
                times, values = snapshot[key]
                n = min(len(times), len(values[i]))
                lines[key][i].set_data(times[:n], values[i][:n])
                visible = [v for t, v in zip(times[:n], values[i][:n]) if t >= t_last - args.window]
                if visible:
                    ymin, ymax = min(ymin, min(visible)), max(ymax, max(visible))
            axes[i].set_xlim(t_last - args.window, t_last)
            if ymin <= ymax:
                margin = max(0.05 * (ymax - ymin), 0.1)
                axes[i].set_ylim(ymin - margin, ymax + margin)
        return []

    _anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)  # noqa: F841
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
