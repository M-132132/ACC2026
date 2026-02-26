#!/usr/bin/env python3
import argparse
import math
import os
from pathlib import Path
from collections import defaultdict

import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def quat_to_yaw(x, y, z, w) -> float:
    # yaw (Z) from quaternion
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def find_bag_dir(bag_path: str) -> Path:
    p = Path(bag_path).expanduser().resolve()
    if p.is_dir() and (p / "metadata.yaml").exists():
        return p

    # user might pass prefix in current dir, try resolve to directory
    if p.is_dir() and not (p / "metadata.yaml").exists():
        raise FileNotFoundError(f"{p} is a directory but has no metadata.yaml (not a rosbag2 folder).")

    # maybe it's a name like "sway_2026..." relative to cwd
    cwd_candidate = Path.cwd() / bag_path
    if cwd_candidate.is_dir() and (cwd_candidate / "metadata.yaml").exists():
        return cwd_candidate.resolve()

    raise FileNotFoundError(
        f"Cannot find rosbag2 directory for '{bag_path}'. "
        f"Please pass the bag folder that contains metadata.yaml."
    )


def main():
    parser = argparse.ArgumentParser(description="Plot ROS2 bag (rosbag2 sqlite3) for sway diagnosis.")
    parser.add_argument("bag", help="Path to rosbag2 folder (contains metadata.yaml)")
    parser.add_argument("-o", "--out", default=None, help="Output directory for plots (default: <bag>/plots)")
    args = parser.parse_args()

    bag_dir = find_bag_dir(args.bag)
    out_dir = Path(args.out).expanduser().resolve() if args.out else (bag_dir / "plots")
    out_dir.mkdir(parents=True, exist_ok=True)

    storage_options = StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_types = {name: get_message(typ) for name, typ in topic_types.items()}

    # time series containers (time in seconds relative to first message in bag)
    t0_ns = None

    series = defaultdict(list)  # key -> list of (t, value)
    xy = defaultdict(list)      # key -> list of (x, y)
    scan_times = []

    # TF extracts
    tf_map_odom = []       # (t, yaw)
    tf_odom_base = []      # (t, yaw)
    tf_base_scan = []      # (t, yaw)

    wanted_topics = {
        "/cmd_vel_nav", "/cmd_vel", "/odom", "/amcl_pose", "/scan", "/tf", "/tf_static"
    }

    count_by_topic = defaultdict(int)

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in wanted_topics:
            continue

        if t0_ns is None:
            t0_ns = int(t_ns)
        t = (int(t_ns) - t0_ns) * 1e-9

        count_by_topic[topic] += 1

        msg = deserialize_message(data, msg_types[topic])

        if topic == "/cmd_vel_nav":
            series["cmd_nav_lin_x"].append((t, msg.linear.x))
            series["cmd_nav_ang_z"].append((t, msg.angular.z))

        elif topic == "/cmd_vel":
            series["cmd_lin_x"].append((t, msg.linear.x))
            series["cmd_ang_z"].append((t, msg.angular.z))

        elif topic == "/odom":
            series["odom_lin_x"].append((t, msg.twist.twist.linear.x))
            series["odom_ang_z"].append((t, msg.twist.twist.angular.z))

            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

            series["odom_yaw"].append((t, yaw))
            xy["odom"].append((px, py))

        elif topic == "/amcl_pose":
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

            series["amcl_yaw"].append((t, yaw))
            xy["amcl"].append((px, py))
            series["amcl_x"].append((t, px))
            series["amcl_y"].append((t, py))

        elif topic == "/scan":
            scan_times.append(t)

        elif topic in ("/tf", "/tf_static"):
            # msg: TFMessage
            for tr in msg.transforms:
                parent = tr.header.frame_id
                child = tr.child_frame_id
                q = tr.transform.rotation
                yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

                if parent == "map" and child == "odom":
                    tf_map_odom.append((t, yaw))
                elif parent == "odom" and child in ("base_link", "base_footprint"):
                    tf_odom_base.append((t, yaw))
                elif parent in ("base_link", "base_footprint") and child == "base_scan":
                    tf_base_scan.append((t, yaw))

    # ---------- helper to plot ----------
    def plot_lines(items, title, xlabel, ylabel, filename):
        plt.figure()
        for label, key in items:
            if key not in series:
                continue
            ts = series[key]
            if not ts:
                continue
            xs = [a for a, _ in ts]
            ys = [b for _, b in ts]
            plt.plot(xs, ys, label=label)
        plt.title(title)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(out_dir / filename, dpi=160)
        plt.close()

    # 1) angular.z
    plot_lines(
        [
            ("cmd_vel_nav ang.z", "cmd_nav_ang_z"),
            ("cmd_vel ang.z", "cmd_ang_z"),
            ("odom ang.z", "odom_ang_z"),
        ],
        "Angular velocity (z): cmd vs odom",
        "time (s)", "rad/s",
        "01_ang_z_cmd_vs_odom.png"
    )

    # 2) linear.x
    plot_lines(
        [
            ("cmd_vel_nav lin.x", "cmd_nav_lin_x"),
            ("cmd_vel lin.x", "cmd_lin_x"),
            ("odom lin.x", "odom_lin_x"),
        ],
        "Linear velocity (x): cmd vs odom",
        "time (s)", "m/s",
        "02_lin_x_cmd_vs_odom.png"
    )

    # 3) yaw compare (odom vs amcl) + TF yaws if available
    # store TF yaw series into series dict for unified plotting
    if tf_map_odom:
        series["tf_map_odom_yaw"] = tf_map_odom
    if tf_odom_base:
        series["tf_odom_base_yaw"] = tf_odom_base
    if tf_base_scan:
        series["tf_base_scan_yaw"] = tf_base_scan

    plot_lines(
        [
            ("odom yaw", "odom_yaw"),
            ("amcl yaw", "amcl_yaw"),
            ("tf map->odom yaw", "tf_map_odom_yaw"),
            ("tf odom->base yaw", "tf_odom_base_yaw"),
            ("tf base->scan yaw", "tf_base_scan_yaw"),
        ],
        "Yaw comparison (rad)",
        "time (s)", "yaw (rad)",
        "03_yaw_compare.png"
    )

    # 4) XY trajectory
    plt.figure()
    if xy["odom"]:
        ox = [p[0] for p in xy["odom"]]
        oy = [p[1] for p in xy["odom"]]
        plt.plot(ox, oy, label="odom")
    if xy["amcl"]:
        ax = [p[0] for p in xy["amcl"]]
        ay = [p[1] for p in xy["amcl"]]
        plt.plot(ax, ay, label="amcl")
    plt.title("XY trajectory")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.tight_layout()
    plt.savefig(out_dir / "04_xy_trajectory.png", dpi=160)
    plt.close()

    # 5) scan dt (jitter)
    if len(scan_times) >= 2:
        dts = [scan_times[i] - scan_times[i - 1] for i in range(1, len(scan_times))]
        ts = scan_times[1:]
        plt.figure()
        plt.plot(ts, dts, label="scan dt")
        plt.title("LaserScan dt (jitter / drop hints)")
        plt.xlabel("time (s)")
        plt.ylabel("dt (s)")
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(out_dir / "05_scan_dt.png", dpi=160)
        plt.close()

    # print summary
    dur = (scan_times[-1] - scan_times[0]) if scan_times else None
    print("Bag:", bag_dir)
    print("Plots saved to:", out_dir)
    print("Counts:", dict(count_by_topic))
    if scan_times and dur and dur > 0:
        print(f"Scan approx Hz: {len(scan_times)/dur:.2f}")
    if "odom_ang_z" in series and series["odom_ang_z"]:
        print("Odom samples:", len(series["odom_ang_z"]))
    if "amcl_yaw" in series and series["amcl_yaw"]:
        print("AMCL samples:", len(series["amcl_yaw"]))


if __name__ == "__main__":
    main()
