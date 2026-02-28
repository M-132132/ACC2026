#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def quat_to_yaw(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def find_bag_dir(bag_path: str) -> Path:
    p = Path(bag_path).expanduser().resolve()
    if p.is_dir() and (p / "metadata.yaml").exists():
        return p
    cand = (Path.cwd() / bag_path).resolve()
    if cand.is_dir() and (cand / "metadata.yaml").exists():
        return cand
    raise FileNotFoundError(f"Cannot find rosbag2 folder for '{bag_path}' (metadata.yaml not found).")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", help="rosbag2 folder containing metadata.yaml")
    ap.add_argument("--out", default=None, help="output dir (default: <bag>/csv)")
    args = ap.parse_args()

    bag_dir = find_bag_dir(args.bag)
    out_dir = Path(args.out).expanduser().resolve() if args.out else (bag_dir / "csv")
    out_dir.mkdir(parents=True, exist_ok=True)

    # Open bag
    storage_options = StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_types = {name: get_message(typ) for name, typ in topic_types.items()}

    # Prepare CSV writers
    def open_csv(name, header):
        f = open(out_dir / name, "w", newline="")
        w = csv.writer(f)
        w.writerow(header)
        return f, w

    f_cmdnav, w_cmdnav = open_csv("cmd_vel_nav.csv", ["t", "lin_x", "ang_z"])
    f_cmd, w_cmd = open_csv("cmd_vel.csv", ["t", "lin_x", "ang_z"])
    f_odom, w_odom = open_csv("odom.csv", ["t", "x", "y", "yaw", "vx", "wz"])
    f_amcl, w_amcl = open_csv("amcl_pose.csv", ["t", "x", "y", "yaw"])
    f_tf, w_tf = open_csv("tf_filtered.csv", ["t", "parent", "child", "tx", "ty", "tz", "yaw"])
    f_scan, w_scan = open_csv("scan_summary.csv", ["t", "dt", "min_range", "mean_range", "num_ranges"])

    wanted = {"/cmd_vel_nav", "/cmd_vel", "/odom", "/amcl_pose", "/tf", "/tf_static", "/scan"}

    t0_ns = None
    last_scan_t = None

    try:
        while reader.has_next():
            topic, data, t_ns = reader.read_next()
            if topic not in wanted:
                continue

            if t0_ns is None:
                t0_ns = int(t_ns)
            t = (int(t_ns) - t0_ns) * 1e-9

            msg = deserialize_message(data, msg_types[topic])

            if topic == "/cmd_vel_nav":
                w_cmdnav.writerow([t, msg.linear.x, msg.angular.z])

            elif topic == "/cmd_vel":
                w_cmd.writerow([t, msg.linear.x, msg.angular.z])

            elif topic == "/odom":
                px = msg.pose.pose.position.x
                py = msg.pose.pose.position.y
                q = msg.pose.pose.orientation
                yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
                vx = msg.twist.twist.linear.x
                wz = msg.twist.twist.angular.z
                w_odom.writerow([t, px, py, yaw, vx, wz])

            elif topic == "/amcl_pose":
                px = msg.pose.pose.position.x
                py = msg.pose.pose.position.y
                q = msg.pose.pose.orientation
                yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
                w_amcl.writerow([t, px, py, yaw])

            elif topic in ("/tf", "/tf_static"):
                for tr in msg.transforms:
                    parent = tr.header.frame_id
                    child = tr.child_frame_id
                    # 只筛你最关心的几条链路（可按需加）
                    keep = (
                        (parent == "map" and child == "odom") or
                        (parent == "odom" and child in ("base_link", "base_footprint")) or
                        (parent in ("base_link", "base_footprint") and child == "base_scan")
                    )
                    if not keep:
                        continue
                    tt = tr.transform.translation
                    q = tr.transform.rotation
                    yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
                    w_tf.writerow([t, parent, child, tt.x, tt.y, tt.z, yaw])

            elif topic == "/scan":
                # ranges 太长，CSV 不适合直接存完整数组；先存摘要用于诊断“滞后/丢帧”
                dt = (t - last_scan_t) if last_scan_t is not None else ""
                last_scan_t = t

                ranges = [r for r in msg.ranges if math.isfinite(r) and r > 0.0]
                if ranges:
                    min_r = min(ranges)
                    mean_r = sum(ranges) / len(ranges)
                    n = len(msg.ranges)
                else:
                    min_r = ""
                    mean_r = ""
                    n = len(msg.ranges)

                w_scan.writerow([t, dt, min_r, mean_r, n])

    finally:
        for f in (f_cmdnav, f_cmd, f_odom, f_amcl, f_tf, f_scan):
            f.close()

    print("Bag:", bag_dir)
    print("CSV saved to:", out_dir)
    print("Files generated:")
    for p in sorted(out_dir.glob("*.csv")):
        print(" -", p.name)


if __name__ == "__main__":
    main()
