#!/usr/bin/env python3
"""Dump the topics used by the report analysis from each trial bag to CSV.

Output: <out_dir>/<bag>/<topic_name>.csv, one row per message, time in seconds
(bag receive time). Requires a sourced ROS 2 + workspace (for avoa3d msgs).

Usage: python3 extract_bags.py <bags_root> <out_dir> [bag ...]
"""
import csv
import math
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def odom_row(m):
    p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
    return dict(x=p.x, y=p.y, yaw=yaw(q), vx=v.x, vy=v.y)


def twist_row(m):
    return dict(vx=m.twist.linear.x, vy=m.twist.linear.y, wz=m.twist.angular.z)


def float_row(m):
    return dict(v=m.data)


def fix_row(m):
    return dict(lat=m.latitude, lon=m.longitude)


def origin_row(m):
    return dict(lat=m.position.latitude, lon=m.position.longitude)


def elements_rows(m):
    return [dict(id=e.id, dynamic=int(e.dynamic), x=e.pose.position.x, y=e.pose.position.y,
                 vx=e.velocity.x, vy=e.velocity.y, r=e.size.x, pz=e.protective_zone)
            for e in m.elements]


TOPICS = {
    "/usv/lily/mavros/local_position/odom": ("lily_odom", odom_row),
    "/usv/nautilus/mavros/local_position/odom": ("naut_odom", odom_row),
    "/usv/lily/mavros/global_position/global": ("lily_gps", fix_row),
    "/usv/nautilus/mavros/global_position/global": ("naut_gps", fix_row),
    "/usv/lily/mavros/global_position/gp_origin": ("lily_origin", origin_row),
    "/usv/nautilus/mavros/global_position/gp_origin": ("naut_origin", origin_row),
    "/usv/lily/desired_vel": ("desired_vel", twist_row),
    "/usv/lily/cmd_vel": ("cmd_vel", twist_row),
    "/usv/lily/left/commands/motor/thrust": ("thrust_left", float_row),
    "/usv/lily/right/commands/motor/thrust": ("thrust_right", float_row),
    "/element_tracking/elements": ("elements", elements_rows),
}


def extract(bag_dir, out_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    rows = {name: [] for name, _ in TOPICS.values()}
    classes = {t: get_message(types[t]) for t in TOPICS if t in types}
    while reader.has_next():
        topic, data, t = reader.read_next()
        name, fn = TOPICS[topic]
        r = fn(deserialize_message(data, classes[topic]))
        for rr in (r if isinstance(r, list) else [r]):
            rows[name].append(dict(t=t * 1e-9, **rr))
    os.makedirs(out_dir, exist_ok=True)
    for name, rs in rows.items():
        if not rs:
            continue
        with open(os.path.join(out_dir, name + ".csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rs[0].keys()))
            w.writeheader()
            w.writerows(rs)


def main():
    root, out = sys.argv[1], sys.argv[2]
    bags = sys.argv[3:] or sorted(d for d in os.listdir(root) if d.startswith("trial_"))
    for b in bags:
        print("extracting", b)
        extract(os.path.join(root, b), os.path.join(out, b))


if __name__ == "__main__":
    main()
