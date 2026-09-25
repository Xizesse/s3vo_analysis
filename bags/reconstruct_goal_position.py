#!/usr/bin/env python3
"""
Reconstructs the goal position that go_to_gps_waypoint.cpp was steering toward
during the trial_* bags, from /usv/lily/desired_vel + /usv/lily/mavros/local_position/odom.

Why this works: the controller publishes a heading-tracking command, not a raw
position error:

    error_theta = theta_goal - yaw                      (theta_goal = world-frame
                                                           bearing from boat to goal)
    angular.z   = clamp(0.5 * sin(error_theta), +-0.6)   -> never saturates (max 0.5 < 0.6)
    linear.x    = speed * max(0, cos(error_theta))       -> sign of cos only, once clipped

So angular.z alone gives sin(error_theta) exactly, and sign(linear.x) gives the
quadrant (sign of cos(error_theta)). Together, atan2 recovers error_theta (and
therefore theta_goal = yaw + error_theta) exactly, without needing to know the
`speed` parameter at all.

Each (position(t), theta_goal(t)) sample defines a line that the (fixed, unknown)
goal point must lie on. Stacking many such lines from a moving boat gives a
classic bearings-only triangulation problem, linear in (x_g, y_g):

    sin(theta_goal_i)*x_g - cos(theta_goal_i)*y_g
        = sin(theta_goal_i)*pos_x_i - cos(theta_goal_i)*pos_y_i

Solved once by least squares over every trial bag pooled together (they all
share the same /usv/lily/mavros/global_position/gp_origin datum, confirmed
before pooling).

Requires: sourced ROS 2 environment (rosbag2_py, rclpy, nav_msgs, geometry_msgs,
geographic_msgs).
"""
import math

import numpy as np
import rosbag2_py
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

BAGS = [
    "trial_static_p_1",
    "trial_headon_p_1", "trial_headon_p_2", "trial_headon_p_3",
    "trial_headon_p_4", "trial_headon_p_5",
    "trial_stabord_p_1", "trial_stabord_p_2", "trial_stabord_p_3",
    "trial_stabord_p_4", "trial_stabord_p_5", "trial_stabord_p_6",
    "trial_stabord_p_8", "trial_stabord_p_9", "trial_stabord_p_10",
]

KP_YAW = 0.5  # hardcoded in go_to_gps_waypoint.cpp:164
DEG2RAD = math.pi / 180.0
SCALE = 111319.5  # gpsToEnu() in go_to_gps_waypoint.cpp:20


def read_topic(bag_dir, topic, msgtype):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    out = []
    while reader.has_next():
        _, data, t = reader.read_next()
        out.append((t, deserialize_message(data, msgtype)))
    return out


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    A_rows, b_rows = [], []
    origins = set()

    for bag in BAGS:
        origin_msgs = read_topic(bag, "/usv/lily/mavros/global_position/gp_origin", GeoPointStamped)
        _, om = origin_msgs[0]
        origins.add((round(om.position.latitude, 7), round(om.position.longitude, 7)))

        odom = read_topic(bag, "/usv/lily/mavros/local_position/odom", Odometry)
        dvel = read_topic(bag, "/usv/lily/desired_vel", TwistStamped)
        odom_t = np.array([t for t, _ in odom])

        for t, m in dvel:
            lin_x, ang_z = m.twist.linear.x, m.twist.angular.z
            if lin_x == 0.0 and ang_z == 0.0:
                continue  # goal-reached dead zone: no bearing info

            idx = np.searchsorted(odom_t, t, side="right") - 1
            if idx < 0:
                continue
            _, om = odom[idx]
            px, py = om.pose.pose.position.x, om.pose.pose.position.y
            yaw = yaw_from_quat(om.pose.pose.orientation)

            sin_val = max(-1.0, min(1.0, ang_z / KP_YAW))
            cos_sign = 1.0 if lin_x > 1e-9 else -1.0
            cos_val = cos_sign * math.sqrt(max(0.0, 1.0 - sin_val * sin_val))
            error_theta = math.atan2(sin_val, cos_val)
            theta_goal = yaw + error_theta

            s, c = math.sin(theta_goal), math.cos(theta_goal)
            A_rows.append([s, -c])
            b_rows.append(s * px - c * py)

    assert len(origins) == 1, f"bags do not share a single ENU origin: {origins}"
    lat0, lon0 = next(iter(origins))

    A, b = np.array(A_rows), np.array(b_rows)
    (xg, yg), *_ = np.linalg.lstsq(A, b, rcond=None)
    resid_rms = float(np.sqrt(np.mean((A @ [xg, yg] - b) ** 2)))

    lat_est = lat0 + yg / SCALE
    lon_est = lon0 + xg / (math.cos(lat0 * DEG2RAD) * SCALE)

    print(f"rays used: {len(A)}")
    print(f"goal local ENU (origin lat={lat0}, lon={lon0}): x={xg:.3f} m, y={yg:.3f} m")
    print(f"residual RMS: {resid_rms:.6f}")
    print(f"goal GPS: lat={lat_est:.7f}, lon={lon_est:.7f}")


if __name__ == "__main__":
    main()
