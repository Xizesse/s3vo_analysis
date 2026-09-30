"""Publish a vehicle's heading, extracted from its nav_msgs/Odometry topic.

The S3VO bags don't contain a heading topic for nautilus (no
global_position/compass_hdg was recorded), but its mavros
local_position/odom carries the EKF attitude. This node takes the yaw out
of that orientation and republishes it as a plain std_msgs/Float64, without
touching the bag.

Conventions ('convention' parameter):
  compass_deg -- degrees in [0, 360), 0 = North, clockwise positive
                 (same as mavros' global_position/compass_hdg). Default.
  enu_rad     -- radians in (-pi, pi], 0 = East, counter-clockwise positive
                 (the raw ENU yaw of mavros' local frame).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class OdomHeadingPublisher(Node):

    def __init__(self):
        super().__init__('odom_heading_publisher')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('heading_topic', '/heading')
        self.declare_parameter('convention', 'compass_deg')

        odom_topic = self.get_parameter('odom_topic').value
        heading_topic = self.get_parameter('heading_topic').value
        self._convention = self.get_parameter('convention').value
        if self._convention not in ('compass_deg', 'enu_rad'):
            raise ValueError(
                f"convention must be 'compass_deg' or 'enu_rad', got '{self._convention}'")

        self._publisher = self.create_publisher(Float64, heading_topic, 10)
        self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)

        self.get_logger().info(
            f"Publishing heading ({self._convention}) from '{odom_topic}' -> '{heading_topic}'")

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw_enu = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        out = Float64()
        if self._convention == 'enu_rad':
            out.data = yaw_enu
        else:
            out.data = (90.0 - math.degrees(yaw_enu)) % 360.0
        self._publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomHeadingPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
