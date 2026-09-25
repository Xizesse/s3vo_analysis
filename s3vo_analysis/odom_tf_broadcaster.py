"""Republish a nav_msgs/Odometry topic as a TF transform.

The S3VO bags were recorded without TF (mavros' local_position plugin had
TF publishing disabled), but they do contain the equivalent Odometry
messages. This node just turns those Odometry messages back into the
odom -> base_link transform that was missing, without touching the bag.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):

    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('parent_frame', '')
        self.declare_parameter('child_frame', '')

        self._odom_topic = self.get_parameter('odom_topic').value
        self._parent_frame_override = self.get_parameter('parent_frame').value
        self._child_frame_override = self.get_parameter('child_frame').value

        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, self._odom_topic, self._on_odom, qos_profile_sensor_data)

        self.get_logger().info(
            f"Broadcasting TF from '{self._odom_topic}' "
            f"(parent override='{self._parent_frame_override or '<msg frame_id>'}', "
            f"child override='{self._child_frame_override or '<msg child_frame_id>'}')")

    def _on_odom(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self._parent_frame_override or msg.header.frame_id
        t.child_frame_id = self._child_frame_override or msg.child_frame_id

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self._broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
