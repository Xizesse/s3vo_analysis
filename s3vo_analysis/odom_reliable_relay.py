"""Bridge a best-effort Odometry topic to a reliable one.

avoa3d's avoa3dnode/rviz_marker subscribe to their odometry topic with
rclcpp's default QoS (RELIABLE, VOLATILE, KEEP_LAST(10)) and don't expose a
way to override it from parameters. The bags were recorded straight off
mavros' local_position/odom, which publishes BEST_EFFORT -- so when the bag
is played back, avoa3d's subscription is QoS-incompatible with it and never
receives anything.

This node just relays the Odometry message from a best-effort input topic
to a reliable output topic, unchanged, so avoa3d can subscribe to the
output topic instead. It never touches the bag or the original topic.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from nav_msgs.msg import Odometry


class OdomReliableRelay(Node):

    def __init__(self):
        super().__init__('odom_reliable_relay')

        self.declare_parameter('input_topic', '/odom')
        self.declare_parameter('output_topic', '/odom_reliable')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # Matches rclcpp's create_subscription<T>(topic, 10, cb) default.
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._publisher = self.create_publisher(Odometry, output_topic, reliable_qos)
        self.create_subscription(
            Odometry, input_topic, self._publisher.publish, qos_profile_sensor_data)

        self.get_logger().info(
            f"Relaying (best_effort) '{input_topic}' -> (reliable) '{output_topic}'")


def main(args=None):
    rclpy.init(args=args)
    node = OdomReliableRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
