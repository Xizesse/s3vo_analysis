"""Publish a reverse-engineered goal position as RViz markers.

The bags don't contain the goal that was actually being pursued (it isn't
one of the recorded topics); it was worked out afterwards as a lat/lon.
This node converts that lat/lon into the same 'world' frame produced by
gps_origin_tf (anchored on the reference vehicle's gp_origin) and publishes
two markers while the bag plays:
  - a small sphere at the goal position
  - a line from the agent's current position (looked up via TF) to the goal

Nothing is written back to the bag; this only publishes visualization
markers.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPointStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from s3vo_analysis.gps_origin_tf import geodetic_to_local_enu


class GoalMarkerPublisher(Node):

    def __init__(self):
        super().__init__('goal_marker_publisher')

        self.declare_parameter('goal_latitude', 0.0)
        self.declare_parameter('goal_longitude', 0.0)
        self.declare_parameter('reference_gp_origin_topic',
                                '/usv/lily/mavros/global_position/gp_origin')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('agent_frame', 'base_link')
        self.declare_parameter('marker_topic', '/s3vo_analysis/goal_markers')
        self.declare_parameter('marker_scale', 2.0)
        self.declare_parameter('line_width', 0.3)
        self.declare_parameter('publish_rate_hz', 5.0)

        self._goal_lat = self.get_parameter('goal_latitude').value
        self._goal_lon = self.get_parameter('goal_longitude').value
        self._world_frame = self.get_parameter('world_frame').value
        self._agent_frame = self.get_parameter('agent_frame').value
        self._marker_scale = self.get_parameter('marker_scale').value
        self._line_width = self.get_parameter('line_width').value

        self._goal_local = None  # (east, north, up) once the origin is known

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, 10)

        qos = QoSProfile(depth=5)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            GeoPointStamped, self.get_parameter('reference_gp_origin_topic').value,
            self._on_reference_origin, qos)

        period = 1.0 / self.get_parameter('publish_rate_hz').value
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"Waiting for reference gp_origin to place goal ({self._goal_lat}, "
            f"{self._goal_lon}) in '{self._world_frame}'")

    def _on_reference_origin(self, msg: GeoPointStamped):
        if self._goal_local is not None:
            return  # origin does not change once the EKF has initialized

        ref_lat = msg.position.latitude
        ref_lon = msg.position.longitude
        ref_alt = msg.position.altitude
        east, north, _ = geodetic_to_local_enu(
            self._goal_lat, self._goal_lon, ref_alt, ref_lat, ref_lon, ref_alt)
        self._goal_local = (east, north, 0.0)
        self.get_logger().info(
            f"Goal placed at '{self._world_frame}' (east={east:.2f}, north={north:.2f})")

    def _on_timer(self):
        if self._goal_local is None:
            return

        goal_point = Point(x=self._goal_local[0], y=self._goal_local[1], z=self._goal_local[2])
        stamp = self.get_clock().now().to_msg()

        markers = MarkerArray()

        goal_marker = Marker()
        goal_marker.header.frame_id = self._world_frame
        goal_marker.header.stamp = stamp
        goal_marker.ns = 'goal'
        goal_marker.id = 0
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position = goal_point
        goal_marker.pose.orientation.w = 1.0
        goal_marker.scale.x = self._marker_scale
        goal_marker.scale.y = self._marker_scale
        goal_marker.scale.z = self._marker_scale
        goal_marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.9)
        markers.markers.append(goal_marker)

        try:
            transform = self._tf_buffer.lookup_transform(
                self._world_frame, self._agent_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            transform = None

        if transform is not None:
            agent_point = Point(
                x=transform.transform.translation.x,
                y=transform.transform.translation.y,
                z=transform.transform.translation.z)

            line_marker = Marker()
            line_marker.header.frame_id = self._world_frame
            line_marker.header.stamp = stamp
            line_marker.ns = 'goal'
            line_marker.id = 1
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.pose.orientation.w = 1.0
            line_marker.scale.x = self._line_width
            line_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
            line_marker.points = [agent_point, goal_point]
            markers.markers.append(line_marker)

        self._marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = GoalMarkerPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
