"""Publish a reverse-engineered goal position as RViz/Foxglove markers,
expressed in the agent's own frame.

The bags don't contain the goal that was actually being pursued (it isn't
one of the recorded topics); it was worked out afterwards as a lat/lon (see
bags/reconstruct_goal_position.py). This node converts that lat/lon into the
'world' ENU frame produced by gps_origin_tf (anchored on the reference
vehicle's gp_origin), and then, on every odometry message, re-expresses it in
the agent's body frame and publishes:
  - a sphere at the goal
  - a line from the agent's origin (0, 0, 0) to the goal

Both markers are stamped in 'agent_frame', so they are valid agent-relative
coordinates rather than a fixed world point that depends on a viewer
resolving the whole TF chain.

Updates are driven by the odometry topic, not by a wall/sim timer, so the
markers advance in lockstep with the agent's recorded motion and carry the
odometry's own timestamp.

Nothing is written back to the bag; this only publishes visualization
markers.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from s3vo_analysis.gps_origin_tf import geodetic_to_local_enu


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GoalMarkerPublisher(Node):

    def __init__(self):
        super().__init__('goal_marker_publisher')

        self.declare_parameter('goal_latitude', 0.0)
        self.declare_parameter('goal_longitude', 0.0)
        self.declare_parameter('reference_gp_origin_topic',
                                '/usv/lily/mavros/global_position/gp_origin')
        self.declare_parameter('agent_odom_topic',
                                '/usv/lily/mavros/local_position/odom')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('agent_frame', 'base_link')
        self.declare_parameter('marker_topic', '/s3vo_analysis/goal_markers')
        self.declare_parameter('marker_scale', 2.0)
        self.declare_parameter('line_width', 0.3)

        self._goal_lat = self.get_parameter('goal_latitude').value
        self._goal_lon = self.get_parameter('goal_longitude').value
        self._world_frame = self.get_parameter('world_frame').value
        self._agent_frame = self.get_parameter('agent_frame').value
        self._marker_scale = self.get_parameter('marker_scale').value
        self._line_width = self.get_parameter('line_width').value

        self._goal_world = None  # (east, north) once the gp_origin is known
        self._warned_missing_tf = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, 10)

        latched = QoSProfile(depth=5)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            GeoPointStamped, self.get_parameter('reference_gp_origin_topic').value,
            self._on_reference_origin, latched)

        # The bag's mavros odom was recorded BEST_EFFORT; a BEST_EFFORT
        # subscription is compatible with both that and a RELIABLE publisher.
        odom_qos = QoSProfile(depth=10)
        odom_qos.history = QoSHistoryPolicy.KEEP_LAST
        odom_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            Odometry, self.get_parameter('agent_odom_topic').value,
            self._on_odom, odom_qos)

        self.get_logger().info(
            f"Waiting for reference gp_origin to place goal ({self._goal_lat}, "
            f"{self._goal_lon}); markers will be published in '{self._agent_frame}'")

    def _on_reference_origin(self, msg: GeoPointStamped):
        if self._goal_world is not None:
            return  # origin does not change once the EKF has initialized

        ref_lat = msg.position.latitude
        ref_lon = msg.position.longitude
        ref_alt = msg.position.altitude
        east, north, _ = geodetic_to_local_enu(
            self._goal_lat, self._goal_lon, ref_alt, ref_lat, ref_lon, ref_alt)
        self._goal_world = (east, north)
        self.get_logger().info(
            f"Goal placed in '{self._world_frame}' (east={east:.2f}, north={north:.2f})")

    def _world_from_odom_frame(self, odom_frame):
        """Static offset of the agent's odom frame within 'world', as
        (x, y, yaw). gps_origin_tf publishes this; identity is the right
        fallback for the reference vehicle, whose local frame *is* world."""
        if odom_frame == self._world_frame:
            return 0.0, 0.0, 0.0
        try:
            tf = self._tf_buffer.lookup_transform(
                self._world_frame, odom_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            if not self._warned_missing_tf:
                self._warned_missing_tf = True
                self.get_logger().warn(
                    f"No TF '{self._world_frame}' -> '{odom_frame}' yet; assuming "
                    f"identity. Run patch_bag_tf.launch.py for the real offset.")
            return 0.0, 0.0, 0.0
        return (tf.transform.translation.x,
                tf.transform.translation.y,
                yaw_from_quat(tf.transform.rotation))

    def _on_odom(self, msg: Odometry):
        if self._goal_world is None:
            return  # goal not placed yet; nothing meaningful to draw

        # Agent pose in 'world': compose the odom frame's static placement
        # with the agent's pose inside that frame.
        ox, oy, oyaw = self._world_from_odom_frame(msg.header.frame_id)
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        cos_o, sin_o = math.cos(oyaw), math.sin(oyaw)
        agent_x = ox + cos_o * px - sin_o * py
        agent_y = oy + sin_o * px + cos_o * py
        agent_yaw = oyaw + yaw_from_quat(msg.pose.pose.orientation)

        # Goal, world -> agent body frame (rotate by -agent_yaw).
        dx = self._goal_world[0] - agent_x
        dy = self._goal_world[1] - agent_y
        cos_a, sin_a = math.cos(agent_yaw), math.sin(agent_yaw)
        goal_point = Point(x=(cos_a * dx + sin_a * dy),
                           y=(-sin_a * dx + cos_a * dy),
                           z=0.0)

        stamp = msg.header.stamp
        markers = MarkerArray()

        goal_marker = Marker()
        goal_marker.header.frame_id = self._agent_frame
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
        goal_marker.frame_locked = True
        markers.markers.append(goal_marker)

        line_marker = Marker()
        line_marker.header.frame_id = self._agent_frame
        line_marker.header.stamp = stamp
        line_marker.ns = 'goal'
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = self._line_width
        line_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
        line_marker.frame_locked = True
        line_marker.points = [Point(x=0.0, y=0.0, z=0.0), goal_point]
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
