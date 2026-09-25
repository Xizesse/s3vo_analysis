"""Anchor every vehicle's local (mavros EKF) frame into one common 'world' frame.

Each vehicle's mavros local_position/odom is expressed relative to its own
EKF origin, published (a handful of times) on .../global_position/gp_origin
as a lat/lon/alt. Since each vehicle can set its origin independently, two
vehicles' 'map' frames are not the same frame even though they share the
name in the recorded messages.

This node listens to every vehicle's gp_origin topic, picks one vehicle as
the reference (its origin becomes the 'world' frame), and publishes static
transforms from 'world' to every other vehicle's local frame using a
flat-earth (equirectangular) approximation of the lat/lon offset. This is
adequate here since vehicles operate within a few hundred meters of each
other.

Nothing is written back to the bag; this only broadcasts TF while the bag
is being played.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import TransformStamped
from geographic_msgs.msg import GeoPointStamped
from tf2_ros import StaticTransformBroadcaster

EARTH_RADIUS_M = 6378137.0


def geodetic_to_local_enu(lat, lon, alt, lat0, lon0, alt0):
    """Flat-earth approximation of the ENU offset of (lat, lon, alt)
    relative to the reference (lat0, lon0, alt0), in meters."""
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    north = dlat * EARTH_RADIUS_M
    east = dlon * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    up = alt - alt0
    return east, north, up


class GpsOriginTf(Node):

    def __init__(self):
        super().__init__('gps_origin_tf')

        self.declare_parameter('vehicles', [''])
        self.declare_parameter('gp_origin_topics', [''])
        self.declare_parameter('local_frames', [''])
        self.declare_parameter('reference_vehicle', '')
        self.declare_parameter('world_frame', 'world')

        vehicles = list(self.get_parameter('vehicles').value)
        topics = list(self.get_parameter('gp_origin_topics').value)
        local_frames = list(self.get_parameter('local_frames').value)
        self._reference_vehicle = self.get_parameter('reference_vehicle').value
        self._world_frame = self.get_parameter('world_frame').value

        if not (len(vehicles) == len(topics) == len(local_frames)):
            raise RuntimeError(
                'vehicles, gp_origin_topics and local_frames must have the same length')
        if self._reference_vehicle not in vehicles:
            raise RuntimeError(
                f"reference_vehicle '{self._reference_vehicle}' not listed in 'vehicles'")

        self._local_frame_of = dict(zip(vehicles, local_frames))
        self._origins = {}
        self._published = set()

        self._static_broadcaster = StaticTransformBroadcaster(self)

        # gp_origin is typically published only a few times, latched-like.
        qos = QoSProfile(depth=5)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        for vehicle, topic in zip(vehicles, topics):
            self.create_subscription(
                GeoPointStamped, topic,
                lambda msg, v=vehicle: self._on_gp_origin(v, msg),
                qos)

        self.get_logger().info(
            f"Waiting for gp_origin of {vehicles} to anchor them into '{self._world_frame}' "
            f"(reference='{self._reference_vehicle}')")

    def _on_gp_origin(self, vehicle, msg: GeoPointStamped):
        if vehicle in self._origins:
            return  # origin does not change once the EKF has initialized

        self._origins[vehicle] = (
            msg.position.latitude, msg.position.longitude, msg.position.altitude)
        self.get_logger().info(f"Got gp_origin for '{vehicle}': {self._origins[vehicle]}")
        self._publish_pending()

    def _publish_pending(self):
        if self._reference_vehicle not in self._origins:
            return

        ref_lat, ref_lon, ref_alt = self._origins[self._reference_vehicle]

        for vehicle, local_frame in self._local_frame_of.items():
            if vehicle in self._published or vehicle not in self._origins:
                continue

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self._world_frame
            t.child_frame_id = local_frame

            if vehicle == self._reference_vehicle:
                east, north, up = 0.0, 0.0, 0.0
            else:
                lat, lon, alt = self._origins[vehicle]
                east, north, up = geodetic_to_local_enu(lat, lon, alt, ref_lat, ref_lon, ref_alt)

            t.transform.translation.x = east
            t.transform.translation.y = north
            t.transform.translation.z = up
            t.transform.rotation.w = 1.0  # local ENU axes assumed aligned (short baseline)

            self._static_broadcaster.sendTransform(t)
            self._published.add(vehicle)
            self.get_logger().info(
                f"Published static TF '{self._world_frame}' -> '{local_frame}' "
                f"(east={east:.2f}, north={north:.2f}, up={up:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = GpsOriginTf()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
