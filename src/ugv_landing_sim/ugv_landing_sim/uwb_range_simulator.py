import math
import random
from typing import Iterable, List, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String
from visualization_msgs.msg import Marker, MarkerArray


Vector3 = Tuple[float, float, float]


def _chunks(values: Sequence[float], size: int) -> Iterable[Sequence[float]]:
    for start in range(0, len(values), size):
        yield values[start:start + size]


def _pose_xyz(pose: Pose) -> Vector3:
    return (
        pose.position.x,
        pose.position.y,
        pose.position.z,
    )


def _rotate_vector(pose: Pose, vector: Vector3) -> Vector3:
    qx = pose.orientation.x
    qy = pose.orientation.y
    qz = pose.orientation.z
    qw = pose.orientation.w

    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def _add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _distance(a: Vector3, b: Vector3) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


class UwbRangeSimulator(Node):
    def __init__(self):
        super().__init__("uwb_range_simulator")

        self.declare_parameter("frame_id", "world")
        self.declare_parameter("tag_pose_topic", "/x500/tag_pose")
        self.declare_parameter("ugv_pose_topic", "/ugv/pose")
        self.declare_parameter("ranges_topic", "/uwb/ranges")
        self.declare_parameter("decawave_topic", "/uwb/decawave_string")
        self.declare_parameter("marker_topic", "/uwb/markers")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("noise_stddev_m", 0.03)
        self.declare_parameter("range_bias_m", 0.0)
        self.declare_parameter("anchor_ids", ["5478", "2479", "4248", "f678"])
        self.declare_parameter(
            "anchor_positions_ugv",
            [
                0.85, 0.85, 0.35,
                0.85, -0.85, 0.35,
                -0.85, -0.85, 0.35,
                -0.85, 0.85, 0.35,
            ],
        )

        self.frame_id = self.get_parameter("frame_id").value
        self.anchor_ids = list(self.get_parameter("anchor_ids").value)
        flat_positions = [float(v) for v in self.get_parameter("anchor_positions_ugv").value]
        self.anchor_positions_ugv = [tuple(chunk) for chunk in _chunks(flat_positions, 3)]

        if len(self.anchor_ids) != len(self.anchor_positions_ugv):
            raise ValueError("anchor_ids and anchor_positions_ugv must have the same length")
        if len(self.anchor_positions_ugv) < 4:
            raise ValueError("3D trilateration needs at least 4 anchors")

        self.noise_stddev_m = float(self.get_parameter("noise_stddev_m").value)
        self.range_bias_m = float(self.get_parameter("range_bias_m").value)

        self.tag_pose = None
        self.ugv_pose = None

        tag_topic = self.get_parameter("tag_pose_topic").value
        ugv_topic = self.get_parameter("ugv_pose_topic").value
        ranges_topic = self.get_parameter("ranges_topic").value
        decawave_topic = self.get_parameter("decawave_topic").value
        marker_topic = self.get_parameter("marker_topic").value

        self.create_subscription(PoseStamped, tag_topic, self._tag_pose_cb, 10)
        self.create_subscription(PoseStamped, ugv_topic, self._ugv_pose_cb, 10)
        self.ranges_pub = self.create_publisher(Float64MultiArray, ranges_topic, 10)
        self.decawave_pub = self.create_publisher(String, decawave_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / rate_hz, self._publish_ranges)
        self.get_logger().info(
            f"Publishing simulated UWB ranges from {tag_topic} to {ranges_topic}"
        )

    def _tag_pose_cb(self, msg: PoseStamped):
        self.tag_pose = msg.pose

    def _ugv_pose_cb(self, msg: PoseStamped):
        self.ugv_pose = msg.pose

    def _anchor_world_positions(self) -> List[Vector3]:
        ugv_xyz = _pose_xyz(self.ugv_pose)
        return [
            _add(ugv_xyz, _rotate_vector(self.ugv_pose, anchor))
            for anchor in self.anchor_positions_ugv
        ]

    def _publish_ranges(self):
        if self.tag_pose is None or self.ugv_pose is None:
            self.get_logger().warn(
                "Waiting for both tag and UGV poses before publishing UWB ranges",
                throttle_duration_sec=2.0,
            )
            return

        stamp = self.get_clock().now().to_msg()
        tag_xyz = _pose_xyz(self.tag_pose)
        anchor_world = self._anchor_world_positions()
        data = []
        string_parts = []
        markers = MarkerArray()

        for index, (anchor_id, anchor_xyz) in enumerate(zip(self.anchor_ids, anchor_world)):
            measured_range = (
                _distance(anchor_xyz, tag_xyz)
                + self.range_bias_m
                + random.gauss(0.0, self.noise_stddev_m)
            )
            measured_range = max(0.0, measured_range)
            data.extend([anchor_xyz[0], anchor_xyz[1], anchor_xyz[2], measured_range])
            string_parts.append(
                f"{anchor_id}[{anchor_xyz[0]:.3f},{anchor_xyz[1]:.3f},{anchor_xyz[2]:.3f},{measured_range:.3f}]"
            )
            markers.markers.append(self._anchor_marker(index, anchor_id, anchor_xyz, stamp))

        range_msg = Float64MultiArray()
        range_msg.layout.dim = [
            MultiArrayDimension(label="anchors", size=len(anchor_world), stride=len(data)),
            MultiArrayDimension(label="x_y_z_range", size=4, stride=4),
        ]
        range_msg.data = data
        self.ranges_pub.publish(range_msg)

        decawave_msg = String()
        decawave_msg.data = " ".join(string_parts)
        self.decawave_pub.publish(decawave_msg)
        self.marker_pub.publish(markers)

    def _anchor_marker(self, index: int, anchor_id: str, xyz: Vector3, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = "uwb_anchors"
        marker.id = index
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = xyz[0]
        marker.pose.position.y = xyz[1]
        marker.pose.position.z = xyz[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.16
        marker.scale.y = 0.16
        marker.scale.z = 0.16
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.05
        marker.color.a = 1.0
        marker.text = anchor_id
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = UwbRangeSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
