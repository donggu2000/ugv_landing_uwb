from typing import Iterable, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


Vector3 = Tuple[float, float, float]


def _chunks(values: Sequence[float], size: int) -> Iterable[Sequence[float]]:
    for start in range(0, len(values), size):
        yield values[start:start + size]


class AnchorMarkerPublisher(Node):
    def __init__(self):
        super().__init__("anchor_marker_publisher")

        self.declare_parameter("frame_id", "bunker_base")
        self.declare_parameter("marker_topic", "/uwb/anchor_markers")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("anchor_ids", ["A0", "A1", "A2", "A3"])
        self.declare_parameter(
            "anchor_names",
            ["front_left", "front_right", "rear_right", "rear_left"],
        )
        self.declare_parameter("anchor_ports", ["", "", "", ""])
        self.declare_parameter(
            "anchor_positions",
            [
                0.5, 0.5, 0.30,
                0.5, -0.5, 0.30,
                -0.5, -0.5, 0.30,
                -0.5, 0.5, 0.30,
            ],
        )
        self.declare_parameter("anchor_radius_m", 0.07)
        self.declare_parameter("label_z_offset_m", 0.16)
        self.declare_parameter("draw_anchor_square", True)

        self.frame_id = self.get_parameter("frame_id").value
        self.marker_topic = self.get_parameter("marker_topic").value
        self.anchor_ids = [str(v) for v in self.get_parameter("anchor_ids").value]
        self.anchor_names = [str(v) for v in self.get_parameter("anchor_names").value]
        self.anchor_ports = [str(v) for v in self.get_parameter("anchor_ports").value]
        self.anchor_radius_m = float(self.get_parameter("anchor_radius_m").value)
        self.label_z_offset_m = float(self.get_parameter("label_z_offset_m").value)
        self.draw_anchor_square = bool(self.get_parameter("draw_anchor_square").value)

        flat_positions = [
            float(v) for v in self.get_parameter("anchor_positions").value
        ]
        self.anchor_positions = [
            tuple(chunk) for chunk in _chunks(flat_positions, 3)
        ]

        self._validate_config()

        self.publisher = self.create_publisher(MarkerArray, self.marker_topic, 10)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / publish_rate_hz, self._publish_markers)

        self.get_logger().info(
            f"Publishing {len(self.anchor_positions)} UWB anchors on {self.marker_topic} "
            f"in frame '{self.frame_id}'"
        )

    def _validate_config(self):
        if len(self.anchor_positions) < 1:
            raise ValueError("anchor_positions must contain at least one x,y,z group")
        if len(self.anchor_ids) != len(self.anchor_positions):
            raise ValueError("anchor_ids and anchor_positions must have the same length")
        if len(self.anchor_names) != len(self.anchor_positions):
            raise ValueError("anchor_names and anchor_positions must have the same length")
        if len(self.anchor_ports) != len(self.anchor_positions):
            raise ValueError("anchor_ports and anchor_positions must have the same length")

    def _publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        for index, (anchor_id, anchor_name, xyz) in enumerate(
            zip(self.anchor_ids, self.anchor_names, self.anchor_positions)
        ):
            markers.markers.append(self._anchor_marker(index, xyz, stamp))
            markers.markers.append(
                self._label_marker(index + 100, anchor_id, anchor_name, xyz, stamp)
            )

        if self.draw_anchor_square and len(self.anchor_positions) >= 3:
            markers.markers.append(self._square_marker(500, stamp))

        self.publisher.publish(markers)

    def _anchor_marker(self, marker_id: int, xyz: Vector3, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = "uwb_anchor_points"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = xyz[0]
        marker.pose.position.y = xyz[1]
        marker.pose.position.z = xyz[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.anchor_radius_m * 2.0
        marker.scale.y = self.anchor_radius_m * 2.0
        marker.scale.z = self.anchor_radius_m * 2.0
        marker.color.r = 1.0
        marker.color.g = 0.18
        marker.color.b = 0.08
        marker.color.a = 1.0
        return marker

    def _label_marker(
        self,
        marker_id: int,
        anchor_id: str,
        anchor_name: str,
        xyz: Vector3,
        stamp,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = "uwb_anchor_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = xyz[0]
        marker.pose.position.y = xyz[1]
        marker.pose.position.z = xyz[2] + self.label_z_offset_m
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.10
        marker.color.r = 0.95
        marker.color.g = 0.95
        marker.color.b = 0.95
        marker.color.a = 1.0
        marker.text = f"{anchor_id} {anchor_name}"
        return marker

    def _square_marker(self, marker_id: int, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = "uwb_anchor_layout"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.025
        marker.color.r = 0.15
        marker.color.g = 0.72
        marker.color.b = 1.0
        marker.color.a = 0.9

        closed_positions = list(self.anchor_positions) + [self.anchor_positions[0]]
        for xyz in closed_positions:
            point = Point()
            point.x = xyz[0]
            point.y = xyz[1]
            point.z = xyz[2]
            marker.points.append(point)

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = AnchorMarkerPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
