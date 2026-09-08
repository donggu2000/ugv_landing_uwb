import json
import socket
import threading
from typing import Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


Vector3 = Tuple[float, float, float]


class UdpTagReceiver(Node):
    def __init__(self):
        super().__init__("udp_tag_receiver")

        self.declare_parameter("frame_id", "bunker_base")
        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("udp_port", 5005)
        self.declare_parameter("pose_topic", "/uwb/tag_pose")
        self.declare_parameter("marker_topic", "/uwb/tag_markers")
        self.declare_parameter("tag_radius_m", 0.08)
        self.declare_parameter("label_z_offset_m", 0.16)

        self.frame_id = self.get_parameter("frame_id").value
        self.listen_host = self.get_parameter("listen_host").value
        self.udp_port = int(self.get_parameter("udp_port").value)
        self.tag_radius_m = float(self.get_parameter("tag_radius_m").value)
        self.label_z_offset_m = float(self.get_parameter("label_z_offset_m").value)

        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter("pose_topic").value,
            10,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter("marker_topic").value,
            10,
        )
        self.received_count = 0

        self.stop_event = threading.Event()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(0.2)
        self.socket.bind((self.listen_host, self.udp_port))
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()

        self.get_logger().info(
            f"Listening for UDP tag poses on {self.listen_host}:{self.udp_port}"
        )

    def destroy_node(self):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.socket.close()
        super().destroy_node()

    def _receive_loop(self):
        while not self.stop_event.is_set():
            try:
                packet, address = self.socket.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError as exc:
                if not self.stop_event.is_set():
                    self.get_logger().warn(f"UDP receive failed: {exc}")
                return

            try:
                data = json.loads(packet.decode("utf-8"))
                xyz = (float(data["x"]), float(data["y"]), float(data["z"]))
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                self.get_logger().warn(
                    f"Ignoring invalid UDP packet from {address}: {exc}",
                    throttle_duration_sec=2.0,
                )
                continue

            self.received_count += 1
            if self.received_count == 1:
                self.get_logger().info(
                    f"Received first UDP tag pose from {address}: "
                    f"x={xyz[0]:.3f}, y={xyz[1]:.3f}, z={xyz[2]:.3f}"
                )
            self._publish_pose_and_marker(xyz)

    def _publish_pose_and_marker(self, xyz: Vector3):
        stamp = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = xyz[0]
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        markers = MarkerArray()
        markers.markers.append(self._tag_marker(xyz, stamp))
        markers.markers.append(self._label_marker(xyz, stamp))
        self.marker_pub.publish(markers)

    def _tag_marker(self, xyz: Vector3, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = "uwb_tag_udp"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = xyz[0]
        marker.pose.position.y = xyz[1]
        marker.pose.position.z = xyz[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.tag_radius_m * 2.0
        marker.scale.y = self.tag_radius_m * 2.0
        marker.scale.z = self.tag_radius_m * 2.0
        marker.color.r = 0.1
        marker.color.g = 0.35
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker

    def _label_marker(self, xyz: Vector3, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = "uwb_tag_udp_label"
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = xyz[0]
        marker.pose.position.y = xyz[1]
        marker.pose.position.z = xyz[2] + self.label_z_offset_m
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.10
        marker.color.r = 0.8
        marker.color.g = 0.9
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f"TAG x={xyz[0]:.2f} y={xyz[1]:.2f}"
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = UdpTagReceiver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
