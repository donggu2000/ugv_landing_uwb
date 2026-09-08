import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class TagPoseMarkerPublisher(Node):
    def __init__(self):
        super().__init__("tag_pose_marker_publisher")

        self.declare_parameter("pose_topic", "/uwb/tag_pose")
        self.declare_parameter("marker_topic", "/uwb/tag_markers")
        self.declare_parameter("tag_radius_m", 0.11)
        self.declare_parameter("label_z_offset_m", 0.20)

        self.pose_topic = self.get_parameter("pose_topic").value
        self.marker_topic = self.get_parameter("marker_topic").value
        self.tag_radius_m = float(self.get_parameter("tag_radius_m").value)
        self.label_z_offset_m = float(self.get_parameter("label_z_offset_m").value)

        self.publisher = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.subscription = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self._publish_marker,
            10,
        )
        self.last_log_time = 0.0

        self.get_logger().info(
            f"Converting {self.pose_topic} to RViz markers on {self.marker_topic}"
        )

    def _publish_marker(self, pose_msg: PoseStamped):
        xyz = (
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            pose_msg.pose.position.z,
        )
        stamp = pose_msg.header.stamp
        frame_id = pose_msg.header.frame_id or "bunker_base"

        markers = MarkerArray()
        markers.markers.append(self._tag_marker(frame_id, stamp, xyz))
        markers.markers.append(self._label_marker(frame_id, stamp, xyz))
        self.publisher.publish(markers)

        now = time.monotonic()
        if now - self.last_log_time >= 5.0:
            self.get_logger().info(
                f"Publishing tag marker x={xyz[0]:.3f} y={xyz[1]:.3f} z={xyz[2]:.3f}"
            )
            self.last_log_time = now

    def _tag_marker(self, frame_id, stamp, xyz) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "uwb_tag_pose"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = xyz[0]
        marker.pose.position.y = xyz[1]
        marker.pose.position.z = xyz[2] + 0.04
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.tag_radius_m * 2.0
        marker.scale.y = self.tag_radius_m * 2.0
        marker.scale.z = self.tag_radius_m * 2.0
        marker.color.r = 0.05
        marker.color.g = 0.35
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker

    def _label_marker(self, frame_id, stamp, xyz) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "uwb_tag_pose_label"
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = xyz[0]
        marker.pose.position.y = xyz[1]
        marker.pose.position.z = xyz[2] + self.label_z_offset_m
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.12
        marker.color.r = 0.75
        marker.color.g = 0.88
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f"TAG x={xyz[0]:.2f} y={xyz[1]:.2f}"
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = TagPoseMarkerPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
