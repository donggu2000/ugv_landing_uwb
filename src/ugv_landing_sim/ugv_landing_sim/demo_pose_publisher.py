import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class DemoPosePublisher(Node):
    def __init__(self):
        super().__init__("demo_pose_publisher")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("ugv_pose_topic", "/ugv/pose")
        self.declare_parameter("tag_pose_topic", "/x500/tag_pose")
        self.declare_parameter("publish_rate_hz", 30.0)

        self.frame_id = self.get_parameter("frame_id").value
        self.ugv_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter("ugv_pose_topic").value,
            10,
        )
        self.tag_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter("tag_pose_topic").value,
            10,
        )

        self.start_time = self.get_clock().now()
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / rate_hz, self._publish)

    def _publish(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9

        ugv = PoseStamped()
        ugv.header.frame_id = self.frame_id
        ugv.header.stamp = now.to_msg()
        ugv.pose.position.x = 0.15 * math.sin(0.15 * t)
        ugv.pose.position.y = 0.10 * math.cos(0.12 * t)
        ugv.pose.position.z = 0.0
        yaw = 0.08 * math.sin(0.1 * t)
        ugv.pose.orientation.z = math.sin(yaw / 2.0)
        ugv.pose.orientation.w = math.cos(yaw / 2.0)
        self.ugv_pub.publish(ugv)

        cycle = t % 18.0
        descent = min(cycle / 14.0, 1.0)
        tag = PoseStamped()
        tag.header.frame_id = self.frame_id
        tag.header.stamp = now.to_msg()
        tag.pose.position.x = ugv.pose.position.x + 1.2 * (1.0 - descent)
        tag.pose.position.y = ugv.pose.position.y - 0.8 * (1.0 - descent)
        tag.pose.position.z = 2.8 - 2.35 * descent
        tag.pose.orientation.w = 1.0
        self.tag_pub.publish(tag)


def main(args=None):
    rclpy.init(args=args)
    node = DemoPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
