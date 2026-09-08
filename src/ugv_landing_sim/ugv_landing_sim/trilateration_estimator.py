import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class TrilaterationEstimator(Node):
    def __init__(self):
        super().__init__("trilateration_estimator")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("ranges_topic", "/uwb/ranges")
        self.declare_parameter("estimate_topic", "/uwb/tag_pose_estimate")

        self.frame_id = self.get_parameter("frame_id").value
        ranges_topic = self.get_parameter("ranges_topic").value
        estimate_topic = self.get_parameter("estimate_topic").value

        self.pose_pub = self.create_publisher(PoseStamped, estimate_topic, 10)
        self.create_subscription(Float64MultiArray, ranges_topic, self._ranges_cb, 10)
        self.get_logger().info(f"Estimating tag pose from {ranges_topic} to {estimate_topic}")

    def _ranges_cb(self, msg: Float64MultiArray):
        if len(msg.data) < 16 or len(msg.data) % 4 != 0:
            self.get_logger().warn("Need at least 4 anchors, encoded as x,y,z,range groups")
            return

        rows = np.array(msg.data, dtype=float).reshape((-1, 4))
        anchors = rows[:, 0:3]
        ranges = rows[:, 3]

        p0 = anchors[0]
        r0 = ranges[0]
        a_rows = []
        b_rows = []
        for anchor, radius in zip(anchors[1:], ranges[1:]):
            a_rows.append(2.0 * (anchor - p0))
            b_rows.append(
                (r0 ** 2 - radius ** 2)
                - np.dot(p0, p0)
                + np.dot(anchor, anchor)
            )

        a_matrix = np.vstack(a_rows)
        b_vector = np.array(b_rows)

        try:
            estimate, residuals, rank, singular_values = np.linalg.lstsq(
                a_matrix,
                b_vector,
                rcond=None,
            )
        except np.linalg.LinAlgError as exc:
            self.get_logger().warn(f"Trilateration failed: {exc}")
            return

        if rank < 3 and np.max(np.abs(anchors[:, 2] - anchors[0, 2])) < 1e-6:
            estimate = self._solve_coplanar_above(anchors, ranges)
        elif rank < 3:
            self.get_logger().warn(
                "Anchor geometry is rank deficient; estimate may be unstable",
                throttle_duration_sec=2.0,
            )

        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(estimate[0])
        pose.pose.position.y = float(estimate[1])
        pose.pose.position.z = float(estimate[2])
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

    def _solve_coplanar_above(self, anchors: np.ndarray, ranges: np.ndarray) -> np.ndarray:
        p0 = anchors[0, 0:2]
        r0 = ranges[0]
        a_rows = []
        b_rows = []
        for anchor, radius in zip(anchors[1:, 0:2], ranges[1:]):
            a_rows.append(2.0 * (anchor - p0))
            b_rows.append(
                (r0 ** 2 - radius ** 2)
                - np.dot(p0, p0)
                + np.dot(anchor, anchor)
            )

        xy, residuals, rank, singular_values = np.linalg.lstsq(
            np.vstack(a_rows),
            np.array(b_rows),
            rcond=None,
        )
        dx = xy[0] - anchors[0, 0]
        dy = xy[1] - anchors[0, 1]
        z_delta_sq = max(0.0, r0 ** 2 - dx ** 2 - dy ** 2)
        z = anchors[0, 2] + float(np.sqrt(z_delta_sq))
        return np.array([xy[0], xy[1], z])


def main(args=None):
    rclpy.init(args=args)
    node = TrilaterationEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
