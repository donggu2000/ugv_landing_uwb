import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("ugv_landing_sim")
    params = os.path.join(pkg_dir, "config", "uwb_params.yaml")
    use_demo_poses = LaunchConfiguration("use_demo_poses")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_demo_poses",
                default_value="true",
                description="Publish demo /ugv/pose and /x500/tag_pose topics.",
            ),
            Node(
                package="ugv_landing_sim",
                executable="demo_pose_publisher",
                name="demo_pose_publisher",
                parameters=[params],
                condition=IfCondition(use_demo_poses),
                output="screen",
            ),
            Node(
                package="ugv_landing_sim",
                executable="uwb_range_simulator",
                name="uwb_range_simulator",
                parameters=[params],
                output="screen",
            ),
            Node(
                package="ugv_landing_sim",
                executable="trilateration_estimator",
                name="trilateration_estimator",
                parameters=[params],
                output="screen",
            ),
        ]
    )
