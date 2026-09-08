import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("ugv_landing_sim")
    params = LaunchConfiguration("params")
    use_rviz = LaunchConfiguration("use_rviz")
    use_tag = LaunchConfiguration("use_tag")
    use_udp_tag = LaunchConfiguration("use_udp_tag")
    use_pose_marker = LaunchConfiguration("use_pose_marker")
    tag_port = LaunchConfiguration("tag_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params",
                default_value=os.path.join(pkg_dir, "config", "bunker_uwb_params.yaml"),
                description="YAML file containing Bunker Pro UWB anchor positions.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the UWB anchor display.",
            ),
            DeclareLaunchArgument(
                "use_tag",
                default_value="true",
                description="Read the connected DWM1001 tag serial port and show x/y in RViz.",
            ),
            DeclareLaunchArgument(
                "use_udp_tag",
                default_value="false",
                description="Receive tag x/y from Jetson over UDP and publish ROS topics locally.",
            ),
            DeclareLaunchArgument(
                "use_pose_marker",
                default_value="false",
                description="Convert /uwb/tag_pose to markers when the pose source does not publish markers.",
            ),
            DeclareLaunchArgument(
                "tag_port",
                default_value="/dev/serial/by-id/usb-SEGGER_J-Link_000760217793-if00",
                description="Serial port for the DWM1001 tag.",
            ),
            Node(
                package="ugv_landing_sim",
                executable="anchor_marker_publisher",
                name="anchor_marker_publisher",
                parameters=[params],
                output="screen",
            ),
            Node(
                package="ugv_landing_sim",
                executable="dwm_tag_serial_publisher",
                name="dwm_tag_serial_publisher",
                parameters=[params, {"tag_port": tag_port}],
                condition=IfCondition(use_tag),
                output="screen",
            ),
            Node(
                package="ugv_landing_sim",
                executable="udp_tag_receiver",
                name="udp_tag_receiver",
                parameters=[params],
                condition=IfCondition(use_udp_tag),
                output="screen",
            ),
            Node(
                package="ugv_landing_sim",
                executable="tag_pose_marker_publisher",
                name="tag_pose_marker_publisher",
                condition=IfCondition(use_pose_marker),
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    os.path.join(pkg_dir, "config", "bunker_uwb.rviz"),
                ],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
