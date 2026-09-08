import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, TextSubstitution


def generate_launch_description():
    pkg_dir = get_package_share_directory("ugv_landing_sim")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")
    models_path = os.path.join(pkg_dir, "models")
    world_path = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(pkg_dir, "worlds", "ugv_landing.sdf"),
                description="Gazebo Sim world file.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start Gazebo GUI. Set false for headless server tests.",
            ),
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=[
                    models_path,
                    TextSubstitution(text=":"),
                    EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[
                    models_path,
                    TextSubstitution(text=":"),
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
                ),
                launch_arguments={"gz_args": ["-r ", world_path]}.items(),
                condition=IfCondition(gui),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
                ),
                launch_arguments={"gz_args": ["-s -r ", world_path]}.items(),
                condition=UnlessCondition(gui),
            ),
        ]
    )
