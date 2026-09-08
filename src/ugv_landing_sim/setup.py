import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ugv_landing_sim"


def package_files(data_dir):
    paths = []
    for path in glob(os.path.join(data_dir, "**", "*"), recursive=True):
        if os.path.isfile(path):
            install_dir = os.path.join("share", package_name, os.path.dirname(path))
            paths.append((install_dir, [path]))
    return paths


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
    ] + package_files("models"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hyeon",
    maintainer_email="hyeon@example.com",
    description="UWB landing simulation scaffold for ROS 2 Humble and Gazebo Sim.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "demo_pose_publisher = ugv_landing_sim.demo_pose_publisher:main",
            "anchor_marker_publisher = ugv_landing_sim.anchor_marker_publisher:main",
            "dwm_tag_serial_publisher = ugv_landing_sim.dwm_tag_serial_publisher:main",
            "tag_pose_marker_publisher = ugv_landing_sim.tag_pose_marker_publisher:main",
            "udp_tag_receiver = ugv_landing_sim.udp_tag_receiver:main",
            "trilateration_estimator = ugv_landing_sim.trilateration_estimator:main",
            "uwb_range_simulator = ugv_landing_sim.uwb_range_simulator:main",
        ],
    },
)
