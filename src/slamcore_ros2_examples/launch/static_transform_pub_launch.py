"""Publish a static transformation which is parsed by the given configuration file."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import Shutdown
from slamcore_ros2_examples.utils import get_yaml_ros_params, logger


def generate_launch_description():
    if "SLAMCORE_CAMERA_LINK_FRAME_FILE" in os.environ:
        params_file = Path(os.environ["SLAMCORE_CAMERA_LINK_FRAME_FILE"])
        logger.warn(
            f"Reading custom Kobuki Base -> SLAMcore Camera calibration from {params_file}"
        )
    else:
        share_dir = get_package_share_path("slamcore_ros2_examples")
        params_file = share_dir / "config" / "nav2-demo-params.yaml"

    transform = get_yaml_ros_params(params_file, "slamcore_camera_to_robot_base_transform")
    parent_frame = transform["parent_frame"]
    child_frame = transform["child_frame"]
    xyz = transform["xyz"]
    rpy = transform["rpy"]

    ld = LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[*map(str, xyz), *map(str, rpy), parent_frame, child_frame],
                on_exit=Shutdown(),
            )
        ]
    )

    return ld
