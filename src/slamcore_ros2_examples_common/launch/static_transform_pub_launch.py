"""Publish a static transformation which is parsed by the given configuration file."""
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node
from slamcore_ros2_examples_common.configuration import Configuration
from slamcore_ros2_examples_common.utils import add_all_actions


def generate_launch_description():
    cfg = Configuration()
    cfg.register(
        "params_file",
        default_value="",
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )
    cfg.register(
        "params_file_section",
        default_value="slamcore_camera_to_robot_base_transform",
        description=(
            "Name of the section in the YAML file to fetch the transform and reference frames"
            " from"
        ),
    )

    # assemble launch description -------------------------------------------------------------
    ld = LaunchDescription([*cfg.launch_arguments()])

    add_all_actions(
        Node(
            package="slamcore_ros2_examples_common",
            executable="static_transform_publisher_from_file",
            parameters=[
                {
                    "params_file": cfg.params_file.cfg,
                    "params_file_section": cfg.params_file_section.cfg,
                }
            ],
            on_exit=Shutdown(),
        ),
        ld=ld,
    )
    return ld
