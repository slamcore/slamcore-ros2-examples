"""
Functions for initializing Nodes and Launchfiles without having to repeat their arguments.
"""


from launch.actions import IncludeLaunchDescription
from launch.actions.execute_process import LaunchConfiguration
from slamcore_ros2_examples_common.utils import launchfile_for


def static_transform_pub_launch(params_file: LaunchConfiguration):
    return IncludeLaunchDescription(
        launchfile_for("slamcore_ros2_examples_common", "static_transform_pub_launch.py"),
        launch_arguments={"params_file": params_file}.items(),
    )
