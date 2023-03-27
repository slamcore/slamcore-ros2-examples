"""
Joystick Teleoperation Nodes
"""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node
from slamcore_ros2_examples_common.configuration import Configuration
from slamcore_ros2_examples_common.utils import add_all_actions


def generate_launch_description():
    cfg = Configuration()

    # get directories & paths -----------------------------------------------------------------
    share_dir = get_package_share_path("slamcore_ros2_examples_common")

    # non-editable config variables -----------------------------------------------------------
    default_joy_params_file = share_dir / "config" / "ps4-joy.yaml"

    # launch arguments ------------------------------------------------------------------------
    cfg.register(
        "joy_params_file",
        default_value=default_joy_params_file,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    # assemble launch description -------------------------------------------------------------
    ld = LaunchDescription(
        [
            # launch arguments ----------------------------------------------------------------
            *cfg.launch_arguments(),
        ]
    )

    # include launchfiles and nodes -----------------------------------------------------------
    add_all_actions(
        Node(
            package="joy_linux",
            executable="joy_linux_node",
            name="joy_linux_node",
            output="screen",
            parameters=[cfg.joy_params_file.cfg],
            on_exit=Shutdown(),
        ),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_node",
            output="screen",
            parameters=[cfg.joy_params_file.cfg],
            remappings=[("/cmd_vel", "/input/joy")],
            on_exit=Shutdown(),
        ),
        ld=ld,
    )

    return ld
