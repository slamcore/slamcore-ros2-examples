"""
Set up cmd vel mux for Create 3 for selecting among a number of incoming
Twist messages and publishing the highest priority one to avoid conflicts.
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
    share_dir = get_package_share_path("slamcore_ros2_create3_example")

    # non-editable config variables -----------------------------------------------------------
    default_demo_params_file = share_dir / "config" / "create3-nav2-demo-params.yaml"

    # launch arguments ------------------------------------------------------------------------
    cfg.register(
        "params_file",
        default_value=default_demo_params_file,
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
            package="cmd_vel_mux",
            executable="cmd_vel_mux_node",
            name="cmd_vel_mux_node",
            output="screen",
            parameters=[cfg.params_file.cfg],
            on_exit=Shutdown(),
        ),
        ld=ld,
    )

    return ld
