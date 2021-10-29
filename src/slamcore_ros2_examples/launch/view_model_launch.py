"""
Visualize the transforms of a URDF/Xacro model along with the Odometry Base <-> SLAMcore
estimation frame transformation.
"""
from pathlib import Path
from typing import cast

import xacro
from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions.include_launch_description import IncludeLaunchDescription
from slamcore_ros2_examples.configuration import Configuration
from slamcore_ros2_examples.utils import add_all_actions, launchfile_for
from slamcore_ros2_examples.xacro_file_contents import XacroFileContents


def generate_launch_description():
    cfg = Configuration()

    # get directories & paths -----------------------------------------------------------------
    share_dir = get_package_share_path("slamcore_ros2_examples")

    # non-editable config variables -----------------------------------------------------------
    use_sim_time = "false"
    default_rviz_config_file = share_dir / "rviz" / "model.rviz"
    default_urdf_file = share_dir / "descriptions" / "slamcore-kobuki.urdf.xacro"

    # remappings ------------------------------------------------------------------------------
    # map fully qualified names to relative ones so the node's namespace can be prepended.
    # in case of the transforms (tf), currently, there doesn't seem to be a better alternative
    pub_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # launch arguments ------------------------------------------------------------------------
    cfg.register(
        "rviz_config_file",
        default_value=default_rviz_config_file,
        description="Full path to the RVIZ config file to use",
    )
    cfg.register(
        "urdf_file",
        default_value=default_urdf_file,
        description="Path to the URDF file to visualize in rviz",
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
        # rviz --------------------------------------------------------------------------------
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", cfg.rviz_config_file.cfg],
            output="screen",
        ),
        IncludeLaunchDescription(
            launchfile_for("slamcore_ros2_examples", "static_transform_pub_launch.py"),
        ),
        # robot stater publisher (static transforms for robot model) --------------------------
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "robot_description": XacroFileContents(cfg.urdf_file.cfg),
                }
            ],
            remappings=pub_remappings,
        ),
        # robot stater publisher (static transforms for robot model) --------------------------
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "robot_description": XacroFileContents(cfg.urdf_file.cfg),
                    "delta": 0.05,
                }
            ],
            remappings=pub_remappings,
        ),
        ld=ld,
    )

    return ld
