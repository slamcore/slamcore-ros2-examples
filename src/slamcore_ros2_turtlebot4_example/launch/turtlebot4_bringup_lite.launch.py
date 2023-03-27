"""

Bring up TurtleBot 4 launch file, diagnostics, robot description and
cmd_vel_mux - to be used on boot. Teleop brought up separately.

"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from slamcore_ros2_examples_common.configuration import Configuration
from slamcore_ros2_examples_common.utils import add_all_actions, launchfile_for


def generate_launch_description():
    cfg = Configuration()

    # get directories & paths -----------------------------------------------------------------
    pkg_turtlebot4_bringup = get_package_share_directory("turtlebot4_bringup")

    # non-editable config variables -----------------------------------------------------------
    default_turtlebot4_params_file = PathJoinSubstitution(
        [
            pkg_turtlebot4_bringup,
            "config",
            "turtlebot4.yaml",
        ]
    )

    # launch arguments ------------------------------------------------------------------------
    cfg.register(
        "turtlebot4_params_file",
        default_value=default_turtlebot4_params_file,
        description="Turtlebot4 Robot param file",
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
        # TurtleBot 4 launch file -------------------------------------------------------
        IncludeLaunchDescription(
            launchfile_for("turtlebot4_bringup", "robot.launch.py"),
            launch_arguments={
                "model": "lite",
                "params_file": cfg.turtlebot4_params_file.cfg,
            }.items(),
        ),
        # Cmd Vel Mux launch file -------------------------------------------------------
        IncludeLaunchDescription(
            launchfile_for(
                "slamcore_ros2_turtlebot4_example", "turtlebot4_cmd_vel_mux_launch.py"
            ),
        ),
        # Robot Description launch file -------------------------------------------------------
        IncludeLaunchDescription(
            launchfile_for(
                "slamcore_ros2_turtlebot4_example", "turtlebot4_robot_description_launch.py"
            ),
            launch_arguments={"model": "lite"}.items(),
        ),
        # Diagnostics launch file -------------------------------------------------------
        IncludeLaunchDescription(
            launchfile_for("turtlebot4_diagnostics", "diagnostics.launch.py"),
        ),
        ld=ld,
    )

    return ld
