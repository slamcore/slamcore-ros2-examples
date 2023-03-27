"""
Call the generic joystick teleoperation launch file with the create3 params file.
"""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from slamcore_ros2_examples_common.configuration import Configuration
from slamcore_ros2_examples_common.utils import add_all_actions, launchfile_for


def generate_launch_description():
    cfg = Configuration()

    # get directories & paths -----------------------------------------------------------------
    share_dir = get_package_share_path("slamcore_ros2_examples_common")

    # non-editable config variables -----------------------------------------------------------
    default_joy_params_file = share_dir / "config" / "ps4-joy.yaml"

    # launch arguments ------------------------------------------------------------------------
    cfg.register(
        "params_file",
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
        # Common joy teleop launch file -------------------------------------------------------
        IncludeLaunchDescription(
            launchfile_for("slamcore_ros2_examples_common", "teleop_joy_launch.py"),
            launch_arguments={
                "params_file": cfg.params_file.cfg,
            }.items(),
        ),
        ld=ld,
    )

    return ld
