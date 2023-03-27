"""
Visualize the transforms of the Kobuki URDF/Xacro model along with the Odometry Base <-> Slamcore
estimation frame transformation.
"""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from slamcore_ros2_examples_common.configuration import Configuration
from slamcore_ros2_examples_common.utils import add_all_actions, launchfile_for


def generate_launch_description():
    cfg = Configuration()

    # get directories & paths -----------------------------------------------------------------
    share_dir = get_package_share_path("slamcore_ros2_kobuki_example")

    # non-editable config variables -----------------------------------------------------------
    default_demo_params_file = share_dir / "config" / "nav2-demo-params.yaml"
    default_urdf_file = share_dir / "descriptions" / "slamcore-kobuki.urdf.xacro"

    # launch arguments ------------------------------------------------------------------------
    cfg.register(
        "params_file",
        default_value=default_demo_params_file,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
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
        # Common view model launch file -------------------------------------------------------
        IncludeLaunchDescription(
            launchfile_for("slamcore_ros2_examples_common", "view_model_launch.py"),
            launch_arguments={
                "params_file": cfg.params_file.cfg,
                "urdf_file": cfg.urdf_file.cfg,
            }.items(),
        ),
        ld=ld,
    )

    return ld
