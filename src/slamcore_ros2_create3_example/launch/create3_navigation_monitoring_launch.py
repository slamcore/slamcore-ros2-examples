"""Enable Visualization of the SLAM and Navigation process via RViz."""
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from slamcore_ros2_examples_common.configuration import Configuration
from slamcore_ros2_examples_common.utils import add_all_actions, launchfile_for


def generate_launch_description():
    cfg = Configuration()

    # get directories & paths -----------------------------------------------------------------
    share_dir = get_package_share_path("slamcore_ros2_create3_example")

    # non-editable config variables -----------------------------------------------------------
    default_rviz_config_file = share_dir / "rviz" / "create3_navigation_monitoring.rviz"

    # launch arguments ------------------------------------------------------------------------
    cfg.register(
        "rviz_config_file",
        default_value=default_rviz_config_file,
        description="Full path to the RVIZ config file to use",
    )

    ld = LaunchDescription(
        [
            # launch arguments ----------------------------------------------------------------
            *cfg.launch_arguments(),
        ]
    )

    add_all_actions(
        # rviz --------------------------------------------------------------------------------
        IncludeLaunchDescription(
            launchfile_for("nav2_bringup", "rviz_launch.py"),
            launch_arguments={
                "namespace": "",
                "use_namespace": "false",
                "rviz_config": cfg.rviz_config_file.cfg,
            }.items(),
        ),
        ld=ld,
    )
    return ld
