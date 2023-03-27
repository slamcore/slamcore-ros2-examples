"""
Launch the Nav2 framework for:

* The iRobot Create 3 robot platform
* with Slamcore Visual-Inertial or Visual-Inertial-Kinematic SLAM

"""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch_ros.actions import SetRemap
from slamcore_ros2_examples_common.configuration import Configuration
from slamcore_ros2_examples_common.utils import add_all_actions, launchfile_for


def generate_launch_description():
    cfg = Configuration()

    # get directories & paths -----------------------------------------------------------------
    share_dir = get_package_share_path("slamcore_ros2_create3_example")

    # non-editable config variables -----------------------------------------------------------
    default_demo_params_file = share_dir / "config" / "create3-nav2-demo-params.yaml"
    default_namespace = ""

    # launch config variables -----------------------------------------------------------------
    namespace = default_namespace

    # launch arguments ------------------------------------------------------------------------
    cfg.register("use_sim_time", default_value=False, description="Use sim time")
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
        # Nav2 navigation ---------------------------------------------------------------------
        GroupAction(
            actions=[
                SetRemap(src="/cmd_vel", dst="/input/navigation"),
                IncludeLaunchDescription(
                    launchfile_for("nav2_bringup", "navigation_launch.py"),
                    launch_arguments={
                        "namespace": namespace,
                        "use_sim_time": cfg.use_sim_time.cfg,
                        "autostart": "true",
                        "params_file": cfg.params_file.cfg,
                    }.items(),
                ),
            ]
        ),
        ld=ld,
    )

    return ld
