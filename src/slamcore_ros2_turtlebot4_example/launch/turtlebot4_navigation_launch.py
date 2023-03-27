"""
Launch the Nav2 framework for:

* The TurtleBot4 using an iRobot Create 3 robot platform
* with Slamcore Visual-Inertial or Visual-Inertial-Kinematic SLAM

"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import SetRemap
from slamcore_ros2_examples_common.configuration import Configuration
from slamcore_ros2_examples_common.utils import add_all_actions, launchfile_for


def generate_launch_description():
    cfg = Configuration()

    # get directories & paths -----------------------------------------------------------------
    pkg_slamcore_turtlebot4 = get_package_share_directory("slamcore_ros2_turtlebot4_example")

    # non-editable config variables -----------------------------------------------------------
    default_demo_params_file = PathJoinSubstitution(
        [
            pkg_slamcore_turtlebot4,
            "config",
            LaunchConfiguration("model"),
            "turtlebot4-nav2-demo-params.yaml",
        ]
    )
    default_namespace = ""

    # launch config variables -----------------------------------------------------------------
    namespace = default_namespace

    # launch arguments ------------------------------------------------------------------------
    cfg.register(
        "model",
        default_value="standard",
        choices=["standard", "lite"],
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )
    cfg.register(
        "params_file",
        default_value=default_demo_params_file,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )
    cfg.register("use_sim_time", default_value=False, description="Use sim time")

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
