"""
Launch Slamcore Visual-Inertial or Visual-Inertial-Kinematic SLAM for:

* The TurtleBot4 using an iRobot Create 3 robot platform
* and the Nav2 framework

"""

from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from slamcore_ros2_examples_common.configuration import Configuration
from slamcore_ros2_examples_common.shortcuts import static_transform_pub_launch
from slamcore_ros2_examples_common.utils import add_all_actions, launchfile_for


def generate_launch_description():
    cfg = Configuration()

    # get directories & paths -----------------------------------------------------------------
    pkg_slamcore_turtlebot4 = get_package_share_directory("slamcore_ros2_turtlebot4_example")
    common_share_dir = get_package_share_path("slamcore_ros2_examples_common")

    # non-editable config variables -----------------------------------------------------------
    default_slam_config_file = PathJoinSubstitution(
        [
            pkg_slamcore_turtlebot4,
            "config",
            LaunchConfiguration("model"),
            "turtlebot4-slam-config.json",
        ]
    )
    default_demo_params_file = PathJoinSubstitution(
        [
            pkg_slamcore_turtlebot4,
            "config",
            LaunchConfiguration("model"),
            "turtlebot4-nav2-demo-params.yaml",
        ]
    )
    default_ros_extra_params_file = (
        common_share_dir / "config" / "slam-publisher-odom-override.yaml"
    )
    default_session_file = '""'

    # launch arguments ------------------------------------------------------------------------
    cfg.register(
        "model",
        default_value="standard",
        choices=["standard", "lite"],
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )
    cfg.register(
        "config_file",
        default_value=default_slam_config_file,
        description="Full path to the Slamcore SLAM configuration file",
    )
    cfg.register(
        "params_file",
        default_value=default_demo_params_file,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )
    cfg.register(
        "ros_extra_params",
        default_value=default_ros_extra_params_file,
        description=(
            "Full path to the ROS2 extra params file used to override the SLAM node's wheel"
            " odometry QoS settings"
        ),
    )
    cfg.register(
        "session_file",
        default_value=default_session_file,
        description=(
            "Path to Slamcore session file to load. "
            "If unset, the software will not operate in Multisession mode"
        ),
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
        # Slamcore SLAM -----------------------------------------------------------------------
        IncludeLaunchDescription(
            launchfile_for("slamcore_slam", "slam_publisher.launch.py"),
            launch_arguments={
                "config_file": cfg.config_file.cfg,
                "generate_map2d": "true",
                "odom_reading_topic": "/odom",
                "ros_extra_params": cfg.ros_extra_params.cfg,
                "override_realsense_depth": "true",
                "realsense_depth_override_value": "true",
                "session_file": cfg.session_file.cfg,
                # frames of ref
                "base_frame": "slamcore/base_link",
                "odom_frame": "odom",
                "map_frame": "map",
            }.items(),
        ),
        # Slamcore camera <-> robot base transformation - Edit the transform in the params file
        static_transform_pub_launch(cfg.params_file.cfg),
        ld=ld,
    )

    return ld
