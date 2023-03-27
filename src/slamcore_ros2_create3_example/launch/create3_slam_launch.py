"""
Launch Slamcore Visual-Inertial or Visual-Inertial-Kinematic SLAM for:

* The iRobot Create 3 robot platform
* and the Nav2 framework

Optionally, on `use_rviz:=true` also launch an RViz visualization window to inspect the
navigation process.
"""

import xacro
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, Shutdown
from launch_ros.actions import Node
from slamcore_ros2_examples_common.configuration import Configuration
from slamcore_ros2_examples_common.shortcuts import static_transform_pub_launch
from slamcore_ros2_examples_common.utils import add_all_actions, launchfile_for


def generate_launch_description():
    cfg = Configuration()

    # get directories & paths -----------------------------------------------------------------
    share_dir = get_package_share_path("slamcore_ros2_create3_example")
    common_share_dir = get_package_share_path("slamcore_ros2_examples_common")

    # non-editable config variables -----------------------------------------------------------
    default_slam_config_file = share_dir / "config" / "create3-slam-config.json"
    default_demo_params_file = share_dir / "config" / "create3-nav2-demo-params.yaml"
    default_ros_extra_params_file = (
        common_share_dir / "config" / "slam-publisher-odom-override.yaml"
    )
    default_session_file = '""'
    default_namespace = ""

    # robot model
    xacro_file = share_dir / "descriptions" / "slamcore-create3.urdf.xacro"
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent="  ")  # type: ignore

    # launch config variables -----------------------------------------------------------------
    namespace = default_namespace

    # remappings ------------------------------------------------------------------------------
    # map fully qualified names to relative ones so the node's namespace can be prepended.
    # in case of the transforms (tf), currently, there doesn't seem to be a better alternative
    pub_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # launch arguments ------------------------------------------------------------------------
    cfg.register("use_sim_time", default_value=False, description="Use sim time")
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
        # robot state publisher (static transforms for robot model) ---------------------------
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=namespace,
            output="screen",
            parameters=[
                {"use_sim_time": cfg.use_sim_time.cfg, "robot_description": robot_desc}
            ],
            remappings=pub_remappings,
            on_exit=Shutdown(),
        ),
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
