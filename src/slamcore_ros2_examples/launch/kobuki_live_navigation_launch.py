"""
Launch live navigation using:

* The kobuki robot platform
* SLAMcore Visual-Inertial or Visual-Inertial-Kinematic SLAM
* Nav2 framework

Optionally, on `use_rviz:=true` also launch an RViz visualization window to inspect the
navigation process.
"""

from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from slamcore_ros2_examples.configuration import Configuration
from slamcore_ros2_examples.utils import add_all_actions, launchfile_for


def generate_launch_description():
    cfg = Configuration()

    # get directories & paths -----------------------------------------------------------------
    share_dir = get_package_share_path("slamcore_ros2_examples")

    # non-editable config variables -----------------------------------------------------------
    use_sim_time = "false"
    default_slam_config_file = share_dir / "config" / "slam-config.json"
    default_demo_params_file = share_dir / "config" / "nav2-demo-params.yaml"
    default_session_file = ""
    if default_session_file:
        default_session_file = Path(default_session_file).expanduser()
    default_namespace = ""

    # robot model
    xacro_file = share_dir / "descriptions" / "slamcore-kobuki.urdf.xacro"
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent="  ")  # type: ignore

    # launch config variables -----------------------------------------------------------------
    namespace = default_namespace

    # remappings ------------------------------------------------------------------------------
    # map fully qualified names to relative ones so the node's namespace can be prepended.
    # in case of the transforms (tf), currently, there doesn't seem to be a better alternative
    pub_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # launch arguments ------------------------------------------------------------------------
    cfg.register(
        "comms",
        default_value=True,
        description=(
            "Establish communications with the Kobuki platform so that it starts "
            "publishing odometry measurements and accepting velocity commands"
        ),
    ),
    cfg.register("use_rviz", default_value=False, description="Start RVIZ")
    cfg.register(
        "config_file",
        default_value=default_slam_config_file,
        description="Full path to the SLAMcore SLAM configuration file",
    )
    cfg.register(
        "params_file",
        default_value=default_demo_params_file,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )
    cfg.register(
        "session_file",
        default_value=default_session_file,
        description=(
            "Path to SLAMcore session file to load. "
            "If unset, the software will not operate in Multisession mode"
        ),
    )
    cfg.register(
        "bt_xml_filename",
        default_value=get_package_share_path("nav2_bt_navigator")
        / "behavior_trees"
        / "navigate_w_replanning_and_recovery.xml",
        description="Full path to the behavior tree xml file to use",
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
        IncludeLaunchDescription(
            launchfile_for("slamcore_ros2_examples", "navigation_monitoring_launch.py"),
            condition=IfCondition(cfg.use_rviz.cfg),
        ),
        # kobuki ------------------------------------------------------------------------------
        IncludeLaunchDescription(
            launchfile_for("slamcore_ros2_examples", "kobuki_setup_comms_launch.py"),
            condition=IfCondition(cfg.comms.cfg),
        ),
        # robot stater publisher (static transforms for robot model) --------------------------
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=namespace,
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc}],
            remappings=pub_remappings,
            on_exit=Shutdown(),
        ),
        # SLAMcore SLAM -----------------------------------------------------------------------
        IncludeLaunchDescription(
            launchfile_for("slamcore_slam", "slam_publisher.launch.py"),
            launch_arguments={
                "config_file": cfg.config_file.cfg,
                "generate_map2d": PythonExpression(
                    ['"false" if "', cfg.session_file.cfg, '" else "true"']
                ),
                "odom_reading_topic": "/odom",
                "override_realsense_depth": "true",
                "realsense_depth_override_value": "true",
                "session_file": cfg.session_file.cfg,
                # frames of ref
                "base_frame": "slamcore/base_link",
                "odom_frame": "odom",
                "world_frame": "map",
            }.items(),
        ),
        # SLAMcore camera <-> kobuki base transformation --------------------------------------
        IncludeLaunchDescription(
            launchfile_for("slamcore_ros2_examples", "static_transform_pub_launch.py"),
        ),
        # nav2 navigation ---------------------------------------------------------------------
        IncludeLaunchDescription(
            launchfile_for("nav2_bringup", "navigation_launch.py"),
            launch_arguments={
                "namespace": namespace,
                "use_sim_time": use_sim_time,
                "autostart": "true",
                "params_file": cfg.params_file.cfg,
                "bt_xml_filename": cfg.bt_xml_filename.cfg,
                "use_lifecycle_mgr": "false",
            }.items(),
        ),
        ld=ld,
    )

    return ld
