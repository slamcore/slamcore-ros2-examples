"""
Launch live navigation using:

* The kobuki robot platform
* Slamcore Visual-Inertial or Visual-Inertial-Kinematic SLAM
* Nav2 framework

"""

import xacro
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetRemap
from slamcore_ros2_examples_common.configuration import Configuration
from slamcore_ros2_examples_common.shortcuts import static_transform_pub_launch
from slamcore_ros2_examples_common.utils import add_all_actions, launchfile_for


def generate_launch_description():
    cfg = Configuration()

    # get directories & paths -----------------------------------------------------------------
    share_dir = get_package_share_path("slamcore_ros2_kobuki_example")

    # non-editable config variables -----------------------------------------------------------
    default_slam_config_file = share_dir / "config" / "slam-config.json"
    default_demo_params_file = share_dir / "config" / "nav2-demo-params.yaml"
    default_session_file = '""'
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
    )
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
        "session_file",
        default_value=default_session_file,
        description=(
            "Path to Slamcore session file to load. "
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
        # kobuki ------------------------------------------------------------------------------
        IncludeLaunchDescription(
            launchfile_for("slamcore_ros2_kobuki_example", "kobuki_setup_comms_launch.py"),
            condition=IfCondition(cfg.comms.cfg),
        ),
        # kobuki bumper2pointcloud ------------------------------------------------------------
        Node(
            package="kobuki_bumper2pc",
            executable="kobuki_bumper2pc_node",
            name="kobuki_bumper2pc_node",
            namespace=namespace,
            output="screen",
            parameters=[cfg.params_file.cfg],
            remappings=[
                ("/core_sensors", "/sensors/core"),
                ("/pointcloud", "/bumper_pointcloud"),
            ],
            on_exit=Shutdown(),
        ),
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
                "override_realsense_depth": "true",
                "realsense_depth_override_value": "true",
                "session_file": cfg.session_file.cfg,
                # frames of ref
                "base_frame": "slamcore/base_link",
                "odom_frame": "odom",
                "map_frame": "map",
            }.items(),
        ),
        # Slamcore camera <-> robot base transformation ---------------------------------------
        static_transform_pub_launch(cfg.params_file.cfg),
        # nav2 navigation ---------------------------------------------------------------------
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
                        "bt_xml_filename": cfg.bt_xml_filename.cfg,
                        "use_lifecycle_mgr": "false",
                    }.items(),
                ),
            ]
        ),
        ld=ld,
    )

    return ld
