"""
Establish communications with the Kobuki platform so that it starts publishing odometry
measurements and accepting velocity commands.
"""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import Shutdown
from launch.conditions import IfCondition
from launch_ros.actions import Node
from slamcore_ros2_examples_common.configuration import Configuration
from slamcore_ros2_examples_common.utils import add_all_actions


def generate_launch_description():
    cfg = Configuration()

    # get directories & paths -----------------------------------------------------------------
    share_dir = get_package_share_path("slamcore_ros2_kobuki_example")

    # non-editable config variables -----------------------------------------------------------
    default_demo_params_file = share_dir / "config" / "nav2-demo-params.yaml"

    # launch arguments ------------------------------------------------------------------------
    cfg.register(
        "params_file",
        default_value=default_demo_params_file,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )
    cfg.register(
        "use_safety_controller",
        default_value=True,
        description=(
            "Enable/disable the safety_controller of the kobuki platform - uses the platform"
            " bumpers"
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
        # kobuki ------------------------------------------------------------------------------
        Node(
            package="kobuki_node",
            executable="kobuki_ros_node",
            name="kobuki_ros_node",
            output="screen",
            parameters=[
                str(
                    get_package_share_path("kobuki_node")
                    / "config"
                    / "kobuki_node_params.yaml"
                ),
                cfg.params_file.cfg,
            ],
            remappings=[("/commands/velocity", "/cmd_vel")],
            on_exit=Shutdown(),
        ),
        Node(
            package="kobuki_safety_controller",
            executable="kobuki_safety_controller_node",
            name="kobuki_safety_controller_node",
            output="screen",
            parameters=[cfg.params_file.cfg],
            remappings=[("/cmd_vel", "/input/safety_controller")],
            on_exit=Shutdown(),
            condition=IfCondition(cfg.use_safety_controller.cfg),
        ),
        Node(
            package="cmd_vel_mux",
            executable="cmd_vel_mux_node",
            name="cmd_vel_mux_node",
            output="screen",
            parameters=[cfg.params_file.cfg],
            on_exit=Shutdown(),
        ),
        ld=ld,
    )

    return ld
