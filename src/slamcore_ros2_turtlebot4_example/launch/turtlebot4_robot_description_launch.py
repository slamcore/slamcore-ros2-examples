from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node
from slamcore_ros2_examples_common.configuration import Configuration
from slamcore_ros2_examples_common.utils import add_all_actions


def generate_launch_description():
    cfg = Configuration()

    # get package directory -----------------------------------------------------------------
    share_dir = get_package_share_directory("slamcore_ros2_turtlebot4_example")

    # robot model
    xacro_file = PathJoinSubstitution(
        [
            share_dir,
            "descriptions",
            LaunchConfiguration("model"),
            "slamcore-turtlebot4.urdf.xacro",
        ]
    )

    # remappings ------------------------------------------------------------------------------
    # map fully qualified names to relative ones so the node's namespace can be prepended.
    # in case of the transforms (tf), currently, there doesn't seem to be a better alternative
    pub_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # launch arguments ------------------------------------------------------------------------
    cfg.register(
        "model",
        default_value="standard",
        choices=["standard", "lite"],
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
        # robot state publisher (static transforms for robot model) --------------------------
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "use_sim_time": cfg.use_sim_time.cfg,
                    "robot_description": Command(["xacro ", xacro_file]),
                }
            ],
            remappings=pub_remappings,
        ),
        ld=ld,
    )

    return ld
