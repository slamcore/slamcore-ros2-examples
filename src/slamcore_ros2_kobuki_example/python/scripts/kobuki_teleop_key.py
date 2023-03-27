#!/usr/bin/env python3
import subprocess
import sys

from ament_index_python.packages import get_package_share_path
from slamcore_ros2_examples_common.utils import get_yaml_ros_params


def main():
    """Poor man's teleoperation-node launcher.

    This process offers a shortcut for launching the kobuki_keyop_node and setting its
    arguments from a params file of our own -> "nav2-demo-params.yaml".

    This should have been a launchfile but currently I cannot find a way of forwarding
    the stdin of the launch system to the stdin of a child node.

    https://answers.ros.org/question/392516/ros2-launch-forward-keystrokes-to-child-node/
    """
    share_dir = get_package_share_path("slamcore_ros2_kobuki_example")

    keyop_params_file = get_package_share_path("kobuki_keyop") / "config" / "keyop_params.yaml"
    override_params_file = share_dir / "config" / "nav2-demo-params.yaml"
    params = get_yaml_ros_params(params_file=keyop_params_file, section="kobuki_keyop_node")
    params.update(
        get_yaml_ros_params(params_file=override_params_file, section="kobuki_keyop_node")
    )

    params_formatted = [f"{key}:={val}" for key, val in params.items()]
    cmd_args = [
        "ros2",
        "run",
        "kobuki_keyop",
        "kobuki_keyop_node",
        "--ros-args",
        "--remap",
        "/cmd_vel:=/input/keyop",
    ]
    for p in params_formatted:
        cmd_args.extend(["-p", p])

    proc = subprocess.run(cmd_args, check=True)
    return proc.returncode


if __name__ == "__main__":
    sys.exit(main())
