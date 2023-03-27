#!/usr/bin/env python3
"""
Wrapper script around the tf2_ros/static_transform_publisher executable reading the pose and
reference frames from a YAML and forwarding them as CLI parameters.
"""

import subprocess
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from slamcore_ros2_examples_common.utils import get_yaml_ros_params


def main(args=None):
    rclpy.init(args=args)
    node = Node(Path(__file__).name)  # type: ignore
    logger = node.get_logger()

    node.declare_parameter("params_file", "")
    node.declare_parameter("params_file_section", "")

    params_file = node.get_parameter("params_file").get_parameter_value().string_value
    params_file_section = (
        node.get_parameter("params_file_section").get_parameter_value().string_value
    )

    logger.info(f"Parsing the static transform from YAML: {params_file}:{params_file_section}")

    transform = get_yaml_ros_params(params_file=Path(params_file), section=params_file_section)
    parent_frame = transform["parent_frame"]
    child_frame = transform["child_frame"]
    xyz = transform["xyz"]
    rpy = transform["rpy"]

    cmd_args = [
        str(x)
        for x in (
            "ros2",
            "run",
            "tf2_ros",
            "static_transform_publisher",
            *xyz,
            *rpy,
            parent_frame,
            child_frame,
        )
    ]

    proc = subprocess.run(cmd_args, check=True)

    rclpy.shutdown()
    return proc.returncode


if __name__ == "__main__":
    sys.exit(main())
