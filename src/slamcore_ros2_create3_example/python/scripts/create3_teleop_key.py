#!/usr/bin/env python3
import subprocess
import sys


def main():
    """Poor man's teleoperation-node launcher.

    See kobuki_teleop_key.py for original implementation.
    """

    cmd_args = [
        "ros2",
        "run",
        "teleop_twist_keyboard",
        "teleop_twist_keyboard",
        "--ros-args",
        "--remap",
        "/cmd_vel:=/input/keyop",
    ]

    proc = subprocess.run(cmd_args, check=True)
    return proc.returncode


if __name__ == "__main__":
    sys.exit(main())
