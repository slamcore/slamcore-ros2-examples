#!/usr/bin/env python3
import os
import sys
from argparse import ArgumentParser
from pathlib import Path
from typing import Optional

name = Path(__file__).name


def get_var(var_name: str) -> Optional[str]:
    var = os.environ.get(var_name)
    return var


valid_robot_to_ros_version_combinations = {
    "kobuki": ["foxy"],
    "create3": ["galactic"],
    "turtlebot4": ["galactic"],
}


class ProgrammaticError(RuntimeError):
    def __init__(self, msg):
        msg = f"{msg}. This shoulnd't happen - please report to maintainers"
        super().__init__(msg)


def check_combination_validity():
    ros_ver = get_var("ROS_VER")
    robot = get_var("ROBOT")

    if not robot:
        raise ProgrammaticError("No robot has been specified via the ROBOT flag")

    if not ros_ver:
        raise RuntimeError(
            "No ROS version has been specified. You have to specify one using the ROS_VER env"
            f" variable\n{report_combinations()}"
        )

    if robot not in valid_robot_to_ros_version_combinations:
        raise ProgrammaticError(f"Unknown robot type -> {robot}")

    if ros_ver not in valid_robot_to_ros_version_combinations[robot]:
        msg_lines = (
            f"The given robot:ros_version is not supported -> {robot}:{ros_ver}",
            "\n",
            report_combinations(),
        )

        raise RuntimeError("\n".join(msg_lines))


def report_combinations() -> str:
    lines = (
        "\nAvailable robot:ros_version combinations",
        "=" * 40,
        *[f'{k}: {", ".join(v)}' for k, v in valid_robot_to_ros_version_combinations.items()],
    )

    return "\n".join(lines)


def main():
    parser = ArgumentParser(
        description="Combination validity check and report",
        epilog="Not meant to run manually - Executed automatically via Taskfile",
    )

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--report-combinations", action="store_true")
    group.add_argument("--check-combinations", action="store_true")

    args = vars(parser.parse_args())

    if args["report_combinations"]:
        print(report_combinations())
    elif args["check_combinations"]:
        check_combination_validity()
    else:
        ProgrammaticError("Unknown flag")


main()
