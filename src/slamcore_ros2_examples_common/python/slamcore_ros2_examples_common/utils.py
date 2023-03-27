import logging
import operator
from functools import reduce
from pathlib import Path
from typing import Any, Dict, Optional, cast

import yaml
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.action import Action
from launch.launch_description_sources import PythonLaunchDescriptionSource


def add_all_actions(*actions: Action, ld: LaunchDescription):
    """Helper method to add a batch of given actions to the LaunchDescription."""
    for action in actions:
        ld.add_action(action)


def existing_path(p: Path) -> Path:
    if not p.is_file():
        raise RuntimeError(f"Non-existing path -> {p}")
    return p


def launchfile_for(pkg_name: str, launchfile_name: str) -> PythonLaunchDescriptionSource:
    path = existing_path(get_package_share_path(pkg_name) / "launch" / launchfile_name)
    return PythonLaunchDescriptionSource(str(path))


def _get_from_dict(data: dict, keys_seq: list):
    return reduce(operator.getitem, keys_seq, data)


def get_yaml_ros_params(params_file: Path, section: str) -> Dict[str, Any]:
    return get_yaml_params(params_file=params_file, section=f"{section}.ros__parameters")


def get_yaml_params(params_file: Path, section: str) -> Dict[str, Any]:
    """Fetch the ROS parameters from the parameters file and section at hand."""
    if not params_file.is_file():
        raise FileNotFoundError(params_file)

    with params_file.open("rb") as f:
        params = yaml.load(f, Loader=yaml.Loader)

    sections = section.split(".")
    try:
        conts = _get_from_dict(data=params, keys_seq=sections)
    except KeyError:
        raise RuntimeError(
            f'Required section sequence [{".".join(sections)}] not present in parameters file'
            f" -> {params_file}"
        )

    return cast(Dict[str, Any], conts)


def get_params_w_overrides(
    *,
    params: Path,
    section: str,
    override_params: Path,
    override_section: Optional[str] = None,
) -> dict:
    if override_section is None:
        override_section = section

    conts = get_yaml_params(params, section=section)
    conts.update(get_yaml_params(override_params, section=override_section))
    return conts


def get_ros_params_w_overrides(
    *,
    params: Path,
    section: str,
    override_params: Path,
    override_section: Optional[str] = None,
) -> dict:
    return get_params_w_overrides(
        params=params,
        section=f"{section}.ros__parameters",
        override_params=override_params,
        override_section=f"{override_section}.ros__parameters",
    )


# setup a global logger for this package ------------------------------------------------------
logger = logging.getLogger(Path(".").absolute().name)
logger.setLevel("DEBUG")
