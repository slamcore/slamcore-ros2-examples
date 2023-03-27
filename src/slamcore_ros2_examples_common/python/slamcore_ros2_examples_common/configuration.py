from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Text, Union

from launch.actions import DeclareLaunchArgument
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration


@dataclass
class ConfigurationEntry:
    cfg: LaunchConfiguration
    declare_arg: DeclareLaunchArgument


class Configuration:
    def __init__(self):
        self._map: Dict[str, ConfigurationEntry] = {}

    def register(
        self,
        name: Text,
        *,
        default_value: Optional[Union[SomeSubstitutionsType, Path, bool]] = None,
        description: Optional[Text] = None,
        choices: [List[Text]] = None,  # type: ignore
        **kwargs,
    ):
        if isinstance(default_value, Path) or isinstance(default_value, bool):
            default_value = str(default_value)

        self._map[name] = ConfigurationEntry(
            LaunchConfiguration(name),
            DeclareLaunchArgument(
                name,
                default_value=default_value,
                description=description,
                choices=choices,
                **kwargs,
            ),
        )

    def launch_arguments(self) -> List[DeclareLaunchArgument]:
        return [entry.declare_arg for entry in self._map.values()]

    def __getattr__(self, attr) -> ConfigurationEntry:
        return self._map[attr]
