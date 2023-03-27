"""Module for the XacroFile substitution class."""

from pathlib import Path
from typing import Text, cast

import xacro
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch.substitutions import SubstitutionFailure
from launch.utilities import normalize_to_list_of_substitutions


class XacroFileContents(Substitution):
    """
    Reads the xacro file provided and returns its context during evalution.
    """

    name = "XacroFileContents"

    def __init__(self, substitution: SomeSubstitutionsType) -> None:
        """Create a class instance."""
        self.__substitution = normalize_to_list_of_substitutions((substitution,))[0]  # type: ignore

    @property
    def substitution(self) -> Substitution:
        """Getter."""
        return self.__substitution

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return f"{self.name}({self.substitution.describe()})"

    @classmethod
    def read_xacro(cls, path: Path) -> str:
        """Read the xacro contents and return the corresponding string."""
        doc = xacro.process_file(path)
        xacro_contents = doc.toprettyxml(indent="  ")  # type: ignore
        return cast(str, xacro_contents)

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution - return the contents of the given xacro file."""
        path = Path(self.substitution.perform(context))
        if not path.is_file():
            raise SubstitutionFailure(f"Not a file: {path.absolute()}")

        xacro_contents = self.read_xacro(path)
        # I have to escape double quotes, then double quote the whole string so that the YAML
        # parser is happy
        xacro_contents = xacro_contents.replace('"', '\\"')
        xacro_contents = f'"{xacro_contents}"'
        return xacro_contents
