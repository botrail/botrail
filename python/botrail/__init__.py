"""botrail: ROS-free robot motion authoring with a web-based 3D studio."""

from ._core import IkResult, Robot, Scene, StudioServer, __version__
from ._launcher import studio

__all__ = ["IkResult", "Robot", "Scene", "StudioServer", "studio", "__version__"]
