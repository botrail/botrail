"""botrail: ROS-free robot motion authoring with a web-based 3D studio."""

from . import seq
from ._core import (
    Clearance,
    IkResult,
    Robot,
    Scene,
    SequenceTimeline,
    SignalTrack,
    Span,
    StudioServer,
    Trajectory,
    __version__,
)
from ._launcher import studio

__all__ = [
    "Clearance",
    "IkResult",
    "Robot",
    "Scene",
    "SequenceTimeline",
    "SignalTrack",
    "Span",
    "StudioServer",
    "Trajectory",
    "seq",
    "studio",
    "__version__",
]
