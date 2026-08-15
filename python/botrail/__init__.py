"""botrail: ROS-free robot motion authoring with a web-based 3D studio."""

from . import paint, seq, toolpath
from ._core import (
    Clearance,
    FeedReport,
    FilmCoat,
    IkResult,
    PaintReport,
    Robot,
    Scene,
    SequenceTimeline,
    SignalTrack,
    Span,
    StockCarve,
    StudioServer,
    ToolpathReport,
    Trajectory,
    __version__,
    catalog_package,
)
from ._launcher import studio

__all__ = [
    "Clearance",
    "FeedReport",
    "FilmCoat",
    "IkResult",
    "PaintReport",
    "Robot",
    "Scene",
    "SequenceTimeline",
    "SignalTrack",
    "Span",
    "StockCarve",
    "StudioServer",
    "ToolpathReport",
    "Trajectory",
    "catalog_package",
    "paint",
    "seq",
    "studio",
    "toolpath",
    "__version__",
]
