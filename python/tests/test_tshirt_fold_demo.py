"""The T-shirt fold example, asserted the way its owner would.

`examples/cloth/tshirt_fold_demo.py` is the shipped demonstration of cloth
in a cell: two arms turn a shirt's sleeves in and fold its hem onto its
shoulders, the shirt a `bt.cloth` shell simulated against the baked cycle.
This pins what makes that cell *right*: the cloth rides the bake and
finishes it, each hand holds cloth exactly while its signal is on, and the
shirt ends up folded — a quarter of its footprint, the hem at the
shoulders, the cuffs over the body.

The arms and their grippers come from the catalog, so the cell is baked
only where the catalog is cached. It is the demo's own bake: about a minute
of cloth simulation, once for the module.
"""

import os
import sys
from pathlib import Path

import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "cloth"))

HF_CACHE = Path(os.environ.get("HF_HOME") or Path.home() / ".cache" / "huggingface") / "hub"
CATALOG_CACHED = any(HF_CACHE.glob("datasets--botrail--botrail-catalog*"))

pytestmark = pytest.mark.skipif(not CATALOG_CACHED, reason="the arms come from the catalog")


@pytest.fixture(scope="module")
def fold():
    pytest.importorskip("huggingface_hub", reason="the arms come from the catalog")
    import tshirt_fold_demo as demo

    scene, cloth, name = demo.build()
    return demo, scene, scene.simulate_sequence(name, max_duration=120.0)


def test_the_shirt_rides_the_bake_to_its_end(fold) -> None:
    demo, scene, timeline = fold
    assert timeline.robots == list(demo.ARMS)
    assert timeline.cloths == ["shirt"]
    track = timeline.cloth("shirt")
    assert track.failure is None
    assert track.times[0] == 0.0 and track.times[-1] == timeline.duration
    # Nothing the pass noticed is a hand closing on nothing.
    assert not any("closed on nothing" in warning for warning in track.warnings)


def test_each_hand_holds_cloth_exactly_while_its_signal_is_on(fold) -> None:
    demo, scene, timeline = fold
    track = timeline.cloth("shirt")
    signals = dict(timeline.signals)
    for arm in demo.ARMS:
        edges = signals[f"grip_{arm}"]
        # Twice closed: the sleeve, then the hem.
        assert [on for _, on in edges] == [False, True, False, True, False]
    # The first hand holds alone during its sleeve; both hold the hem.
    north = signals["grip_north"]
    sleeve = (north[1][0] + north[2][0]) / 2
    hem = (north[3][0] + north[4][0]) / 2
    alone, together = len(track.held(sleeve)), len(track.held(hem))
    # A pinch goes through both layers: a dozen vertices a hand at least.
    assert alone >= 12 and together >= 2 * 12
    assert track.held(0.0) == [] and track.held(timeline.duration) == []
    # Between its two grasps a hand holds nothing.
    assert track.held((north[2][0] + signals["grip_south"][1][0]) / 2) == []


def test_the_shirt_ends_up_folded(fold) -> None:
    demo, scene, timeline = fold
    numbers = demo.measure(scene, timeline)
    flat, folded = numbers["flat"], numbers["folded"]
    # Half as long and as wide as the body alone: a third of the footprint.
    assert folded[0] < 0.6 * flat[0] and folded[1] < 0.6 * flat[1]
    assert folded[0] * folded[1] < 0.35 * flat[0] * flat[1]
    # A low pile: a few layers of cloth and the folds between them.
    assert numbers["height"] < 0.06
    # The hem lies at the shoulders and each cuff well inside the body.
    assert abs(numbers["hem_short"]) < 0.06, numbers
    assert numbers["cuffs_in"] > 0.08, numbers
