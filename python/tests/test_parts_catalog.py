"""`bt.parts` against a catalog spec pack — equipment bought to size.

A fence is standard parts arranged to a layout: the panels, posts and door
are catalogue items, and what is custom is the run they are built into. So
the catalog ships the configuration (the widths and heights that exist, the
part numbers they compose into, the mass that comes with them) and the
generator lays it out. These tests pin that the fence is built from widths
that exist, that each width lands on the BOM as its own orderable line, and
that a dimension nobody sells is refused with the list of the ones that are.
"""

import json
import re
import sys
import types
from pathlib import Path

import pytest

import botrail as bt

FENCE_ID = "acme/fence/guard/r1"
BELT_ID = "acme/conveyor/belt/r1"
RACK_ID = "acme/rack/shelf/r1"
SHA = "0123abcd0123abcd0123abcd0123abcd0123abcd"

# A spec pack: no geometry, only what you can order and what it weighs.
MANIFEST = """
schema_version: '0.1'
id: acme/fence/guard/r1
kind: spec
category: structure.fence
name: Guard Fence
manufacturer:
  name: ACME Guarding
distribution: public
mechanical:
  footprint_mm: [1000, 30]
  height_mm: 2000
  mass_kg: 11.0
  mount: floor
configuration:
  generator: fence
  params:
    height_mm:
      values: [1600, 2000, 2200]
      default: 2000
    mesh_mm:
      values: ["49x49", "20x20"]
      default: "49x49"
  components:
    - role: panel
      category: structure.fence
      part_number: GP-{height_mm}x{width_mm}
      widths_mm: [200, 400, 600, 800, 1000, 1200]
      dimensions_mm: {thickness: 30}
      mass:
        base_kg: 1.0
        per_m2_kg: 5.0
        area: [height_mm, width_mm]
    - role: post
      category: structure.fence.post
      part_number: GPP-{height_mm}
      dimensions_mm: {section_w: 60, section_d: 40}
      mass:
        base_kg: 1.0
        per_mm: {height_mm: 0.003}
    - role: door
      category: structure.door
      part_number: GPD-{height_mm}x{width_mm}
      widths_mm: [1000]
      mass:
        table:
          - {height_mm: 1600, width_mm: 1000, kg: 15.0}
          - {height_mm: 2000, width_mm: 1000, kg: 18.0}
          - {height_mm: 2200, width_mm: 1000, kg: 19.5}
"""

# Some makers do not write a dimension in the article number as it stands:
# a 2200 mm panel is coded 220, and the post that carries it — a floor gap
# taller — is coded 230. The pack brings the table, per part.
CODED_MANIFEST = """
schema_version: '0.1'
id: acme/fence/coded/r1
kind: spec
category: structure.fence
name: Coded Guard
manufacturer:
  name: ACME Guarding
distribution: public
configuration:
  generator: fence
  params:
    height_mm:
      values: [1900, 2200]
      default: 2200
    post_finish:
      values: [graphite-black, zinc-yellow]
      default: graphite-black
  components:
    - role: panel
      category: structure.fence
      part_number: W322-{height_mm_code}{width_mm_code}
      widths_mm: [250, 400, 800, 1000, 1200, 1500]
      dimensions_mm: {thickness: 30}
      codes:
        height_mm: {1900: "190", 2200: "220"}
        width_mm: {250: "025", 400: "040", 800: "080", 1000: "100", 1200: "120", 1500: "150"}
      mass:
        per_m2_kg: 2.4
        area: [height_mm, width_mm]
        per_mm: {height_mm: 0.0022, width_mm: 0.0022}
    - role: post
      category: structure.fence.post
      part_number: "{post_finish_code}-{height_mm_code}"
      dimensions_mm: {section_w: 50, section_d: 50}
      # Written the way a published manifest carries it: YAML through JSON
      # turns the keys into strings.
      codes:
        height_mm: {"1900": "200", "2200": "230"}
        post_finish: {graphite-black: P31, zinc-yellow: P11}
      mass:
        base_kg: 1.2
        per_mm: {height_mm: 0.0028}
"""

# A conveyor is the other shape of equipment: one model, ordered to a size,
# with a speed the drive has to be able to reach.
BELT_MANIFEST = """
schema_version: '0.1'
id: acme/conveyor/belt/r1
kind: spec
category: conveyor.belt
name: Belt Unit
manufacturer:
  name: ACME Handling
distribution: public
configuration:
  generator: conveyor
  params:
    length_mm: {min: 600, max: 4000, step: 100, default: 2000}
    width_mm: {values: [200, 300, 400], default: 300}
    height_mm: {min: 500, max: 900, step: 50, default: 750}
  components:
    - role: unit
      category: conveyor.belt
      part_number: BU-{width_mm}x{length_mm}
      dimensions_mm: {belt_thickness: 50, rail: 30}
      mass:
        base_kg: 8.0
        per_mm: {length_mm: 0.0075, width_mm: 0.02}
    - role: stand
      category: structure.pedestal
      part_number: BS-{height_mm}
      dimensions_mm: {leg: 50}
      mass:
        base_kg: 2.0
        per_mm: {height_mm: 0.004}
  behavior:
    speed_mps: {min: 0.02, max: 0.5, default: 0.2}
    payload_kg_per_m: 25
  rules:
    stand_span_max_mm: 1500
"""

# Shelving is the third shape: sizes you pick from a list, plus a *count*
# (the levels) that decides how many shelf boards come with it.
RACK_MANIFEST = """
schema_version: '0.1'
id: acme/rack/shelf/r1
kind: spec
category: structure.rack
name: Shelf Rack
manufacturer:
  name: ACME Storage
distribution: public
specs:
  capacity_kg_per_level: 300
configuration:
  generator: rack
  params:
    width_mm: {values: [900, 1200, 1800], default: 1200}
    depth_mm: {values: [450, 600], default: 600}
    height_mm: {values: [1200, 1800, 2400], default: 1800}
    levels: {min: 2, max: 6, step: 1, default: 4}
  components:
    - role: bay
      category: structure.rack
      part_number: SR-{width_mm}x{depth_mm}x{height_mm}
      dimensions_mm: {upright: 40}
      mass:
        base_kg: 4.0
        per_mm: {height_mm: 0.006, width_mm: 0.004}
    - role: shelf
      category: structure.rack.shelf
      part_number: SR-SH-{width_mm}x{depth_mm}
      dimensions_mm: {thickness: 30}
      mass:
        base_kg: 1.0
        per_m2_kg: 8.0
        area: [width_mm, depth_mm]
  rules:
    level_pitch_min_mm: 250
"""

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
# A 2.4 x 1.6 m cell: both edge lengths need more than one panel width.
# Wire shelving is not sold as a bay: you buy posts (in pairs) and shelves,
# and the same finish is called something different on each of them.
POST_RACK_MANIFEST = """
schema_version: '0.1'
id: acme/shelf/wire/r1
kind: spec
category: structure.rack
name: Wire Shelving
manufacturer:
  name: ACME Storage
distribution: public
configuration:
  generator: rack
  params:
    width_mm:
      values: [900, 1200]
      default: 1200
    depth_mm:
      values: [450, 600]
      default: 450
    height_mm:
      values: [1400, 1900]
      default: 1900
    levels:
      min: 2
      max: 5
      step: 1
      default: 4
    finish:
      values: [chrome, white]
      default: chrome
  components:
    - role: upright
      category: structure.rack
      part_number: B{height_mm_code}P{finish_code}2
      dimensions_mm: {section: 25.4}
      codes:
        height_mm: {1400: "54", 1900: "74"}
        finish: {chrome: S, white: W}
      mass:
        table:
          - {height_mm: 1400, kg: 2.1}
          - {height_mm: 1900, kg: 2.8}
    - role: shelf
      category: structure.rack.shelf
      part_number: B{depth_mm_code}{width_mm_code}{finish_code}1
      dimensions_mm: {thickness: 30}
      codes:
        depth_mm: {450: "18", 600: "24"}
        width_mm: {900: "36", 1200: "48"}
        finish: {chrome: C, white: W}
      mass:
        table:
          - {depth_mm: 450, width_mm: 900, kg: 3.2}
          - {depth_mm: 450, width_mm: 1200, kg: 4.0}
          - {depth_mm: 600, width_mm: 900, kg: 3.9}
          - {depth_mm: 600, width_mm: 1200, kg: 4.8}
  rules:
    uprights_per_pack: 2
    level_pitch_min_mm: 25.4
"""

# Some stands are cut to order and sold in bands: any height from 700 to 799
# is one article, so the code table is read as the low end of a band.
BAND_MANIFEST = """
schema_version: '0.1'
id: acme/stand/cut-pillar/r1
kind: spec
category: structure.pedestal
name: Cut Pillar
manufacturer:
  name: ACME Framing
distribution: public
configuration:
  generator: pedestal
  params:
    height_mm:
      min: 200
      max: 1300
      step: 1
      default: 750
  components:
    - role: pedestal
      category: structure.pedestal
      part_number: ZFR-F03{height_mm_code}
      dimensions_mm: {column: 280, plate: 30, base_w: 485, base_d: 485, top_w: 280, top_d: 280}
      codes:
        height_mm:
          {200: "2", 300: "3", 400: "4", 500: "5", 600: "6", 700: "7",
           800: "8", 900: "9", 1000: A, 1100: B, 1200: C}
      mass:
        base_kg: 30.4
        per_mm: {height_mm: 0.048}
"""

# A stand is the other half of the framing catalogue: the aluminium frame is
# one article and the board on it is another, so the bench takes two lines.
STAND_MANIFEST = """
schema_version: '0.1'
id: acme/stand/frame-bench/r1
kind: spec
category: structure.table
name: Frame Bench
manufacturer:
  name: ACME Framing
distribution: public
configuration:
  generator: table
  params:
    width_mm:
      values: [900, 1200, 1500]
      default: 1200
    depth_mm:
      values: [600, 750]
      default: 600
    height_mm:
      values: [700, 750, 800, 900]
      default: 750
  components:
    - role: frame
      category: structure.table
      part_number: FB-{width_mm}x{depth_mm}x{height_mm}
      dimensions_mm: {leg: 40}
      mass:
        base_kg: 6.0
        per_mm: {width_mm: 0.008, depth_mm: 0.006, height_mm: 0.009}
    - role: top
      category: structure.table
      part_number: FB-TOP-{width_mm}x{depth_mm}
      dimensions_mm: {thickness: 25}
      mass:
        base_kg: 1.0
        per_m2_kg: 14.0
        area: [width_mm, depth_mm]
"""

# A pedestal is one article — what a robot is bolted to.
PILLAR_MANIFEST = """
schema_version: '0.1'
id: acme/stand/robot-pillar/r1
kind: spec
category: structure.pedestal
name: Robot Pillar
manufacturer:
  name: ACME Framing
distribution: public
configuration:
  generator: pedestal
  params:
    height_mm:
      values: [400, 500, 600, 700]
      default: 500
    finish:
      values: [painted, plated]
      default: painted
  components:
    - role: pedestal
      category: structure.pedestal
      part_number: RP-{height_mm}{finish_code}
      dimensions_mm:
        column: 180
        plate: 20
        base_w: 450
        base_d: 450
        top_w: 300
        top_d: 300
      codes:
        finish: {painted: K, plated: P}
      mass:
        base_kg: 12.0
        per_mm: {height_mm: 0.05}
"""

# A control cabinet: the enclosure is the article, the plinth base and the
# mounting plate are articles of their own, and a size nobody sells has no
# row in the mass table.
CABINET_MANIFEST = """
schema_version: '0.1'
id: acme/panel/enclosure/r1
kind: spec
category: structure.cabinet
name: Panel Enclosure
manufacturer:
  name: ACME Panels
distribution: public
mechanical:
  footprint_mm: [800, 600]
  height_mm: 2200
  mass_kg: 150.0
  mount: floor
specs:
  ip_rating: IP55
configuration:
  generator: cabinet
  params:
    width_mm:
      values: [600, 800, 1000]
      default: 800
    height_mm:
      values: [1600, 2100]
      default: 2100
    depth_mm:
      values: [400, 600]
      default: 600
    base_height_mm:
      values: [50, 100]
      default: 100
  components:
    - role: body
      category: structure.cabinet
      part_number: PE{depth_mm_code}-{width_mm_code}{height_mm_code}
      codes:
        depth_mm: {400: "40", 600: "60"}
        width_mm: {600: "6", 800: "8", 1000: "10"}
        height_mm: {1600: "16", 2100: "21"}
      mass:
        table:
          - {width_mm: 600, height_mm: 1600, depth_mm: 400, kg: 84.0}
          - {width_mm: 800, height_mm: 2100, depth_mm: 600, kg: 150.0}
          - {width_mm: 1000, height_mm: 2100, depth_mm: 600, kg: 176.0}
    - role: base
      category: structure.cabinet.base
      part_number: PB-{width_mm_code}{depth_mm_code}{base_height_mm_code}
      codes:
        width_mm: {600: "6", 800: "8", 1000: "10"}
        depth_mm: {400: "40", 600: "60"}
        base_height_mm: {50: "05", 100: "10"}
      mass:
        base_kg: 2.0
        per_mm: {width_mm: 0.008}
    - role: plate
      category: structure.cabinet.plate
      part_number: PP-{width_mm_code}{height_mm_code}
      dimensions_mm: {thickness: 2.3}
      codes:
        width_mm: {600: "6", 800: "8", 1000: "10"}
        height_mm: {1600: "16", 2100: "21"}
      mass:
        base_kg: 0.0
        per_m2_kg: 18.0
        area: [width_mm, height_mm]
"""

# An article number that cannot be composed from the axes — the pack lists
# the sold combinations instead, each with the number it is sold under.
AX_MANIFEST = """
schema_version: '0.1'
id: acme/ax/steel-box/r1
kind: spec
category: structure.cabinet
name: AX Steel Box
manufacturer:
  name: ACME Boxes
distribution: public
configuration:
  generator: cabinet
  params:
    width_mm: {values: [600, 800], default: 800}
    height_mm: {values: [800, 1000], default: 1000}
    depth_mm: {values: [300, 400], default: 300}
  components:
    - role: body
      category: structure.cabinet
      variants:
        - {width_mm: 600, height_mm: 800, depth_mm: 400, part_number: AX 1059.000, kg: 35.5}
        - {width_mm: 800, height_mm: 1000, depth_mm: 300, part_number: AX 1180.000, kg: 49.7}
        - {width_mm: 800, height_mm: 1000, depth_mm: 400, part_number: AX 1181.000, kg: 56.0}
"""

RING = [(-1.2, -0.6), (1.2, -0.6), (1.2, 1.0), (-1.2, 1.0)]


@pytest.fixture()
def pack(tmp_path: Path) -> Path:
    """The package as it sits on disk — how the catalog builder validates one
    before it is published, and what `catalog=<path>` takes."""
    directory = tmp_path / "guard"
    directory.mkdir()
    (directory / "manifest.yaml").write_text(MANIFEST)
    return directory


@pytest.fixture()
def wire(tmp_path: Path) -> Path:
    directory = tmp_path / "wire"
    directory.mkdir()
    (directory / "manifest.yaml").write_text(POST_RACK_MANIFEST)
    return directory


@pytest.fixture()
def coded(tmp_path: Path) -> Path:
    directory = tmp_path / "coded"
    directory.mkdir()
    (directory / "manifest.yaml").write_text(CODED_MANIFEST)
    return directory


@pytest.fixture()
def belt(tmp_path: Path) -> Path:
    directory = tmp_path / "belt"
    directory.mkdir()
    (directory / "manifest.yaml").write_text(BELT_MANIFEST)
    return directory


@pytest.fixture()
def shelving(tmp_path: Path) -> Path:
    directory = tmp_path / "rack"
    directory.mkdir()
    (directory / "manifest.yaml").write_text(RACK_MANIFEST)
    return directory


@pytest.fixture()
def stand(tmp_path: Path) -> Path:
    directory = tmp_path / "bench"
    directory.mkdir()
    (directory / "manifest.yaml").write_text(STAND_MANIFEST)
    return directory


@pytest.fixture()
def pillar(tmp_path: Path) -> Path:
    directory = tmp_path / "pillar"
    directory.mkdir()
    (directory / "manifest.yaml").write_text(PILLAR_MANIFEST)
    return directory


@pytest.fixture()
def enclosure(tmp_path: Path) -> Path:
    directory = tmp_path / "enclosure"
    directory.mkdir()
    (directory / "manifest.yaml").write_text(CABINET_MANIFEST)
    return directory


@pytest.fixture()
def hub(pack: Path, monkeypatch: pytest.MonkeyPatch) -> dict:
    """`catalog=<id>` — the same package reached through the dataset."""
    # The real cache lays a snapshot out as <…>/snapshots/<sha>/<id>, and the
    # revision a package came from is read back off that path.
    repo = pack.parent / "dataset" / "snapshots" / SHA
    (repo / FENCE_ID).mkdir(parents=True)
    (repo / FENCE_ID / "manifest.yaml").write_text(MANIFEST)
    (repo / "index.json").write_text(
        json.dumps(
            {
                "schema_version": "0.1",
                "generated_at": "2026-08-21",
                "products": [
                    {
                        "id": FENCE_ID,
                        "kind": "spec",
                        "category": "structure.fence",
                        "name": "Guard Fence",
                        "manufacturer": "ACME Guarding",
                        "specs": {},
                        "validation_level": "V2",
                        "distribution": "public",
                        "assets": {"urdf": None, "usd": None},
                    }
                ],
            }
        )
    )
    calls: dict = {}
    fake = types.ModuleType("huggingface_hub")

    def dataset_info(repo_id, *, revision=None, timeout=None, files_metadata=False, token=None):
        calls["revision_requested"] = revision
        return types.SimpleNamespace(sha=SHA)

    def hf_hub_download(repo_id, filename=None, repo_type=None, revision=None):
        return str(repo / filename)

    def snapshot_download(repo_id, repo_type=None, revision=None, allow_patterns=None):
        calls["allow_patterns"] = allow_patterns
        return str(repo)

    fake.dataset_info = dataset_info
    fake.hf_hub_download = hf_hub_download
    fake.snapshot_download = snapshot_download
    monkeypatch.setitem(sys.modules, "huggingface_hub", fake)
    return calls


def scene_() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))


def rows(scene) -> dict:
    return {row["names"][0]: row for row in scene.bom().rows}


def test_the_fence_is_built_from_widths_that_exist(pack: Path) -> None:
    scene = scene_()
    built = bt.parts.fence(scene, "fence", path=RING, catalog=pack, height=2.0)
    # Every panel is one of the catalogue widths, and every edge is covered:
    # a 2.4 m edge takes 1000 + 1000 + 200 and a 1.6 m edge 1000 + 200 + 200
    # (the posts between them count towards the length).
    widths = sorted(
        round((hi[0] - lo[0]) * 1000) if abs(hi[0] - lo[0]) > abs(hi[1] - lo[1])
        else round((hi[1] - lo[1]) * 1000)
        for name in built.obstacles
        if "/panels/" in name
        for lo, hi in [scene.obstacle_bounds(name)]
    )
    assert set(widths) <= {200, 400, 600, 800, 1000, 1200}
    assert widths.count(1000) == 6 and widths.count(200) == 6
    # The run reaches the corners: the panels plus one post per bay fill the
    # edge to within the catalog's tolerance (25 mm).
    for edge, length in ((0, 2.4), (1, 1.6)):
        bays = [
            n for n in built.obstacles
            if "/panels/" in n and n.endswith(tuple(f"/e{edge}_{i}" for i in range(9)))
        ]
        panels = sum(
            max(hi[0] - lo[0], hi[1] - lo[1]) for lo, hi in map(scene.obstacle_bounds, bays)
        )
        assert length - (panels + 0.06 * len(bays)) == pytest.approx(0.0, abs=0.026)


def test_every_width_lands_on_the_bom_as_its_own_orderable_line(pack: Path) -> None:
    scene = scene_()
    bt.parts.fence(scene, "fence", path=RING, catalog=pack, height=2.0, door=(0, 1))
    by = rows(scene)
    # The fence as one product — the layout sheet labels it once, and it
    # carries the configuration. No mass on it: the panels have that.
    fence = by["fence"]
    assert fence["qty"] == 1 and fence["model"] == "Guard Fence"
    assert fence["manufacturer"] == "ACME Guarding"
    assert fence["catalog"].startswith(FENCE_ID)
    assert fence["attributes"]["height_mm"] == "2000"
    assert fence["attributes"]["mesh_mm"] == "49x49"
    assert "mass_kg" not in fence["attributes"]
    # One line per width, with the part number you would order it by.
    panels = {
        name: row for name, row in by.items() if name.startswith("fence/panels/")
    }
    # The door edge is packed around the door: 1000 door + 600 + 600.
    assert {row["model"] for row in panels.values()} == {
        "GP-2000x1000", "GP-2000x600", "GP-2000x200",
    }
    assert by["fence/panels/w1000"]["qty"] == 4 and by["fence/panels/w600"]["qty"] == 2
    assert by["fence/panels/w1000"]["attributes"]["mass_kg"] == pytest.approx(11.0)
    assert by["fence/posts"]["model"] == "GPP-2000"
    assert by["fence/posts"]["attributes"]["mass_kg"] == pytest.approx(7.0)
    # The door is as wide as the catalogue sells and weighs what the table says.
    assert by["fence/door"]["model"] == "GPD-2000x1000"
    assert by["fence/door"]["attributes"]["mass_kg"] == pytest.approx(18.0)
    # Totals are the sum of the parts, not of the fence line.
    total = sum(
        row["qty"] * row["attributes"]["mass_kg"]
        for row in by.values()
        if "mass_kg" in row.get("attributes", {})
    )
    assert scene.bom().total("mass_kg") == pytest.approx(total)


def test_a_height_nobody_sells_is_refused_with_the_ones_that_are(pack: Path) -> None:
    scene = scene_()
    with pytest.raises(ValueError, match="height_mm=1800 is not available.*1600 / 2000 / 2200"):
        bt.parts.fence(scene, "fence", path=RING, catalog=pack, height=1.8)
    # And nothing was left behind by the attempt.
    assert scene.obstacle_names == []


def test_an_edge_that_cannot_be_built_says_what_would_fit(pack: Path) -> None:
    scene = scene_()
    with pytest.raises(ValueError, match="cannot be built.*Nearest buildable: 460 mm / 520 mm"):
        bt.parts.fence(scene, "fence", path=[(0, 0), (0.5, 0)], catalog=pack, closed=False)


def test_parameters_are_chosen_by_name_and_recorded(pack: Path) -> None:
    scene = scene_()
    bt.parts.fence(scene, "fence", path=RING, catalog=pack, height=2.2, mesh_mm="20x20")
    fence = rows(scene)["fence"]
    assert fence["attributes"]["mesh_mm"] == "20x20"
    assert fence["attributes"]["height_mm"] == "2200"
    assert rows(scene)["fence/posts"]["model"] == "GPP-2200"
    with pytest.raises(ValueError, match="mesh_mm=10x10 is not available"):
        bt.parts.fence(scene, "f2", path=RING, catalog=pack, mesh_mm="10x10")


def test_the_bom_carries_the_article_number_the_maker_prints(coded: Path) -> None:
    """A dimension is not always what the article number says. Here 2200 mm
    is written 220 on the panel and 230 on the post — same fence, one order
    code per part — and a colour picks the post's prefix."""
    scene = scene_()
    bt.parts.fence(scene, "fence", path=RING, catalog=coded, post_finish="zinc-yellow")
    by = rows(scene)
    # 2400 mm edge = 1500 + 800 with 50 mm posts; 1600 = 800 + 400 + 250.
    assert {row["model"] for name, row in by.items() if "/panels/" in name} == {
        "W322-220150", "W322-220080", "W322-220040", "W322-220025",
    }
    assert by["fence/posts"]["model"] == "P11-230"
    assert by["fence"]["attributes"]["post_finish"] == "zinc-yellow"
    # Mesh by the square metre, frame by the edge: 2.4 x 2.2 x 1.5 + 0.0022 x 3700
    assert by["fence/panels/w1500"]["attributes"]["mass_kg"] == pytest.approx(16.06)


def test_an_order_code_the_pack_does_not_have_says_so(coded: Path, tmp_path: Path) -> None:
    directory = tmp_path / "holed"
    directory.mkdir()
    (directory / "manifest.yaml").write_text(
        CODED_MANIFEST.replace(', 2200: "230"', "").replace(', "2200": "230"', "")
    )
    scene = scene_()
    with pytest.raises(ValueError, match="post has no order code for height_mm=2200"):
        bt.parts.fence(scene, "fence", path=RING, catalog=directory)


def test_the_panel_pitch_still_caps_the_width(pack: Path) -> None:
    """With a catalog `panel_pitch` is an upper bound, not the pitch itself —
    the widths still come from what is sold."""
    scene = scene_()
    square = [(0, 0), (1.6, 0), (1.6, 1.6), (0, 1.6)]
    bt.parts.fence(scene, "fence", path=square, catalog=pack, panel_pitch=0.6)
    models = {row["model"] for name, row in rows(scene).items() if "/panels/" in name}
    assert models == {"GP-2000x600", "GP-2000x200"}
    # Ask for panels that cannot reach the corners and it says so.
    with pytest.raises(ValueError, match="cannot be built"):
        bt.parts.fence(scene, "wide", path=RING, catalog=pack, panel_pitch=0.6)


def test_the_layout_sheet_still_labels_the_fence_once(pack: Path, tmp_path: Path) -> None:
    """One product, one label — that is why the fence keeps a part of its own
    on top of the per-width lines."""
    scene = scene_()
    built = bt.parts.fence(scene, "fence", path=RING, catalog=pack, door=(0, 1))
    sheet = tmp_path / "layout.svg"
    scene.export_layout(sheet)
    assert sheet.read_text().count("Guard Fence") == 1
    built.remove(scene)
    assert scene.obstacle_names == [] and scene.parts() == []


def test_a_catalog_id_resolves_through_the_dataset_and_pins_the_revision(hub: dict) -> None:
    scene = scene_()
    bt.parts.fence(scene, "fence", path=RING, catalog=FENCE_ID, height=2.0)
    assert hub["allow_patterns"] == [f"{FENCE_ID}/*"]
    fence = rows(scene)["fence"]
    assert fence["catalog"] == f"{FENCE_ID}@{SHA}"


# ------------------------------------------------------------------ conveyor


def test_the_conveyor_is_a_size_you_can_order(belt: Path) -> None:
    scene = scene_()
    built = bt.parts.conveyor(scene, "conv", catalog=belt, position=(0.9, 0.0))
    # Omitted dimensions come from the catalog — including the stand height,
    # which is why (x, y) is enough to place it.
    lo, hi = scene.obstacle_bounds("conv/belt")
    assert hi[0] - lo[0] == pytest.approx(2.0) and hi[1] - lo[1] == pytest.approx(0.3)
    assert hi[2] == pytest.approx(0.75)
    row = rows(scene)["conv"]
    assert row["model"] == "BU-300x2000" and row["description"] == "Belt Unit"
    assert row["category"] == "conveyor.belt"
    assert row["catalog"].startswith(BELT_ID)
    # 8.0 + 0.0075 x 2000 + 0.02 x 300
    assert row["attributes"]["mass_kg"] == pytest.approx(29.0)
    assert row["attributes"]["speed_mps"] == "0.2"
    assert row["attributes"]["payload_kg_per_m"] == "25"
    # The stands are spaced by the catalog's maximum span: 2 m at 1.5 m -> 3.
    stands = rows(scene)["conv/stands"]
    assert (stands["model"], stands["qty"]) == ("BS-750", 3)
    assert stands["attributes"]["mass_kg"] == pytest.approx(5.0)
    assert scene.bom().total("mass_kg") == pytest.approx(29.0 + 3 * 5.0)
    assert built.devices == ["conv"] and sorted(built.frames) == ["conv/infeed", "conv/outfeed"]
    # A longer run needs more stands, and weighs more, without touching the code.
    built.remove(scene)
    bt.parts.conveyor(scene, "conv", catalog=belt, position=(0.9, 0.0), length=4.0, width=0.4)
    assert rows(scene)["conv"]["model"] == "BU-400x4000"
    assert rows(scene)["conv/stands"]["qty"] == 4


def test_a_speed_the_drive_cannot_reach_is_refused(belt: Path) -> None:
    """E2 の受け入れ条件 — `behavior:` は幾何より先に挙動を供給する。"""
    scene = scene_()
    with pytest.raises(ValueError, match=r"speed_mps=0.8 is out of range 0.02..0.5"):
        bt.parts.conveyor(scene, "conv", catalog=belt, position=(0.9, 0.0), speed=0.8)
    assert scene.obstacle_names == []
    # A speed inside the range is set on the device and recorded as ordered.
    bt.parts.conveyor(scene, "conv", catalog=belt, position=(0.9, 0.0), speed=0.35)
    assert rows(scene)["conv"]["attributes"]["speed_mps"] == "0.35"


def test_a_size_the_catalog_does_not_sell_is_refused(belt: Path) -> None:
    scene = scene_()
    with pytest.raises(ValueError, match=r"length_mm=4030 is out of range 600..4000"):
        bt.parts.conveyor(scene, "conv", catalog=belt, position=(0.9, 0.0), length=4.03)
    with pytest.raises(ValueError, match="length_mm=3030 is off the 100 step — nearest is 3000"):
        bt.parts.conveyor(scene, "conv", catalog=belt, position=(0.9, 0.0), length=3.03)
    with pytest.raises(ValueError, match="width_mm=350 is not available.*200 / 300 / 400"):
        bt.parts.conveyor(scene, "conv", catalog=belt, position=(0.9, 0.0), width=0.35)
    with pytest.raises(ValueError, match="height_mm=770 is off the 50 step"):
        bt.parts.conveyor(scene, "conv", catalog=belt, position=(0.9, 0.0, 0.77))


def test_the_conveyor_body_is_still_labelled_once(belt: Path, tmp_path: Path) -> None:
    """The belt and rails are the device's geometry, so the sheet labels them
    with the device — even though the stands make it a nested name. The
    stands have a part of their own, so they keep their line and their
    label."""
    scene = scene_()
    bt.parts.conveyor(scene, "conv", catalog=belt, position=(0.9, 0.0))
    sheet = tmp_path / "layout.svg"
    scene.export_layout(sheet)
    labels = re.findall(r"<text[^>]*>([^<]*)</text>", sheet.read_text())
    assert labels.count("conv") == 1
    assert "stands (BS-750)" in labels
    assert "belt" not in labels and "rail_l" not in labels


def test_a_conveyor_pack_is_not_a_fence_pack(belt: Path, pack: Path) -> None:
    scene = scene_()
    with pytest.raises(ValueError, match="is a `conveyor` package"):
        bt.parts.fence(scene, "fence", path=RING, catalog=belt)
    with pytest.raises(ValueError, match="is a `fence` package"):
        bt.parts.conveyor(scene, "conv", catalog=pack, position=(0.9, 0.0))


# ---------------------------------------------------------------------- rack


def test_the_rack_puts_a_frame_on_every_shelf(shelving: Path) -> None:
    """A rack is where parts sit, so what the cell needs from it is a frame
    per level — that is what a pick aims at."""
    scene = scene_()
    built = bt.parts.rack(scene, "rack", catalog=shelving, position=(1.0, 0.5))
    assert built.frames == [f"rack/level{i}" for i in range(4)]
    # Evenly spaced with the top shelf at the bay height.
    heights = [round(scene.frame(f)[0][2], 3) for f in built.frames]
    assert heights == [0.45, 0.9, 1.35, 1.8]
    for f in built.frames:
        x, y, _ = scene.frame(f)[0]
        assert (round(x, 3), round(y, 3)) == (1.0, 0.5)
    # Four uprights and one board per level.
    assert sum("/uprights/" in n for n in built.obstacles) == 4
    assert sum("/shelves/" in n for n in built.obstacles) == 4


def test_the_shelves_are_their_own_line_counted_from_the_levels(shelving: Path) -> None:
    scene = scene_()
    bt.parts.rack(scene, "rack", catalog=shelving, position=(1.0, 0.5), levels=6, size=(1.8, 0.6, 2.4))
    bay, shelves = rows(scene)["rack"], rows(scene)["rack/shelves"]
    assert bay["model"] == "SR-1800x600x2400" and bay["description"] == "Shelf Rack"
    assert bay["catalog"].startswith(RACK_ID)
    assert bay["attributes"]["levels"] == "6"
    # The datasheet number the generator does not use still reaches the bill.
    assert bay["attributes"]["capacity_kg_per_level"] == "300"
    # 4.0 + 0.006 x 2400 + 0.004 x 1800
    assert bay["attributes"]["mass_kg"] == pytest.approx(25.6)
    assert (shelves["model"], shelves["qty"]) == ("SR-SH-1800x600", 6)
    assert shelves["attributes"]["mass_kg"] == pytest.approx(1.0 + 1.8 * 0.6 * 8.0)
    assert scene.bom().total("mass_kg") == pytest.approx(25.6 + 6 * (1.0 + 1.8 * 0.6 * 8.0))


def test_levels_that_do_not_fit_the_bay_are_refused(shelving: Path) -> None:
    scene = scene_()
    with pytest.raises(ValueError, match="6 levels in 1200 mm leaves 200 mm"):
        bt.parts.rack(scene, "rack", catalog=shelving, position=(1.0, 0.5), levels=6, height_mm=1200)
    with pytest.raises(ValueError, match="width_mm=1300 is not available"):
        bt.parts.rack(scene, "rack", catalog=shelving, position=(1.0, 0.5), size=(1.3, 0.6, 1.8))
    with pytest.raises(ValueError, match=r"levels=8 is out of range 2..6"):
        bt.parts.rack(scene, "rack", catalog=shelving, position=(1.0, 0.5), levels=8)
    assert scene.obstacle_names == []


def test_the_rack_sheet_labels_the_bay_once(shelving: Path, tmp_path: Path) -> None:
    scene = scene_()
    bt.parts.rack(scene, "rack", catalog=shelving, position=(1.0, 0.5))
    sheet = tmp_path / "layout.svg"
    scene.export_layout(sheet)
    labels = re.findall(r"<text[^>]*>([^<]*)</text>", sheet.read_text())
    assert labels.count("rack (SR-1200x600x1800)") == 1
    assert "shelves" not in labels


PARTIAL_MANIFEST = """
schema_version: '0.1'
id: acme/rack/half/r1
kind: spec
category: structure.rack
name: Half Spec Rack
manufacturer:
  name: ACME Storage
distribution: public
configuration:
  generator: rack
  params:
    width_mm: {values: [1200], default: 1200}
  components:
    - role: bay
      category: structure.rack
      part_number: HR-{width_mm}
"""


def test_a_pack_that_sizes_only_some_of_it_leaves_the_rest_to_the_caller(tmp_path: Path) -> None:
    """Not every maker sells every axis from a list. A pack that names only
    the ones it does still builds — the caller supplies the rest, and asking
    for what it cannot size says so."""
    directory = tmp_path / "half"
    directory.mkdir()
    (directory / "manifest.yaml").write_text(PARTIAL_MANIFEST)
    scene = scene_()
    built = bt.parts.rack(scene, "rack", (1.2, 0.6, 1.8), (0.0, 0.0), catalog=directory, levels=3)
    assert len(built.frames) == 3
    assert rows(scene)["rack"]["model"] == "HR-1200"
    # No shelf component: no second line, and the generator keeps its own
    # board thickness.
    assert "rack/shelves" not in rows(scene)
    with pytest.raises(ValueError, match="does not size the depth, height — pass size="):
        bt.parts.rack(scene, "other", catalog=directory, position=(2.0, 0.0))


def test_shelving_bought_as_posts_and_shelves_lists_both(wire: Path) -> None:
    """No bay article to name, so the group line carries the series and the
    posts get their own — four of them, in the pairs they are sold in. The
    finish is one choice with a different code on each part."""
    scene = scene_()
    built = bt.parts.rack(scene, "shelf", catalog=wire, position=(0.0, 0.0))
    assert len(built.frames) == 4
    by = rows(scene)
    series = by["shelf"]
    assert series["model"] == "Wire Shelving" and series["qty"] == 1
    assert "mass_kg" not in series["attributes"]          # 二重計上しない
    assert by["shelf/uprights"]["model"] == "B74PS2"      # ステンレスポール 1900
    assert by["shelf/uprights"]["qty"] == 2               # 4 本 = 2 本入り x 2
    assert by["shelf/uprights"]["attributes"]["mass_kg"] == pytest.approx(2.8)
    assert by["shelf/shelves"]["model"] == "B1848C1"      # 奥行 450 x 間口 1200 クローム
    assert by["shelf/shelves"]["qty"] == 4
    assert scene.bom().total("mass_kg") == pytest.approx(2 * 2.8 + 4 * 4.0)
    # 仕上げは 1 つの軸だが、ポールは S、棚板は C — 部材ごとのコード表が効く
    other = scene_()
    bt.parts.rack(other, "shelf", catalog=wire, position=(0.0, 0.0), finish="white", depth_mm=600)
    assert rows(other)["shelf/uprights"]["model"] == "B74PW2"
    assert rows(other)["shelf/shelves"]["model"] == "B2448W1"


# ------------------------------------------------------------ stand, pedestal


def test_the_bench_is_a_size_you_can_order_and_the_board_is_its_own_line(stand: Path) -> None:
    """An aluminium stand is bought as a frame plus a board — two articles,
    two BOM lines — and the sizes are the ones the maker cuts to."""
    scene = scene_()
    built = bt.parts.table(scene, "bench", catalog=stand, position=(0.0, 0.0))
    # The pack sizes it, so `position` alone was enough.
    lo, hi = scene.obstacle_bounds("bench/top")
    assert (round(hi[0] - lo[0], 3), round(hi[1] - lo[1], 3)) == (1.2, 0.6)
    assert hi[2] == pytest.approx(0.75)
    assert built.frames == ["bench/top"]          # where the work sits
    by = rows(scene)
    frame = by["bench"]
    assert frame["model"] == "FB-1200x600x750" and frame["manufacturer"] == "ACME Framing"
    assert frame["description"] == "Frame Bench"
    assert frame["catalog"].startswith("acme/stand/frame-bench/r1")
    # 6.0 + 0.008x1200 + 0.006x600 + 0.009x750
    assert frame["attributes"]["mass_kg"] == pytest.approx(25.95)
    board = by["bench/top"]
    assert board["model"] == "FB-TOP-1200x600" and board["qty"] == 1
    assert board["attributes"]["mass_kg"] == pytest.approx(11.08)   # 1.0 + 0.72 x 14
    assert scene.bom().total("mass_kg") == pytest.approx(25.95 + 11.08)


def test_a_bench_size_nobody_cuts_is_refused(stand: Path) -> None:
    scene = scene_()
    with pytest.raises(ValueError, match=r"width_mm=1000 is not available — choose from 900 / 1200 / 1500"):
        bt.parts.table(scene, "bench", (1.0, 0.6, 0.75), (0.0, 0.0), catalog=stand)
    with pytest.raises(ValueError, match="height_mm=850 is not available"):
        bt.parts.table(scene, "bench", (1.2, 0.6, 0.85), (0.0, 0.0), catalog=stand)


def test_the_pedestal_names_the_stand_the_robot_is_bolted_to(pillar: Path) -> None:
    """The mount frame is the robot's base pose, and the line under it says
    which stand was ordered — the finish is part of that number."""
    scene = scene_()
    built = bt.parts.pedestal(scene, "ped", catalog=pillar, position=(1.0, 0.0))
    assert built.frames == ["ped/mount"]
    assert scene.frame("ped/mount")[0][2] == pytest.approx(0.5)     # the pack's height
    lo, hi = scene.obstacle_bounds("ped/base")
    assert (round(hi[0] - lo[0], 3), round(hi[1] - lo[1], 3)) == (0.45, 0.45)
    lo, hi = scene.obstacle_bounds("ped/column")
    assert round(hi[0] - lo[0], 3) == 0.18
    row = rows(scene)["ped"]
    assert row["model"] == "RP-500K" and row["attributes"]["finish"] == "painted"
    assert row["attributes"]["mass_kg"] == pytest.approx(37.0)      # 12.0 + 0.05 x 500
    # The finish is an axis of the part number, chosen by name.
    scene2 = scene_()
    bt.parts.pedestal(scene2, "ped", catalog=pillar, position=(0.0, 0.0), height=0.7, finish="plated")
    assert rows(scene2)["ped"]["model"] == "RP-700P"
    with pytest.raises(ValueError, match="height_mm=550 is not available"):
        bt.parts.pedestal(scene_(), "ped", catalog=pillar, position=(0.0, 0.0), height=0.55)


def test_a_stand_pack_without_a_height_leaves_it_to_the_caller(tmp_path: Path) -> None:
    """Not every stand is sold by height (a maker may cut the column to
    order), and the generator says so rather than guessing."""
    directory = tmp_path / "cut"
    directory.mkdir()
    (directory / "manifest.yaml").write_text(
        PILLAR_MANIFEST.replace("""    height_mm:
      values: [400, 500, 600, 700]
      default: 500
""", "")
        .replace("RP-{height_mm}{finish_code}", "RP-CUT{finish_code}")
        .replace("        per_mm: {height_mm: 0.05}\n", "")
    )
    scene = scene_()
    bt.parts.pedestal(scene, "ped", 0.62, (0.0, 0.0), catalog=directory)
    assert scene.frame("ped/mount")[0][2] == pytest.approx(0.62)
    assert rows(scene)["ped"]["model"] == "RP-CUTK"
    with pytest.raises(ValueError, match="does not size the height — pass height="):
        bt.parts.pedestal(scene_(), "ped", catalog=directory, position=(0.0, 0.0))
    # ...and a pack that weighs a part by an axis it does not sell says so,
    # rather than quietly leaving the term out.
    (directory / "manifest.yaml").write_text(
        (directory / "manifest.yaml").read_text().replace(
            "        base_kg: 12.0\n", "        base_kg: 12.0\n        per_mm: {height_mm: 0.05}\n"
        )
    )
    with pytest.raises(ValueError, match="the mass of 'pedestal' needs height_mm"):
        bt.parts.pedestal(scene_(), "ped", 0.62, (0.0, 0.0), catalog=directory)


def test_a_stand_cut_to_order_still_names_one_article(tmp_path: Path) -> None:
    """A height given to the millimetre still lands on a part number, because
    the pack codes the axis in bands — and the mass follows the cut."""
    directory = tmp_path / "cut"
    directory.mkdir()
    (directory / "manifest.yaml").write_text(BAND_MANIFEST)
    scene = scene_()
    bt.parts.pedestal(scene, "ped", 0.75, (0.0, 0.0), catalog=directory)
    row = rows(scene)["ped"]
    assert row["model"] == "ZFR-F037"
    assert row["attributes"]["mass_kg"] == pytest.approx(66.4)      # 30.4 + 0.048 x 750
    for height, code in ((0.2, "2"), (0.799, "7"), (0.8, "8"), (1.0, "A"), (1.3, "C")):
        one = scene_()
        bt.parts.pedestal(one, "ped", height, (0.0, 0.0), catalog=directory)
        assert rows(one)["ped"]["model"] == f"ZFR-F03{code}"
    with pytest.raises(ValueError, match=r"height_mm=1400 is out of range 200\.\.1300"):
        bt.parts.pedestal(scene_(), "ped", 1.4, (0.0, 0.0), catalog=directory)


def test_the_cabinet_names_the_enclosure_and_its_articles(enclosure: Path) -> None:
    """A control cabinet is customised the way the maker sells it: a size
    from the matrix, a plinth base and a mounting plate as articles of their
    own. Three lines, each with the number you would order it by — what is
    *inside* the cabinet is other people's BOM lines."""
    scene = scene_()
    built = bt.parts.cabinet(scene, "cab", catalog=enclosure, position=(1.0, 0.5))
    # The massing: plinth, body on top of it, plate standing inside.
    massing = sorted(n for n in built.obstacles if "/trim/" not in n)
    assert massing == ["cab/base", "cab/body", "cab/plate"]
    lo, hi = scene.obstacle_bounds("cab/body")
    assert (round(hi[0] - lo[0], 3), round(hi[1] - lo[1], 3)) == (0.8, 0.6)
    assert (lo[2], hi[2]) == (pytest.approx(0.1), pytest.approx(2.2))   # on its plinth
    # The door-face frame, floor level — where an operator stands.
    assert built.frames[:1] == ["cab/front"]
    fx, fy, fz = scene.frame("cab/front")[0]
    assert (fx, fy, fz) == (pytest.approx(1.0), pytest.approx(0.2), pytest.approx(0.0))
    # Three orderable lines: enclosure + base + plate.
    table = rows(scene)
    body, plinth, mount = table["cab"], table["cab/base"], table["cab/plate"]
    assert body["model"] == "PE60-821" and body["qty"] == 1
    assert body["attributes"]["mass_kg"] == pytest.approx(150.0)
    assert body["attributes"]["width_mm"] == "800" and body["attributes"]["ip_rating"] == "IP55"
    assert plinth["model"] == "PB-86010"
    assert plinth["attributes"]["mass_kg"] == pytest.approx(8.4)        # 2.0 + 0.008 x 800
    assert mount["model"] == "PP-821"
    assert mount["attributes"]["mass_kg"] == pytest.approx(30.24)       # 0.8 x 2.1 x 18
    # A width nobody sells is refused with the ones that are.
    with pytest.raises(ValueError, match="width_mm=700 is not available — choose from 600 / 800 / 1000"):
        bt.parts.cabinet(scene_(), "cab", (0.7, 0.6, 2.1), (0.0, 0.0), catalog=enclosure)
    # The plinth height is an axis of its own: 50 or 100 mm, in the number.
    low = scene_()
    bt.parts.cabinet(low, "cab", catalog=enclosure, position=(0.0, 0.0), base_height=0.05)
    assert rows(low)["cab/base"]["model"] == "PB-86005"
    lo, hi = low.obstacle_bounds("cab/body")
    assert lo[2] == pytest.approx(0.05)
    with pytest.raises(ValueError, match="base_height_mm=70 is not available"):
        bt.parts.cabinet(scene_(), "cab", catalog=enclosure, position=(0.0, 0.0), base_height=0.07)
    # Leave the articles out and their lines go with them.
    bare = scene_()
    built = bt.parts.cabinet(bare, "cab", catalog=enclosure, position=(0.0, 0.0),
                             base=False, plate=False)
    assert sorted(n for n in built.obstacles if "/trim/" not in n) == ["cab/body"]
    assert {n for n in rows(bare) if n.startswith("cab")} == {"cab"}
    lo, hi = bare.obstacle_bounds("cab/body")
    assert (lo[2], hi[2]) == (pytest.approx(0.0), pytest.approx(2.1))   # straight on the floor


def test_a_cabinet_size_nobody_sells_is_stopped_by_the_mass_table(enclosure: Path) -> None:
    """The axes are independent but the sold sizes are not a full grid — the
    hole in the mass table is what stops a combination nobody sells."""
    with pytest.raises(ValueError, match="no mass row for body"):
        bt.parts.cabinet(scene_(), "cab", (0.6, 0.6, 2.1), (0.0, 0.0), catalog=enclosure)
    # The same size with the depth it is sold in is fine.
    scene = scene_()
    bt.parts.cabinet(scene, "cab", (0.6, 0.4, 1.6), (0.0, 0.0), catalog=enclosure)
    assert rows(scene)["cab"]["model"] == "PE40-616"


def test_a_hand_written_cabinet_is_one_line(enclosure: Path) -> None:
    """Without a catalog the cabinet works like every hand-written part: a
    box (no plinth, no plate, no trim) and one line saying what it is."""
    scene = scene_()
    built = bt.parts.cabinet(scene, "cab", (0.5, 0.4, 1.2), (0.0, 0.0),
                             model="PNL-500", manufacturer="ACME Panels")
    assert built.obstacles == ["cab/body"] and built.frames == ["cab/front"]
    row = rows(scene)["cab"]
    assert row["model"] == "PNL-500" and row["category"] == "structure.cabinet"
    with pytest.raises(ValueError, match="size is required without a catalog"):
        bt.parts.cabinet(scene_(), "cab")


def test_a_variant_table_is_the_article_list(tmp_path: Path) -> None:
    """Some article numbers cannot be composed from the axes (Rittal's
    1050.000 is 500x500x210 — a plain running number). The pack lists each
    sold combination with its number and weight instead, and a combination
    it does not list is refused as *not sold*, not as a missing mass row."""
    directory = tmp_path / "axbox"
    directory.mkdir()
    (directory / "manifest.yaml").write_text(AX_MANIFEST)
    scene = scene_()
    bt.parts.cabinet(scene, "box", catalog=directory, position=(0.0, 2.0, 1.2))
    row = rows(scene)["box"]
    assert row["model"] == "AX 1180.000"
    assert row["attributes"]["mass_kg"] == pytest.approx(49.7)
    # Another sold combination, sized explicitly (width, depth, height).
    scene2 = scene_()
    bt.parts.cabinet(scene2, "box", (0.6, 0.4, 0.8), (1.0, 0.0, 1.2), catalog=directory)
    assert rows(scene2)["box"]["model"] == "AX 1059.000"
    # Every axis value is legal on its own; the combination is what is not sold.
    with pytest.raises(ValueError, match="no body is sold at width_mm=600 x height_mm=1000 x depth_mm=300"):
        bt.parts.cabinet(scene_(), "box", (0.6, 0.3, 1.0), (0.0, 0.0), catalog=directory)


# -------------------------------------------------------------------- detail


def _drawn(scene, tmp_path: Path) -> dict:
    """Every obstacle as (enabled, visible), read back off a saved project."""
    project = tmp_path / "cell.botrail"
    scene.save_project(project)
    data = json.loads(project.read_text())
    return {o["name"]: (o.get("enabled", True), o.get("visible", True)) for o in data["obstacles"]}


def test_full_detail_draws_the_machine_without_changing_what_it_hits(
    pack: Path, belt: Path, shelving: Path, stand: Path, pillar: Path,
    enclosure: Path, tmp_path: Path
) -> None:
    """`detail="full"` is decoration: a mesh panel gets its frame and wire, a
    conveyor its rollers and drive, a rack its beams and braces, a stand its
    rails and feet, a cabinet its doors and handles — all of it out of
    collision, with the massing underneath untouched. How a cell looks never
    changes how it verifies."""
    def build(mode: str):
        scene = scene_()
        bt.parts.fence(scene, "fence", path=RING, catalog=pack, door=(0, 1), detail=mode)
        bt.parts.conveyor(scene, "conv", catalog=belt, position=(0.0, 2.0), detail=mode)
        bt.parts.rack(scene, "rack", catalog=shelving, position=(2.0, 2.0), detail=mode)
        bt.parts.table(scene, "bench", catalog=stand, position=(-2.0, 2.0), detail=mode)
        bt.parts.pedestal(scene, "ped", catalog=pillar, position=(-2.0, -2.0), detail=mode)
        bt.parts.cabinet(scene, "cab", catalog=enclosure, position=(2.0, -2.0), detail=mode)
        return scene

    plain, full = build("plain"), build("full")
    massing = sorted(plain.obstacle_names)
    assert massing == sorted(n for n in full.obstacle_names if "/trim/" not in n)
    assert all(plain.obstacle_bounds(n) == full.obstacle_bounds(n) for n in massing)
    assert plain.bom().to_markdown() == full.bom().to_markdown()
    # Detail is worth having: it is most of what you see, and none of what you hit.
    trim = [n for n in full.obstacle_names if "/trim/" in n]
    assert len(trim) > len(massing)
    flags = _drawn(full, tmp_path)
    assert all(flags[n] == (False, True) for n in trim)          # drawn, not collided
    # The panels themselves stop being drawn — the frame and wire stand in.
    panels = [n for n in massing if "/panels/" in n]
    assert panels and all(flags[n] == (True, False) for n in panels)
    assert {
        "fence/trim/e0_0/frame_t", "conv/trim/drive", "rack/trim/brace_l",
        "bench/trim/rail0", "bench/trim/foot0", "ped/trim/gusset0",
        "cab/trim/door0", "cab/trim/handle0",
    } <= set(trim)


def test_a_hand_written_part_stays_the_plain_massing(shelving: Path, tmp_path: Path) -> None:
    """Without a catalog there are no real sections to draw, so nothing
    changes for anyone who was already calling these."""
    scene = scene_()
    built = bt.parts.rack(scene, "rack", (1.2, 0.6, 1.8), (0.0, 0.0), levels=3)
    assert not any("/trim/" in n for n in built.obstacles)
    # ...and asking for it anyway is allowed.
    bt.parts.rack(scene, "fancy", (1.2, 0.6, 1.8), (3.0, 0.0), levels=3, detail="full")
    assert any("/trim/" in n for n in scene.obstacle_names)
    with pytest.raises(ValueError, match="detail must be one of"):
        bt.parts.rack(scene, "nope", (1.0, 1.0, 1.0), (6.0, 0.0), detail="shiny")


TRIM_PANEL = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="panel">
  <xacro:arg name="width" default="1.0"/>
  <xacro:arg name="height" default="2.0"/>
  <xacro:arg name="frame" default="0.03"/>
  <xacro:property name="w" value="$(arg width)"/>
  <xacro:property name="h" value="$(arg height)"/>
  <xacro:property name="f" value="$(arg frame)"/>
  <link name="panel"/>
  <link name="rail">
    <visual>
      <origin xyz="0 0 ${h - f/2}"/>
      <geometry><box size="${w} 0.03 ${f}"/></geometry>
    </visual>
  </link>
  <joint name="j" type="fixed"><parent link="panel"/><child link="rail"/></joint>
</robot>
"""


def test_a_pack_can_bring_its_own_drawing(pack: Path, tmp_path: Path) -> None:
    """The look belongs to the product: a pack that ships a file of
    primitives is drawn from it, expanded to the size at hand, instead of the
    generator's built-in shapes. It is still decoration — the massing and the
    BOM do not move."""
    (pack / "panel.urdf.xacro").write_text(TRIM_PANEL)
    manifest = pack / "manifest.yaml"
    manifest.write_text(
        manifest.read_text().replace(
            "      part_number: GP-{height_mm}x{width_mm}",
            "      part_number: GP-{height_mm}x{width_mm}\n      trim: panel.urdf.xacro",
        )
    )
    scene = scene_()
    bt.parts.fence(scene, "fence", path=RING, catalog=pack, height=2.0)
    drawn = [n for n in scene.obstacle_names if n.startswith("fence/trim/")]
    rails = [n for n in drawn if n.endswith("/rail")]
    # One rail per panel, from the file — the built-in frame and wire are gone.
    assert len(rails) == sum("/panels/" in n for n in scene.obstacle_names)
    assert not any("wire_v" in n for n in drawn)
    # The posts name no file of their own, so they keep the built-in look:
    # the fallback is per part, not per pack.
    assert any(n.startswith("fence/trim/base_") for n in drawn)
    # The rail is as wide as the panel the catalog picked for that bay.
    widths = {round((hi[0] - lo[0]) * 1000) if abs(hi[0] - lo[0]) > abs(hi[1] - lo[1])
              else round((hi[1] - lo[1]) * 1000)
              for lo, hi in map(scene.obstacle_bounds, rails)}
    assert widths <= {200, 400, 600, 800, 1000, 1200}
    # ...and it hangs at the top of the panel, which the file computed.
    lo, hi = scene.obstacle_bounds(rails[0])
    assert hi[2] == pytest.approx(2.0)
    flags = _drawn(scene, tmp_path)
    assert all(flags[n] == (False, True) for n in drawn)
