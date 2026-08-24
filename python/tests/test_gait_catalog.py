"""`bt.Gait.from_catalog`: a vehicle.legged package's `locomotion` block
read as a Gait — from a package directory on disk, or by catalog id."""

from pathlib import Path

import botrail as bt
import pytest
import yaml

LOCOMOTION = {
    "kind": "quadruped",
    "body_frame": "base",
    "legs": [
        {"name": "FL", "foot": "FL_foot", "contact": "point"},
        {"name": "FR", "foot": "FR_foot", "contact": "point"},
        {"name": "RL", "foot": "RL_foot", "contact": "point"},
        {"name": "RR", "foot": "RR_foot", "contact": "point"},
    ],
    "stance": {f"{leg}_{j}_joint": v for leg in ("FL", "FR", "RL", "RR")
               for j, v in (("hip", 0), ("thigh", 0.8), ("calf", -1.5))},
    "foot_radius_m": 0.022,
    "gait": {"pattern": "trot", "period_s": 0.45, "lift_m": 0.07, "max_stride_m": 0.45,
             "bob_m": 0.0, "lateral_m": 0.0},
    "arm_swing": {},
}


def _package(tmp_path: Path, **manifest) -> Path:
    data = {"id": "unitree/go2/go2/r1", "category": "vehicle.legged", "name": "Go2"}
    data.update(manifest)
    (tmp_path / "manifest.yaml").write_text(yaml.safe_dump(data), encoding="utf-8")
    return tmp_path


def test_a_package_directory_yields_the_declared_gait(tmp_path: Path) -> None:
    gait = bt.Gait.from_catalog(_package(tmp_path, locomotion=LOCOMOTION))
    assert list(gait.legs) == ["FL", "FR", "RL", "RR"]
    assert gait.legs["RR"] == ("RR_foot", "point")
    assert gait.stance["FL_calf_joint"] == -1.5 and isinstance(gait.stance["FL_hip_joint"], float)
    assert (gait.pattern, gait.period, gait.lift, gait.max_stride) == ("trot", 0.45, 0.07, 0.45)
    assert gait.foot_radius == 0.022 and gait.body_link == "base"
    assert gait.arm_swing == {} and gait.bob == 0.0 and gait.lateral == 0.0
    # and it is a Gait the extension accepts
    spec = gait._spec()
    assert spec["legs"][0] == ("FL", "FL_foot", "point") and len(spec["stance"]) == 12


def test_the_manifest_file_itself_and_overrides_are_accepted(tmp_path: Path) -> None:
    package = _package(tmp_path, locomotion=LOCOMOTION)
    gait = bt.Gait.from_catalog(package / "manifest.yaml", period=0.6, lift=0.05)
    assert gait.period == 0.6 and gait.lift == 0.05 and gait.max_stride == 0.45


def test_a_biped_block_carries_soles_sway_and_arm_swing(tmp_path: Path) -> None:
    biped = {
        "kind": "biped",
        "body_frame": "pelvis",
        "legs": [{"name": "L", "foot": "left_ankle_roll_link", "contact": "sole"},
                 {"name": "R", "foot": "right_ankle_roll_link", "contact": "sole"}],
        "stance": {"left_knee_joint": 0.8, "right_knee_joint": 0.8},
        "foot_radius_m": 0.035,
        "gait": {"pattern": "biped", "period_s": 0.85, "lift_m": 0.05, "max_stride_m": 0.5,
                 "bob_m": 0.015, "lateral_m": 0.02},
        "arm_swing": {"left_shoulder_pitch_joint": -0.25, "right_shoulder_pitch_joint": 0.25},
    }
    gait = bt.Gait.from_catalog(_package(tmp_path, id="unitree/g1/g1/r1", locomotion=biped))
    assert gait.legs == {"L": ("left_ankle_roll_link", "sole"), "R": ("right_ankle_roll_link", "sole")}
    assert gait.pattern == "biped" and gait.bob == 0.015 and gait.lateral == 0.02
    assert gait.arm_swing == {"left_shoulder_pitch_joint": -0.25, "right_shoulder_pitch_joint": 0.25}
    assert gait._spec()["legs"][1] == ("R", "right_ankle_roll_link", "sole")


STAIRS = {
    "stance": {f"{leg}_{j}_joint": v for leg in ("FL", "FR", "RL", "RR")
               for j, v in (("hip", 0), ("thigh", 0.9436), ("calf", -1.8873))},
    "lift_m": 0.015,
}


def test_a_package_states_which_postures_it_carries(tmp_path: Path) -> None:
    """A cell that can do either asks, rather than requiring the posture and
    handling the refusal."""
    assert bt.Gait.postures(_package(tmp_path, locomotion=LOCOMOTION)) == ()
    with_stairs = _package(tmp_path, locomotion={**LOCOMOTION, "stairs": STAIRS})
    assert bt.Gait.postures(with_stairs) == ("stairs",)


def test_the_stair_posture_replaces_the_stance_and_the_swing(tmp_path: Path) -> None:
    package = _package(tmp_path, locomotion={**LOCOMOTION, "stairs": STAIRS},
                       specs={"max_step_height_mm": 160})
    stairs = bt.Gait.from_catalog(package, posture="stairs")
    assert stairs.stance["FL_thigh_joint"] == 0.9436
    assert stairs.lift == 0.015
    # everything else is the package's walk, and the rating still rides along
    assert (stairs.pattern, stairs.period, stairs.max_stride) == ("trot", 0.45, 0.45)
    assert stairs.max_step == 0.16

    standing = bt.Gait.from_catalog(package)
    assert standing.stance["FL_thigh_joint"] == 0.8 and standing.lift == 0.07

    # an override still wins over the posture
    assert bt.Gait.from_catalog(package, posture="stairs", lift=0.02).lift == 0.02


def test_a_posture_the_package_does_not_carry_is_refused_by_name(tmp_path: Path) -> None:
    package = _package(tmp_path, locomotion=LOCOMOTION)
    with pytest.raises(ValueError) as refusal:
        bt.Gait.from_catalog(package, posture="stairs")
    assert "locomotion.stairs" in str(refusal.value)
    assert "unitree/go2/go2/r1" in str(refusal.value)

    with pytest.raises(ValueError, match="posture must be one of"):
        bt.Gait.from_catalog(_package(tmp_path, locomotion={**LOCOMOTION, "stairs": STAIRS}),
                             posture="crouch")


def test_a_package_without_legs_is_refused_by_name(tmp_path: Path) -> None:
    package = _package(tmp_path, id="universal_robots/ur/ur5e/r1", category="manipulator")
    with pytest.raises(ValueError, match=r"universal_robots/ur/ur5e/r1.*no `locomotion` block.*manipulator"):
        bt.Gait.from_catalog(package)
    (tmp_path / "empty").mkdir()
    with pytest.raises(FileNotFoundError, match="no manifest.yaml"):
        bt.Gait.from_catalog(tmp_path / "empty")


def test_a_catalog_id_is_fetched_through_the_hub(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    package = _package(tmp_path, locomotion=LOCOMOTION)
    asked: list = []

    def fake(id: str, *, revision=None) -> str:
        asked.append((id, revision))
        return str(package)

    monkeypatch.setattr(bt._core, "catalog_package", fake)
    gait = bt.Gait.from_catalog("unitree/go2/go2", revision="abc123")
    assert asked == [("unitree/go2/go2", "abc123")]
    assert gait.max_stride == 0.45
