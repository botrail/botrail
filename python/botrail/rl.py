"""Reinforcement-learning environments derived from a cell.

    from botrail import rl

    task = rl.Task(robot="arm", control=rl.TcpDelta(max_step_m=0.01, hz=20),
                   observe=[rl.Joints(), rl.Relative("tcp", "part"), rl.Contacts("arm/tool0", "part")],
                   reset=randomize, reward=rl.rewards.reach("tcp→part"), horizon_s=6.0)
    env = rl.make(build, task, seed=0)          # gymnasium.Env when gymnasium is installed
    obs, info = env.reset()
    obs, reward, terminated, truncated, info = env.step(action)
    tl = env.timeline()                          # the episode as an ordinary SequenceTimeline

A `Task` names the robot a policy drives, how (`JointDelta`, `JointTarget`,
`TcpDelta`), what it observes (channels), and — as plain Python functions —
how each episode is randomised, rewarded and ended. The environment is the
cell's own rollout opened tick by tick (`Scene.open_rollout`): the PLC
programs, devices, sensors and physics of the cell run underneath the
policy exactly as in a batch bake, and an episode closes into the same
timeline every other tool reads (studio, USD, `min_clearance`, capture).

Determinism: botrail holds no random number. The only randomness is the
numpy generator handed to `Task.reset`, seeded from `seed`; the same seed
reproduces an episode bit for bit. Actions are the policy's business —
the drive rate-limits them to the joint velocity limits, so a policy can
steer but never teleport the arm.

Requires numpy (`pip install 'botrail[rl]'`); gymnasium is optional — with
it the environment is a `gymnasium.Env` and the spaces are gymnasium's,
without it the same `reset`/`step` contract runs on a small shim.
"""

from __future__ import annotations

import json
import math
from collections.abc import Sequence
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any, Callable, Optional, Union

try:
    import numpy as np
except ImportError as e:  # pragma: no cover - import-time guard
    raise ImportError("botrail.rl needs numpy: pip install 'botrail[rl]'") from e

try:
    import gymnasium as _gym
    from gymnasium import spaces as _spaces
    from gymnasium import vector as _vector
except ImportError:  # pragma: no cover - gymnasium is optional
    _gym = None
    _spaces = None
    _vector = None

Pose = tuple[tuple[float, float, float], tuple[float, float, float, float]]
ObsDict = dict[str, "np.ndarray"]
Info = dict[str, Any]

__all__ = [
    "Channel",
    "Clearance",
    "Collision",
    "Contacts",
    "Depth",
    "Env",
    "GravityTorque",
    "JointDelta",
    "JointTarget",
    "Joints",
    "Lidar",
    "LinkPose",
    "ObjectPose",
    "ObjectVel",
    "PointCloud",
    "Policy",
    "Relative",
    "Rgb",
    "Segmentation",
    "Signal",
    "Task",
    "TcpDelta",
    "TcpPose",
    "Tiled",
    "Torque",
    "VecEnv",
    "load",
    "make",
    "play",
    "randomize",
    "rewards",
    "segmentation_ids",
    "single",
    "tile",
]

# ------------------------------------------------------------ quaternions
# xyzw, as everywhere in botrail's Python face.


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _qconj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def _qrot(q, v):
    """Rotates vector `v` by unit quaternion `q`."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 q_v × v ; v' = v + w t + q_v × t
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _qnorm(q):
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    return tuple(c / n for c in q)


def _rotvec_to_quat(r):
    """A small rotation vector (axis × angle) as a unit quaternion."""
    angle = math.sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2])
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(angle / 2.0) / angle
    return (r[0] * s, r[1] * s, r[2] * s, math.cos(angle / 2.0))


def _relative(a: Pose, b: Pose) -> Pose:
    """Pose of `b` in `a`'s frame."""
    (pa, qa), (pb, qb) = a, b
    inv = _qconj(qa)
    d = (pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2])
    return _qrot(inv, d), _qnorm(_qmul(inv, qb))


def _pose_vec(pose: Pose) -> np.ndarray:
    (x, y, z), (qx, qy, qz, qw) = pose
    return np.array([x, y, z, qx, qy, qz, qw], dtype=np.float64)


# ------------------------------------------------------------ references


def _ref_pose(live, robot: str, ref: str) -> Pose:
    """Resolves a pose reference: an obstacle name, `tcp` (the task robot's
    TCP), `<robot>/tcp`, or `<robot>/<link>`."""
    if ref == "tcp":
        return live.tcp_pose(robot)
    if "/" in ref:
        who, what = ref.split("/", 1)
        if what == "tcp":
            return live.tcp_pose(who)
        return live.link_pose(what, who)
    return live.object_pose(ref)


# ------------------------------------------------------------ channels


class Channel:
    """One named block of the observation. Subclasses set `key` and `dim`
    (once `bind` has seen the scene) and `read` the live rollout."""

    key: str = ""
    dim: int = 0
    #: The array shape a channel's block is read as (`None`: the flat
    #: `(dim,)` vector); a picture reads as `(h, w)`, a point cloud as
    #: `(h·w, 3)`.
    shape: Optional[tuple[int, ...]] = None

    def bind(self, scene, robot: str) -> None:
        """Resolves names against the built scene (dimensions, defaults)."""

    def read(self, live, robot: str) -> np.ndarray:
        raise NotImplementedError

    def lower(self, robot: str) -> dict:
        """This channel as the spec the Rust packer of a `VecEnv` reads."""
        raise NotImplementedError

    def __repr__(self) -> str:
        return f"{type(self).__name__}({self.key!r}, dim={self.dim})"


class GravityTorque(Channel):
    """The torque per joint that holds a dynamic robot still against
    gravity as the world stands (`dof` values, N·m or N): the simulated
    link masses about each joint axis, what a controller adds as gravity
    compensation. Needs a robot declared dynamic (`Scene.set_robot_physics`).
    Pair it with `Torque()` to let a policy see the load it carries, or
    read `live.gravity_torques()` from a controller of your own."""

    def __init__(self, robot: Optional[str] = None, *, name: Optional[str] = None):
        self.robot = robot
        self._name = name

    def bind(self, scene, robot: str) -> None:
        self.robot = self.robot or robot
        if not scene.robot_physics(self.robot):
            raise ValueError(f"GravityTorque needs robot {self.robot!r} declared dynamic: scene.set_robot_physics({self.robot!r})")
        self.dim = scene.robot_of(self.robot).dof
        self.key = self._name or f"{self.robot}/gravity"

    def read(self, live, robot: str) -> np.ndarray:
        return np.asarray(live.gravity_torques(self.robot), dtype=np.float64)

    def lower(self, robot: str) -> dict:
        return {"kind": "gravity_torque", "robot": self.robot or robot}


class Joints(Channel):
    """Joint positions (and velocities) of a robot: `2·dof` (or `dof`)."""

    def __init__(self, robot: Optional[str] = None, *, velocities: bool = True, name: Optional[str] = None):
        self.robot = robot
        self.velocities = velocities
        self._name = name

    def bind(self, scene, robot: str) -> None:
        self.robot = self.robot or robot
        dof = scene.robot_of(self.robot).dof
        self.dim = 2 * dof if self.velocities else dof
        self.key = self._name or f"{self.robot}/joints"

    def read(self, live, robot: str) -> np.ndarray:
        q = live.joint_positions(self.robot)
        if not self.velocities:
            return np.asarray(q, dtype=np.float64)
        return np.asarray(q + live.joint_velocities(self.robot), dtype=np.float64)

    def lower(self, robot: str) -> dict:
        return {"kind": "joints", "robot": self.robot or robot, "velocities": self.velocities}


class TcpPose(Channel):
    """World pose of a robot's TCP: position + quaternion (7)."""

    dim = 7

    def __init__(self, robot: Optional[str] = None, *, name: Optional[str] = None):
        self.robot = robot
        self._name = name

    def bind(self, scene, robot: str) -> None:
        self.robot = self.robot or robot
        self.key = self._name or f"{self.robot}/tcp"

    def read(self, live, robot: str) -> np.ndarray:
        return _pose_vec(live.tcp_pose(self.robot))

    def lower(self, robot: str) -> dict:
        return {"kind": "tcp_pose", "robot": self.robot or robot}


class LinkPose(Channel):
    """World pose of a named link (7)."""

    dim = 7

    def __init__(self, link: str, robot: Optional[str] = None, *, name: Optional[str] = None):
        self.link = link
        self.robot = robot
        self._name = name

    def bind(self, scene, robot: str) -> None:
        self.robot = self.robot or robot
        self.key = self._name or f"{self.robot}/{self.link}"

    def read(self, live, robot: str) -> np.ndarray:
        return _pose_vec(live.link_pose(self.link, self.robot))

    def lower(self, robot: str) -> dict:
        return {"kind": "link_pose", "robot": self.robot or robot, "link": self.link}


class ObjectPose(Channel):
    """World pose of an obstacle (7) — a physics-dynamic part where the
    engine put it."""

    dim = 7

    def __init__(self, obstacle: str, *, name: Optional[str] = None):
        self.obstacle = obstacle
        self.key = name or f"{obstacle}/pose"

    def read(self, live, robot: str) -> np.ndarray:
        return _pose_vec(live.object_pose(self.obstacle))

    def lower(self, robot: str) -> dict:
        return {"kind": "object_pose", "obstacle": self.obstacle}


class ObjectVel(Channel):
    """World-frame linear + angular velocity of a physics-dynamic obstacle (6)."""

    dim = 6

    def __init__(self, obstacle: str, *, name: Optional[str] = None):
        self.obstacle = obstacle
        self.key = name or f"{obstacle}/vel"

    def read(self, live, robot: str) -> np.ndarray:
        v, w = live.object_velocity(self.obstacle)
        return np.array([*v, *w], dtype=np.float64)

    def lower(self, robot: str) -> dict:
        return {"kind": "object_vel", "obstacle": self.obstacle}


class Relative(Channel):
    """Pose of `b` in `a`'s frame (7): the TCP-to-part vector a reach or
    grasp policy reads. References: an obstacle name, `tcp` (the task
    robot's), `<robot>/tcp`, `<robot>/<link>`."""

    dim = 7

    def __init__(self, a: str, b: str, *, name: Optional[str] = None):
        self.a = a
        self.b = b
        self.key = name or f"{a}→{b}"

    def read(self, live, robot: str) -> np.ndarray:
        return _pose_vec(_relative(_ref_pose(live, robot, self.a), _ref_pose(live, robot, self.b)))

    def lower(self, robot: str) -> dict:
        def qualify(ref: str) -> str:
            return f"{robot}/tcp" if ref == "tcp" else ref

        return {"kind": "relative", "a": qualify(self.a), "b": qualify(self.b)}


class Contacts(Channel):
    """Whether two bodies touch after the last tick, and the tick's peak
    contact force (2). Links are named `<robot>/<link>`. Physics rollouts
    only — without an engine the channel reads zero."""

    dim = 2

    def __init__(self, a: str, b: str, *, name: Optional[str] = None):
        self.a = a
        self.b = b
        self.key = name or f"contact:{a}×{b}"

    def read(self, live, robot: str) -> np.ndarray:
        for a, b, force in live.contacts():
            if {a, b} == {self.a, self.b}:
                return np.array([1.0, force], dtype=np.float64)
        return np.zeros(2)

    def lower(self, robot: str) -> dict:
        return {"kind": "contacts", "a": self.a, "b": self.b}


class Signal(Channel):
    """Level of a signal lane — an internal signal, a sensor, a device's
    running state (1)."""

    dim = 1

    def __init__(self, signal: str, *, name: Optional[str] = None):
        self.signal = signal
        self.key = name or f"signal:{signal}"

    def read(self, live, robot: str) -> np.ndarray:
        return np.array([1.0 if live.signal(self.signal) else 0.0])

    def lower(self, robot: str) -> dict:
        return {"kind": "signal", "signal": self.signal}


class Clearance(Channel):
    """Smallest distance between any robot link and any obstacle, capped
    at `cap` metres (1). Physics-dynamic parts count as obstacles."""

    dim = 1

    def __init__(self, *, cap: float = 1.0, name: Optional[str] = None):
        self.cap = cap
        self.key = name or "clearance"

    def read(self, live, robot: str) -> np.ndarray:
        d = live.clearance()
        return np.array([self.cap if d is None else min(d, self.cap)])

    def lower(self, robot: str) -> dict:
        return {"kind": "clearance", "cap": self.cap}


class Collision(Channel):
    """Whether the driven robot stands in a collision with itself or the
    scenery after the last tick (1) — the per-tick safety read."""

    dim = 1

    def __init__(self, *, name: Optional[str] = None):
        self.key = name or "collision"

    def read(self, live, robot: str) -> np.ndarray:
        return np.array([1.0 if live.collisions() else 0.0])

    def lower(self, robot: str) -> dict:
        return {"kind": "collision"}


class Lidar(Channel):
    """A LiDAR sweep on the live world: one range per beam (meters; a
    beam with no return reads the scanner's max range), every `stride`-th
    azimuth of `rings` (all by default), with Gaussian range noise
    `noise` (1σ m) drawn from the world's seed and the tick. The width
    is the thinned beam count, ring-major, resolved against the scene.
    `points=True` reads the hit points instead, `(beams, 3)` in the
    scanner frame (zeros for no return) — the point-cloud form of the
    same sweep. Cast in Rust at ~1 µs a beam: a planar 541-beam scanner
    costs about one control step, a 16-ring scanner needs
    `rings`/`stride`."""

    def __init__(self, lidar: str, *, stride: int = 1, rings: Optional[Sequence[int]] = None, noise: float = 0.0, points: bool = False, name: Optional[str] = None):
        self.lidar = lidar
        self.stride = int(stride)
        self.rings = None if rings is None else [int(r) for r in rings]
        self.noise = float(noise)
        self.points = bool(points)
        self.key = name or f"lidar:{lidar}"

    def bind(self, scene, robot: str) -> None:
        self.dim = scene._observation_dims(json.dumps([self.lower(robot)]))[0]
        self.shape = (self.dim // 3, 3) if self.points else None

    def read(self, live, robot: str) -> np.ndarray:
        return np.asarray(live.lidar(self.lidar, stride=self.stride, rings=self.rings, noise=self.noise, points=self.points), dtype=np.float64).ravel()

    def lower(self, robot: str) -> dict:
        spec = {"kind": "lidar", "lidar": self.lidar, "stride": self.stride, "noise": self.noise, "points": self.points}
        if self.rings is not None:
            spec["rings"] = self.rings
        return spec


class _Picture(Channel):
    """Base of the camera channels: a picture `size = (w, h)` of the
    named camera, of its `geometry` (`"visual"`, the studio's picture, or
    `"collision"`, what planning and physics see), rendered by the
    rollout's rasteriser (design-rl-sensors.md RS1) — the camera's own
    pinhole and the studio's z16 depth semantics."""

    kind = ""

    def __init__(self, camera: str, size: tuple[int, int] = (64, 64), *, geometry: str = "visual", name: Optional[str] = None):
        self.camera = camera
        self.width, self.height = int(size[0]), int(size[1])
        if geometry not in ("visual", "collision"):
            raise ValueError(f"geometry must be 'visual' or 'collision', got {geometry!r}")
        self.geometry = geometry
        self.key = name or f"{camera}/{self.kind}"
        self.dim = self.width * self.height
        self.shape = (self.height, self.width)

    def _frame(self, live):
        return live.render(self.camera, self.width, self.height, geometry=self.geometry)

    def lower(self, robot: str) -> dict:
        return {"kind": self.kind, "camera": self.camera, "width": self.width, "height": self.height, "geometry": self.geometry}


class Depth(_Picture):
    """A depth picture `(h, w)`: Z along the optical axis in meters, 0 for
    no return or outside the camera's `[near, far]`, through a sensor
    model on the valid pixels, drawn from the world's seed and the tick:
    Gaussian noise `noise + noise_z2·z²` (1σ m — a stereo camera's error
    grows with the square of the range; a RealSense-class sensor is about
    `noise_z2=0.002`), disparity quantisation for a stereo `baseline` (m)
    at the picture's focal length (the depth steps a real stereo pair
    shows on far surfaces), and a `dropout` fraction of pixels reading
    no return. `noise=0` and the rest unset is the clean render."""

    kind = "depth"

    def __init__(
        self,
        camera: str,
        size: tuple[int, int] = (64, 64),
        *,
        geometry: str = "visual",
        noise: float = 0.0,
        noise_z2: float = 0.0,
        baseline: Optional[float] = None,
        dropout: float = 0.0,
        name: Optional[str] = None,
    ):
        super().__init__(camera, size, geometry=geometry, name=name)
        self.noise = float(noise)
        self.noise_z2 = float(noise_z2)
        self.baseline = None if baseline is None else float(baseline)
        self.dropout = float(dropout)

    def read(self, live, robot: str) -> np.ndarray:
        depth, _ = self._frame(live)
        return depth.astype(np.float64).ravel()

    def lower(self, robot: str) -> dict:
        spec = {**super().lower(robot), "noise": self.noise, "noise_z2": self.noise_z2, "dropout": self.dropout}
        if self.baseline is not None:
            spec["baseline"] = self.baseline
        return spec


class Segmentation(_Picture):
    """The segmentation id per pixel `(h, w)`: 0 background; `rl.
    segmentation_ids(scene)` names the rest (obstacles, then `robot/link`)."""

    kind = "segmentation"

    def read(self, live, robot: str) -> np.ndarray:
        _, ids = self._frame(live)
        return ids.astype(np.float64).ravel()


class Rgb(_Picture):
    """A flat-shaded colour picture `(h, w, 3)`, values 0..255 (black
    background): displayColor and shape under the world's lighting — one
    directional light plus ambient, Lambert, hard shadows on request
    (`Task.render["shadows"]`), no textures. Set the light per episode
    with `Task.render` (a domain-randomisation hook); the studio and the
    USD export carry the photoreal look."""

    kind = "rgb"

    def __init__(self, camera: str, size: tuple[int, int] = (64, 64), *, geometry: str = "visual", name: Optional[str] = None):
        super().__init__(camera, size, geometry=geometry, name=name)
        self.dim = 3 * self.width * self.height
        self.shape = (self.height, self.width, 3)

    def read(self, live, robot: str) -> np.ndarray:
        return live.render_rgb(self.camera, self.width, self.height, geometry=self.geometry).astype(np.float64).ravel()


class PointCloud(_Picture):
    """The camera-frame point of every pixel, `(h·w, 3)`: x right, y up,
    z toward the viewer; zeros for no return."""

    kind = "point_cloud"

    def __init__(self, camera: str, size: tuple[int, int] = (32, 32), *, geometry: str = "visual", name: Optional[str] = None):
        super().__init__(camera, size, geometry=geometry, name=name)
        self.dim = 3 * self.width * self.height
        self.shape = (self.height * self.width, 3)

    def read(self, live, robot: str) -> np.ndarray:
        depth, _ = self._frame(live)
        fx = (self.width / 2.0) / math.tan(math.radians(live_fov(live, self.camera)) / 2.0)
        cx, cy = self.width / 2.0, self.height / 2.0
        d = depth.astype(np.float64)
        v, u = np.mgrid[0:self.height, 0:self.width]
        valid = d > 0.0
        x = np.where(valid, (u + 0.5 - cx) / fx * d, 0.0)
        y = np.where(valid, (cy - (v + 0.5)) / fx * d, 0.0)
        z = np.where(valid, -d, 0.0)
        return np.stack([x, y, z], axis=-1).reshape(-1)


def live_fov(live, camera: str) -> float:
    """The horizontal field of view (degrees) of a camera of the live
    rollout's scene."""
    return float(live.camera_fov(camera))


def segmentation_ids(scene) -> dict[str, int]:
    """The segmentation id of every body a picture can show, by name
    (obstacles as is, links as `robot/link`); background is 0."""
    return dict(scene._segmentation_ids())


# ------------------------------------------------------------ controls


def _joint_indices(scene, robot: str, group: Optional[str]) -> list[int]:
    model = scene.robot_of(robot)
    if group is None:
        return list(range(model.dof))
    names = model.joint_names
    return [names.index(j) for j in model.group(group).joints]


def _limits(scene, robot: str, indices: Sequence[int]) -> list[tuple[float, float]]:
    """Position limits per driven joint; an unlimited (continuous) joint
    maps to ±π."""
    limits = scene.robot_of(robot).joint_limits
    out = []
    for qi in indices:
        lim = limits[qi]
        out.append((-math.pi, math.pi) if lim is None else (float(lim[0]), float(lim[1])))
    return out


@dataclass
class JointDelta:
    """Action = a step per driven joint, in `[-1, 1]` scaled by `max_step`
    (rad, or m for a prismatic joint), added to the current commanded
    position. The drive's own rate limit still applies underneath."""

    max_step: float = 0.05
    hz: float = 20.0
    max_velocity: Optional[float] = None

    def bind(self, scene, robot: str, group: Optional[str]) -> None:
        self.indices = _joint_indices(scene, robot, group)
        self.dim = len(self.indices)

    def lower(self) -> dict:
        """This control as the spec the rollout applies actions through."""
        return {"kind": "joint_delta", "indices": self.indices, "max_step": self.max_step}


@dataclass
class JointTarget:
    """Action = an absolute position per driven joint, `[-1, 1]` mapped
    onto the joint's position limits. The drive's rate limit turns a
    far target into a servo move."""

    hz: float = 20.0
    max_velocity: Optional[float] = None

    def bind(self, scene, robot: str, group: Optional[str]) -> None:
        self.indices = _joint_indices(scene, robot, group)
        self.limits = _limits(scene, robot, self.indices)
        self.dim = len(self.indices)

    def lower(self) -> dict:
        return {
            "kind": "joint_target",
            "indices": self.indices,
            "limits": [[lo, hi] for lo, hi in self.limits],
        }


@dataclass
class Torque:
    """Action = a raw torque per driven joint, `[-1, 1]` scaled by
    `max_torque` (N·m, or N for a prismatic joint; a scalar or one value
    per joint; default each joint's URDF effort limit). The joint's servo
    is off while the policy drives it, so gravity is the policy's to
    hold — unless `gravity_compensation`, which adds the model's gravity
    torque on top of the action every tick (a torque interface whose
    firmware compensates; the sum stays under the joint's cap). Off by
    default: raw torques are what the physics answers to. Needs a robot
    declared dynamic (`Scene.set_robot_physics`) and physics on (the
    default) — a kinematic robot has no joint torques. `Joints()` then
    reads the physical joint state."""

    max_torque: Optional[Union[float, Sequence[float]]] = None
    hz: float = 20.0
    max_velocity: Optional[float] = None
    gravity_compensation: bool = False

    def bind(self, scene, robot: str, group: Optional[str]) -> None:
        if not scene.robot_physics(robot):
            raise ValueError(f"Torque control needs robot {robot!r} declared dynamic: scene.set_robot_physics({robot!r})")
        self.indices = _joint_indices(scene, robot, group)
        self.dim = len(self.indices)
        if self.max_torque is None:
            self.caps = None
        elif isinstance(self.max_torque, (int, float)):
            self.caps = [float(self.max_torque)] * self.dim
        else:
            self.caps = [float(v) for v in self.max_torque]
            if len(self.caps) != self.dim:
                raise ValueError(f"Torque.max_torque has {len(self.caps)} values for {self.dim} joints")

    def lower(self) -> dict:
        spec = {"kind": "torque", "indices": self.indices, "gravity_compensation": bool(self.gravity_compensation)}
        if self.caps is not None:
            spec["max_torque"] = self.caps
        return spec


@dataclass
class TcpDelta:
    """Action = a Cartesian step of the TCP: 3 translations in `[-1, 1]`
    scaled by `max_step_m`, plus 3 rotations scaled by `max_step_rad`
    when `rotate` (else the orientation is held), plus one gripper value
    per `gripper` joint (`[-1, 1]` onto its limits; coupled fingers
    follow). `frame` says which axes the step is taken along: `"world"`,
    or `"tcp"` — the tool's own, the frame a `Relative("tcp", …)`
    observation is expressed in, so a policy reading that channel acts
    in the coordinates it sees. The steps integrate a Cartesian setpoint
    (the way a servo controller streams one), so a drive that lags its
    rate limit catches up instead of letting the held orientation drift;
    each setpoint is solved to joints from the current configuration
    with no restarts, so the arm stays on its solution branch. A step the
    IK cannot solve — the workspace boundary, typically — is dropped: the
    setpoint stays, the joints hold, and `info["ik_failed"]` says so."""

    max_step_m: float = 0.01
    max_step_rad: float = 0.05
    rotate: bool = False
    gripper: Sequence[str] = ()
    frame: str = "world"
    hz: float = 20.0
    max_velocity: Optional[float] = None
    max_iters: int = 100

    def bind(self, scene, robot: str, group: Optional[str]) -> None:
        if self.frame not in ("world", "tcp"):
            raise ValueError(f"TcpDelta.frame must be 'world' or 'tcp', got {self.frame!r}")
        self.model = scene.robot_of(robot)
        self.group = group
        names = self.model.joint_names
        self.gripper_indices = [names.index(j) for j in self.gripper]
        self.gripper_limits = _limits(scene, robot, self.gripper_indices)
        self.dim = 3 + (3 if self.rotate else 0) + len(self.gripper_indices)

    def lower(self) -> dict:
        """The setpoint integration and the IK run in the rollout (Rust),
        per world and inside its parallel step; Python only names the
        numbers."""
        return {
            "kind": "tcp_delta",
            "max_step_m": self.max_step_m,
            "max_step_rad": self.max_step_rad,
            "rotate": self.rotate,
            "frame": self.frame,
            "gripper": [[qi, lo, hi] for qi, (lo, hi) in zip(self.gripper_indices, self.gripper_limits)],
            "max_iters": self.max_iters,
        }


# ------------------------------------------------------------ task + env


@dataclass
class Task:
    """What a policy does in the cell. `reset(scene, rng)` randomises the
    built scene with the seeded generator before each episode (set poses
    and properties — do not add residents, the scene persists across
    episodes); `reward(obs, info)` and `done(obs, info)` read the channel
    dict and the step's info. An episode is truncated `horizon_s` after
    its first observation or when every sequence has run to its end.
    `settle_s` is ticked off before that first observation, with no
    action: what the randomisation dropped comes to rest (a part set
    down on a table, a pool tipped into a bin). `render` lights the
    colour pictures (see the field). The `control` is `JointDelta`,
    `JointTarget` or `TcpDelta` (position commands the drive rate-limits)
    or `Torque` (raw joint torques on a robot declared dynamic).
    `physics` is the bake's argument: `True` (the declared scope), an
    engine name, or a `bt.Physics(...)` — the world scope, a ground
    plane, the cell powered (`powered=True`: a machine no program drives
    is otherwise unpowered under the world scope until the task's drive
    takes it)."""

    robot: Optional[str] = None
    control: Any = field(default_factory=JointDelta)
    observe: Sequence[Channel] = ()
    reset: Optional[Callable[[Any, np.random.Generator], None]] = None
    reward: Optional[Callable[[ObsDict, Info], float]] = None
    done: Optional[Callable[[ObsDict, Info], bool]] = None
    horizon_s: float = 10.0
    settle_s: float = 0.0
    group: Optional[str] = None
    sequences: Optional[Sequence[str]] = None
    physics: Union[bool, str, Any] = True
    #: How the pictures are drawn, per episode: a dict with any of
    #: `"light": (x, y, z)` (a direction toward the light), `"ambient": a`,
    #: `"shadows": bool` (hard shadows from that light, about the cost of
    #: a second picture) and `"decimate": cell_m` (visual meshes cut by
    #: vertex clustering to that cell, for dense CAD in a 64-pixel
    #: picture) — or a function of the world's generator returning one,
    #: the domain-randomisation hook. `None` keeps the defaults.
    render: Optional[Union[dict, Callable[[np.random.Generator], dict]]] = None


_RENDER_KEYS = ("light", "ambient", "shadows", "decimate")


def _render_options(task: Task, rng: np.random.Generator) -> Optional[dict]:
    """The normalised picture options a task asks for this episode
    (`light`, `ambient`, `shadows`, `decimate`), or `None`."""
    spec = task.render(rng) if callable(task.render) else task.render
    if spec is None:
        return None
    unknown = sorted(set(spec) - set(_RENDER_KEYS))
    if unknown:
        raise ValueError(f"render: unknown keys {unknown}; expected any of {list(_RENDER_KEYS)}")
    light = tuple(float(v) for v in spec.get("light", (0.3, -0.5, 0.8)))
    if len(light) != 3:
        raise ValueError("render['light'] must be a direction (x, y, z)")
    decimate = spec.get("decimate", "auto")
    if decimate is not None and decimate != "auto" and not (float(decimate) > 0.0):
        raise ValueError("render['decimate'] is a cell size in metres (positive), None for full detail, or absent for the default")
    return {
        "light": light,
        "ambient": float(spec.get("ambient", 0.35)),
        "shadows": bool(spec.get("shadows", False)),
        # Absent: the default cell (`_auto_decimate`); `None`: full detail.
        "decimate": decimate if decimate in (None, "auto") else float(decimate),
    }


def _auto_decimate(channels: Sequence[Channel], fov_of: Callable[[str], float]) -> Optional[float]:
    """The default decimation cell of a task's pictures: half a pixel's
    footprint at one metre of the finest picture asked for (`tan(fov/2)
    / width`) — a catalogue robot's CAD-dense visual meshes cost a
    64-pixel picture ten times its state observation otherwise
    (design-rl-tabletop.md G7), and nothing a pixel could show past half
    a metre is lost. `None` when the task takes no picture."""
    cells = [
        math.tan(math.radians(fov_of(c.camera)) / 2.0) / c.width
        for c in channels
        if isinstance(c, _Picture)
    ]
    return min(cells) if cells else None


def _decimate_cell(render: Optional[dict], auto: Optional[float]) -> Optional[float]:
    """The cell an episode's pictures are decimated to: what `render`
    says, or the task's default when it says nothing."""
    if render is None or render["decimate"] == "auto":
        return auto
    return render["decimate"]


def _settle_ticks(settle_s: float, dt: float) -> int:
    """Scan ticks of a task's settling time."""
    if not (settle_s >= 0.0) or not math.isfinite(settle_s):
        raise ValueError(f"settle_s must be a finite, non-negative time, got {settle_s}")
    return round(settle_s / dt)


def _offsets(channels: Sequence[Channel]) -> list[tuple[str, int, int, Optional[tuple[int, ...]]]]:
    """Each channel's `(key, start, end, shape)` in the flat row."""
    out = []
    at = 0
    for c in channels:
        out.append((c.key, at, at + c.dim, c.shape))
        at += c.dim
    return out


def _shaped(block: np.ndarray, shape: Optional[tuple[int, ...]]) -> np.ndarray:
    return block.reshape(shape) if shape else block


def _bind(scene, task: Task, dt: float) -> tuple[int, list[Channel]]:
    """Resolves a task against a built scene: the driven robot, the
    control (and its scans per step), the channels. Returns
    `(scans_per_step, channels)`."""
    robots = scene.robots
    if task.robot is None:
        if len(robots) != 1:
            raise ValueError(f"Task.robot must name one of {robots}")
        task.robot = robots[0]
    elif task.robot not in robots:
        raise ValueError(f"unknown robot {task.robot!r} (have {robots})")
    control = task.control
    control.bind(scene, task.robot, task.group)
    k = 1.0 / (control.hz * dt)
    scans = max(1, int(round(k)))
    if abs(k - scans) > 1e-6:
        raise ValueError(f"control hz {control.hz} is not a whole number of {dt}s scans")
    for channel in task.observe:
        channel.bind(scene, task.robot)
    keys = [c.key for c in task.observe]
    if len(set(keys)) != len(keys):
        raise ValueError(f"observation keys collide: {keys}")
    channels = list(task.observe)
    if channels:
        dims = scene._observation_dims(json.dumps([c.lower(task.robot) for c in channels]))
        for channel, dim in zip(channels, dims):
            channel.dim = dim
    return scans, channels


class _Box:
    """A `gymnasium.spaces.Box` stand-in for installs without gymnasium."""

    def __init__(self, low, high, shape, dtype=np.float32):
        self.shape = tuple(shape)
        self.dtype = dtype
        self.low = np.full(self.shape, low, dtype=dtype)
        self.high = np.full(self.shape, high, dtype=dtype)

    def sample(self, rng: Optional[np.random.Generator] = None) -> np.ndarray:
        rng = rng or np.random.default_rng()
        lo = np.where(np.isfinite(self.low), self.low, -1.0)
        hi = np.where(np.isfinite(self.high), self.high, 1.0)
        return rng.uniform(lo, hi).astype(self.dtype)

    def contains(self, x) -> bool:
        x = np.asarray(x)
        return x.shape == self.shape and bool(np.all(x >= self.low) and np.all(x <= self.high))

    def __repr__(self) -> str:
        return f"Box{self.shape}"


def _box(low, high, shape):
    if _spaces is not None:
        return _spaces.Box(low=low, high=high, shape=tuple(shape), dtype=np.float32)
    return _Box(low, high, shape)


_Base = _gym.Env if _gym is not None else object


class Env(_Base):
    """One cell, one policy: the rollout of `build()`'s scene opened tick
    by tick, its task robot driven by the action every `1/hz` seconds.
    A `gymnasium.Env` when gymnasium is installed (spaces, `reset(seed=)`,
    the 5-tuple `step`), the same contract on a shim otherwise.

    `build()` runs once; `Task.reset` re-randomises that scene before each
    episode (pass `rebuild=True` to build afresh each time). Observations
    are one flat `float32` vector in channel order (`flatten=False` for a
    dict, one array per channel); the channel dict is what `reward` and
    `done` see either way, and rides `info["channels"]`."""

    metadata = {"render_modes": []}

    def __init__(
        self,
        build: Callable[[], Any],
        task: Task,
        *,
        seed: int = 0,
        dt: float = 0.01,
        flatten: bool = True,
        rebuild: bool = False,
        max_duration: Optional[float] = None,
    ):
        self._build = build
        self.task = task
        self._dt = dt
        self._flatten = flatten
        self._rebuild = rebuild
        self._max_duration = max_duration
        self._seed = int(seed)
        self._rng = np.random.default_rng(seed)
        self._scene = build()
        self._bind()
        self.live = None
        self._done = True
        self._timeline = None
        self._steps = 0
        self._t0 = 0.0

    # -------------------------------------------------- setup

    def _bind(self) -> None:
        self._k, self.channels = _bind(self._scene, self.task, self._dt)
        control = self.task.control
        self.obs_dim = sum(c.dim for c in self.channels)
        self._offsets = _offsets(self.channels)
        self._spec = json.dumps([c.lower(self.task.robot) for c in self.channels])
        self._control = json.dumps(control.lower())
        self._auto_cell = _auto_decimate(self.channels, self._scene.camera_fov)
        self.action_space = _box(-1.0, 1.0, (control.dim,))
        if self._flatten:
            self.observation_space = _box(-np.inf, np.inf, (self.obs_dim,))
        elif _spaces is not None:
            self.observation_space = _spaces.Dict(
                {c.key: _box(-np.inf, np.inf, c.shape or (c.dim,)) for c in self.channels}
            )
        else:
            self.observation_space = {c.key: _box(-np.inf, np.inf, c.shape or (c.dim,)) for c in self.channels}

    @property
    def scene(self):
        """The built (and re-randomised) scene episodes are opened from."""
        return self._scene

    @property
    def scans_per_step(self) -> int:
        """Scan ticks per control step (`1 / (hz · dt)`)."""
        return self._k

    # -------------------------------------------------- episode

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        if _gym is not None:
            super().reset(seed=seed)
        if seed is not None:
            self._seed = int(seed)
            self._rng = np.random.default_rng(seed)
        if self._rebuild:
            self._scene = self._build()
            self._bind()
        if self.task.reset is not None:
            self.task.reset(self._scene, self._rng)
        names = list(self.task.sequences or self._scene.sequence_names)
        if not names:
            raise ValueError("the scene has no sequence to run (a Task needs one program, if only a wait)")
        self.live = self._scene.open_rollout(
            names,
            dt=self._dt,
            max_duration=self._max_duration or self.task.horizon_s + self.task.settle_s + 1.0,
            physics=self.task.physics,
        )
        self.live.drive(
            robot=self.task.robot,
            group=self.task.group,
            max_velocity=getattr(self.task.control, "max_velocity", None),
        )
        dims = self.live.set_observation(self._spec)
        assert dims == [c.dim for c in self.channels], (dims, self.channels)
        self.live.set_noise_seed(self._seed)
        render = _render_options(self.task, self._rng)
        if render is not None:
            self.live.set_lighting(render["light"], render["ambient"], render["shadows"])
        self.live.set_render_decimate(_decimate_cell(render, self._auto_cell))
        self.live.set_control(self._control, robot=self.task.robot, group=self.task.group)
        # The settling ticks: the world runs with the drive holding the
        # robot where it stands, and what the reset dropped comes to rest.
        settle = _settle_ticks(self.task.settle_s, self._dt)
        if settle:
            self.live.tick(settle)
        self._t0 = self.live.t
        self._steps = 0
        self._done = False
        self._timeline = None
        channels = self._observe()
        return self._pack(channels), self._info(channels, {})

    def step(self, action):
        if self.live is None or self._done:
            raise RuntimeError("the episode is over: call reset()")
        a = np.asarray(action, dtype=np.float64).reshape(-1)
        if a.shape[0] != self.task.control.dim:
            raise ValueError(f"action has {a.shape[0]} values, the control takes {self.task.control.dim}")
        a = np.clip(a, -1.0, 1.0)
        extra: Info = {"ik_failed": not self.live.act(a.tolist(), robot=self.task.robot)}
        error = None
        try:
            self.live.tick(self._k)
        except ValueError as e:
            # The bake's own failure — a timeout, a robot-vs-robot
            # collision — ends the episode as a terminal fault.
            error = str(e)
        self._steps += 1
        channels = self._observe()
        info = self._info(channels, extra)
        if error is not None:
            info["error"] = error
        reward = float(self.task.reward(channels, info)) if self.task.reward is not None else 0.0
        terminated = bool(self.task.done(channels, info)) if self.task.done is not None else False
        terminated = terminated or error is not None
        truncated = (not terminated) and (
            self.live.t - self._t0 >= self.task.horizon_s - 1e-9 or self.live.finished
        )
        self._done = terminated or truncated
        return self._pack(channels), reward, terminated, truncated, info

    def timeline(self, *, publish: bool = False):
        """Closes the current episode into its `SequenceTimeline` (the
        batch bake's output: robot tracks, object motion, contacts,
        signals) — USD, studio playback and the timeline assertions all
        read it. After this the episode is over; `reset()` starts the
        next. Returns the same timeline until then."""
        if self._timeline is None:
            if self.live is None:
                raise RuntimeError("no episode to close: call reset() first")
            self._timeline = self.live.finish(publish=publish)
            self.live = None
            self._done = True
        return self._timeline

    def close(self) -> None:
        self.live = None
        self._done = True

    # -------------------------------------------------- readers

    def _observe(self) -> ObsDict:
        """The channels as the world stands, packed in Rust (the same
        packing a `VecEnv` does per world) and sliced per channel — a
        picture channel reshaped to its `(h, w)`."""
        row = self.live.observe()
        return {key: _shaped(row[a:b], shape) for key, a, b, shape in self._offsets}

    def _pack(self, channels: ObsDict):
        if not self._flatten:
            return {k: v.astype(np.float32) for k, v in channels.items()}
        if not self.channels:
            return np.zeros(0, dtype=np.float32)
        return np.concatenate([channels[c.key].reshape(-1) for c in self.channels]).astype(np.float32)

    def _info(self, channels: ObsDict, extra: Info) -> Info:
        live = self.live
        collisions = live.collisions()
        info: Info = {
            "t": live.t,
            "steps": self._steps,
            "collision": bool(collisions),
            "collisions": collisions,
            "contacts": live.contacts(),
            "finished": live.finished,
            "channels": channels,
        }
        info.update(extra)
        return info


_VecBase = _vector.VectorEnv if _vector is not None else object


class VecEnv(_VecBase):
    """N copies of one cell stepped together — `make(..., num_envs=N)`.
    Each world is its own scene (`build()` runs N times, once), its own
    seeded generator (`seed + i`) and its own live rollout; the worlds
    advance in parallel on Rust threads with the GIL released, and the
    observations come back as one `(N, dim)` array packed in Rust.

    A `gymnasium.vector.VectorEnv` when gymnasium is installed, with the
    `NEXT_STEP` autoreset convention: a world that ends at one step is
    reset at the next, where its action is ignored and its first
    observation comes back with zero reward and no termination flags.
    The same contract runs without gymnasium."""

    metadata = {"render_modes": []}
    if _vector is not None:
        metadata["autoreset_mode"] = _vector.AutoresetMode.NEXT_STEP

    def __init__(
        self,
        build: Callable[[], Any],
        task: Task,
        num_envs: int,
        *,
        seed: int = 0,
        dt: float = 0.01,
        max_duration: Optional[float] = None,
        flatten: bool = True,
    ):
        if num_envs < 1:
            raise ValueError("num_envs must be at least 1")
        self.num_envs = num_envs
        self.task = task
        self._dt = dt
        self._max_duration = max_duration
        self._flatten = flatten
        self._seed = int(seed)
        self._scenes = [build() for _ in range(num_envs)]
        self._k, self.channels = _bind(self._scenes[0], task, dt)
        self.obs_dim = sum(c.dim for c in self.channels)
        self._offsets = _offsets(self.channels)
        self._spec = json.dumps([c.lower(task.robot) for c in self.channels])
        self._control = json.dumps(task.control.lower())
        self._auto_cell = _auto_decimate(self.channels, self._scenes[0].camera_fov)
        self.single_action_space = _box(-1.0, 1.0, (task.control.dim,))
        if flatten:
            self.single_observation_space = _box(-np.inf, np.inf, (self.obs_dim,))
        elif _spaces is not None:
            self.single_observation_space = _spaces.Dict(
                {c.key: _box(-np.inf, np.inf, c.shape or (c.dim,)) for c in self.channels}
            )
        else:
            self.single_observation_space = {c.key: _box(-np.inf, np.inf, c.shape or (c.dim,)) for c in self.channels}
        if _vector is not None:
            self.action_space = _vector.utils.batch_space(self.single_action_space, num_envs)
            self.observation_space = _vector.utils.batch_space(self.single_observation_space, num_envs)
        elif flatten:
            self.action_space = _box(-1.0, 1.0, (num_envs, task.control.dim))
            self.observation_space = _box(-np.inf, np.inf, (num_envs, self.obs_dim))
        else:
            self.action_space = _box(-1.0, 1.0, (num_envs, task.control.dim))
            self.observation_space = {
                c.key: _box(-np.inf, np.inf, (num_envs, *(c.shape or (c.dim,)))) for c in self.channels
            }
        self._rngs = [np.random.default_rng(seed + i) for i in range(num_envs)]
        self._vec = None
        self._needs_reset = np.ones(num_envs, dtype=bool)
        self._steps = np.zeros(num_envs, dtype=np.int64)

    @property
    def scenes(self) -> list:
        """The N built (and re-randomised) scenes, one per world."""
        return self._scenes

    @property
    def scans_per_step(self) -> int:
        return self._k

    # -------------------------------------------------- world readers

    def joints(self) -> np.ndarray:
        """The driven robot's joints per world, `(N, dof)`."""
        return self._vec.joints_all()

    def tcps(self) -> np.ndarray:
        """The driven robot's TCP pose per world, `(N, 7)`."""
        return self._vec.tcp_all()

    def _channels_of(self, row: np.ndarray) -> ObsDict:
        return {key: _shaped(row[a:b], shape) for key, a, b, shape in self._offsets}

    def _pack(self, obs: np.ndarray):
        """The `(N, dim)` block as the observation: flat float32, or one
        `(N, ...)` float32 array per channel (pictures `(N, h, w)`)."""
        if self._flatten:
            return obs.astype(np.float32)
        return {
            key: obs[:, a:b].astype(np.float32).reshape((self.num_envs, *shape) if shape else (self.num_envs, b - a))
            for key, a, b, shape in self._offsets
        }

    # -------------------------------------------------- episodes

    def _open(self, indices: Sequence[int]) -> None:
        """Randomises and (re)opens the worlds at `indices`."""
        from . import _core

        for i in indices:
            if self.task.reset is not None:
                self.task.reset(self._scenes[i], self._rngs[i])
        names = list(self.task.sequences or self._scenes[0].sequence_names)
        if not names:
            raise ValueError("the scene has no sequence to run (a Task needs one program, if only a wait)")
        scenes = [self._scenes[i] for i in indices]
        if self._vec is None:
            self._vec = _core.VecRollout(
                scenes,
                names,
                self._spec,
                robot=self.task.robot,
                group=self.task.group,
                max_velocity=getattr(self.task.control, "max_velocity", None),
                dt=self._dt,
                max_duration=self._max_duration or self.task.horizon_s + self.task.settle_s + 1.0,
                physics=self.task.physics,
                control=self._control,
                seed=self._seed,
            )
        else:
            self._vec.reopen(list(indices), scenes)
        for i in indices:
            render = _render_options(self.task, self._rngs[i])
            if render is not None:
                self._vec.set_lighting(i, render["light"], render["ambient"], render["shadows"])
            self._vec.set_render_decimate(i, _decimate_cell(render, self._auto_cell))
            self._steps[i] = 0
            self._needs_reset[i] = False
        settle = _settle_ticks(self.task.settle_s, self._dt)
        if settle:
            only = [i in set(indices) for i in range(self.num_envs)]
            errors = self._vec.tick_all(settle, only)
            failed = [(i, e) for i, e in enumerate(errors) if e is not None and only[i]]
            if failed:
                raise RuntimeError(f"the settling ticks failed: {failed}")

    def reset(self, *, seed: Optional[Union[int, Sequence[int]]] = None, options: Optional[dict] = None):
        if seed is not None:
            seeds = [seed + i for i in range(self.num_envs)] if isinstance(seed, int) else list(seed)
            if len(seeds) != self.num_envs:
                raise ValueError(f"{len(seeds)} seeds for {self.num_envs} worlds")
            self._rngs = [np.random.default_rng(s) for s in seeds]
            self._seed = int(seeds[0])
        self._vec = None
        self._open(range(self.num_envs))
        obs = self._vec.observe_all()
        info = self._info(obs, None, np.ones(self.num_envs, dtype=bool), {})
        return self._pack(obs), info

    def step(self, actions):
        if self._vec is None:
            raise RuntimeError("call reset() first")
        a = np.asarray(actions, dtype=np.float64)
        if a.shape != (self.num_envs, self.task.control.dim):
            raise ValueError(f"actions must be ({self.num_envs}, {self.task.control.dim}), got {a.shape}")
        a = np.clip(a, -1.0, 1.0)
        resetting = self._needs_reset.copy()
        fresh = np.flatnonzero(resetting)
        if len(fresh):
            self._open(fresh.tolist())
        obs, results = self._vec.step_actions(self._k, np.ascontiguousarray(a.ravel()), resetting.tolist())
        self._steps[~resetting] += 1
        extra: Info = {"ik_failed": np.array([r[5] for r in results], dtype=bool)}
        info = self._info(obs, results, resetting, extra)
        rewards = np.zeros(self.num_envs)
        terminated = np.zeros(self.num_envs, dtype=bool)
        truncated = np.zeros(self.num_envs, dtype=bool)
        for i, (t, finished, error, _, _, _) in enumerate(results):
            if resetting[i]:
                continue
            channels = info["envs"][i]["channels"]
            env_info = info["envs"][i]
            if self.task.reward is not None:
                rewards[i] = float(self.task.reward(channels, env_info))
            done = bool(self.task.done(channels, env_info)) if self.task.done is not None else False
            terminated[i] = done or error is not None
            truncated[i] = (not terminated[i]) and (
                t - self.task.settle_s >= self.task.horizon_s - 1e-9 or finished
            )
        self._needs_reset = terminated | truncated
        return self._pack(obs), rewards, terminated, truncated, info

    def _info(self, obs, results, reset: np.ndarray, extra: Info) -> Info:
        envs: list[Info] = []
        for i in range(self.num_envs):
            if results is None:
                # Fresh worlds: the clock reads the settling ticks.
                t, finished, error, collisions, contacts = float(self.task.settle_s), False, None, [], []
            else:
                t, finished, error, collisions, contacts, _ik_failed = results[i]
            env_info: Info = {
                "t": t,
                "steps": int(self._steps[i]),
                "collision": bool(collisions),
                "collisions": collisions,
                "contacts": contacts,
                "finished": finished,
                "reset": bool(reset[i]),
                "channels": self._channels_of(obs[i]),
            }
            if error is not None:
                env_info["error"] = error
            for key, value in extra.items():
                env_info[key] = value[i] if isinstance(value, np.ndarray) else value
            envs.append(env_info)
        info: Info = {
            "t": np.array([e["t"] for e in envs]),
            "steps": self._steps.copy(),
            "collision": np.array([e["collision"] for e in envs]),
            "reset": reset.copy(),
            "envs": envs,
        }
        info.update(extra)
        return info

    def timeline(self, index: int, *, publish: bool = False):
        """Closes world `index`'s episode into its `SequenceTimeline`; the
        world resets at the next step."""
        if self._vec is None:
            raise RuntimeError("call reset() first")
        live = self._vec.take(index)
        self._needs_reset[index] = True
        return live.finish(publish=publish)

    def close(self) -> None:
        self._vec = None


def make(build: Callable[[], Any], task: Task, *, seed: int = 0, num_envs: int = 1, **options):
    """Builds the environment of `build()`'s cell under `task`: an `Env`
    for one world, a `VecEnv` for `num_envs > 1`. Keyword options are the
    class's (`dt`, `max_duration`, `flatten`; `Env` also takes `rebuild`)."""
    if num_envs == 1:
        return Env(build, task, seed=seed, **options)
    return VecEnv(build, task, num_envs, seed=seed, **options)


# ------------------------------------------------------------ policies


class Policy:
    """A controller a cell's `Policy` step hands its robot to
    (`bt.seq.policy(name, ...)` + `simulate_sequence(policies={name: policy})`).

    `model(obs) -> action` is the decision — a trained network, an ONNX
    session, a scripted rule — over the flat float32 observation the
    task's channels pack; the task's control spec turns the action into
    the joint target the rollout drives toward, and `task.done` (when
    given) ends the step. Bound to `scene` at construction: the channels
    resolve against it, and a `TcpDelta` solves its IK on its robot.

        policy = rl.Policy(scene, task, model=lambda obs: net(obs))
        tl = scene.simulate_sequence("cycle", physics=True, policies={"pick": policy})

    The rollout calls the policy every `1/hz` seconds of its step with a
    dict — `t`, `step`, `obs`, `q`, `tcp`, `collisions`, `contacts` — and
    takes back the action (applied through the control spec the policy
    carries as `control`, IK included, on the Rust side), or `None` for
    done."""

    def __init__(self, scene, task: Task, model: Callable[[np.ndarray], Any], *, dt: float = 0.01):
        self.task = task
        self.model = model
        self._k, self.channels_list = _bind(scene, task, dt)
        self._offsets = _offsets(self.channels_list)
        self.channels = json.dumps([c.lower(task.robot) for c in self.channels_list])
        self.control = json.dumps(task.control.lower())

    @property
    def robot(self) -> str:
        return self.task.robot

    def __call__(self, inp: dict) -> Optional[list[float]]:
        obs = np.asarray(inp["obs"], dtype=np.float64)
        channels = {key: _shaped(obs[a:b], shape) for key, a, b, shape in self._offsets}
        info: Info = {
            "t": inp["t"],
            "steps": inp["step"],
            "collision": bool(inp["collisions"]),
            "collisions": inp["collisions"],
            "contacts": inp["contacts"],
            "channels": channels,
        }
        if self.task.done is not None and self.task.done(channels, info):
            return None
        action = np.clip(
            np.asarray(self.model(obs.astype(np.float32)), dtype=np.float64).reshape(-1), -1.0, 1.0
        )
        if action.shape[0] != self.task.control.dim:
            raise ValueError(
                f"the model returned {action.shape[0]} values, the control takes {self.task.control.dim}"
            )
        # The rollout turns the action into the joint target through the
        # control spec (`self.control`), IK included.
        return action.tolist()


def load(path, scene, task: Task, **options) -> Policy:
    """A `Policy` from a saved model: `.onnx` (through onnxruntime, first
    input, first output), or a stable-baselines3 `.zip` (`predict`,
    deterministic). A callable is wrapped as is."""
    if callable(path):
        return Policy(scene, task, path, **options)
    name = str(path)
    if name.endswith(".onnx"):
        try:
            import onnxruntime as ort
        except ImportError as e:
            raise ImportError("loading an .onnx policy needs onnxruntime") from e
        session = ort.InferenceSession(name, providers=["CPUExecutionProvider"])
        input_name = session.get_inputs()[0].name

        def model(obs: np.ndarray):
            return session.run(None, {input_name: obs[None, :].astype(np.float32)})[0][0]

        return Policy(scene, task, model, **options)
    if name.endswith(".zip"):
        try:
            import stable_baselines3 as sb3
        except ImportError as e:
            raise ImportError("loading a stable-baselines3 policy needs stable-baselines3") from e
        algo = None
        for cls_name in ("PPO", "SAC", "TD3", "A2C", "DDPG"):
            try:
                algo = getattr(sb3, cls_name).load(name, device="cpu")
                break
            except Exception:
                continue
        if algo is None:
            raise ValueError(f"could not load {name} with any stable-baselines3 algorithm")
        return Policy(scene, task, lambda obs: algo.predict(obs, deterministic=True)[0], **options)
    raise ValueError(f"unknown policy file {name!r}: pass an .onnx, a stable-baselines3 .zip, or a callable")


# ------------------------------------------------------------ cells, tiled
# A cell written once as `cell(scene, prefix, origin) -> robot name`
# (every resident named `prefix + ...`, everything placed relative to
# `origin`) is one environment for learning (`single`) and a grid of
# copies in one scene for watching or exporting (`tile`, `play`).

Cell = Callable[[Any, str, tuple[float, float]], str]


def _floor(scene, size: tuple[float, float], center: tuple[float, float], thickness: float = 0.1) -> None:
    """A slab whose top face is z = 0 — what the cells stand on."""
    scene.add_box(
        "floor",
        size=(size[0], size[1], thickness),
        position=(center[0], center[1], -thickness / 2.0),
        color=(0.82, 0.82, 0.82),
    )


def single(cell: Cell, *, sequence: str = "run", wait_s: float = 30.0, floor: Optional[tuple[float, float]] = (6.0, 6.0)):
    """`build()` for `make`: one copy of `cell` at the origin with no
    prefix, on a floor slab (`floor` is its size; `None` for none), and
    one program — a `wait_s` wait named `sequence` — for the rollout to
    run. The cell's own program, if it adds one, runs alongside."""

    def build():
        import botrail as bt

        scene = bt.Scene()
        if floor is not None:
            _floor(scene, floor, (0.0, 0.0))
        cell(scene, "", (0.0, 0.0))
        if sequence not in scene.sequence_names:
            scene.sequence(sequence).step("wait", transition=bt.seq.elapsed(wait_s))
        return scene

    return build


@dataclass
class Tiled:
    """A grid of copies of one cell in one scene (`tile`): `cells` are the
    `(prefix, robot)` of each copy, row-major, `origins` where they
    stand; `sequence` the wait program the whole grid runs under."""

    scene: Any
    cell: Cell
    cells: list[tuple[str, str]]
    origins: list[tuple[float, float]]
    sequence: str

    @property
    def robots(self) -> list[str]:
        return [robot for _, robot in self.cells]

    def randomize(
        self,
        reset: Callable[[Any, np.random.Generator, str, tuple[float, float]], None],
        *,
        seed: int = 0,
    ) -> None:
        """Runs a cell-relative randomiser `reset(scene, rng, prefix,
        origin)` on every copy, world `i` drawing from `seed + i` — the
        same draw a `VecEnv` of the single cell makes for its world `i`
        (whose `Task.reset` is the same function at prefix `""` and the
        origin)."""
        for i, ((prefix, _), origin) in enumerate(zip(self.cells, self.origins)):
            reset(self.scene, np.random.default_rng(seed + i), prefix, origin)


def tile(
    cell: Cell,
    rows: int,
    cols: int,
    spacing: tuple[float, float] = (2.0, 2.0),
    *,
    sequence: str = "run",
    wait_s: float = 30.0,
    margin: float = 1.0,
) -> Tiled:
    """`rows × cols` copies of `cell` in one scene, `spacing` apart, on
    one floor slab reaching `margin` past the grid — the picture of an
    environment being learned, and the scene `play` runs a policy on in
    every copy at once. Copy `i` (row-major) is named `c{i}/` and its
    robot whatever `cell` returned for it. One physical world, one
    thread: for learning, `make(single(cell), ..., num_envs=N)` steps N
    independent worlds in parallel instead."""
    import botrail as bt

    if rows < 1 or cols < 1:
        raise ValueError("tile needs at least one row and one column")
    sx, sy = float(spacing[0]), float(spacing[1])
    scene = bt.Scene()
    _floor(
        scene,
        ((cols - 1) * sx + 2 * margin, (rows - 1) * sy + 2 * margin),
        ((cols - 1) * sx / 2.0, (rows - 1) * sy / 2.0),
    )
    cells: list[tuple[str, str]] = []
    origins: list[tuple[float, float]] = []
    for r in range(rows):
        for c in range(cols):
            i = r * cols + c
            prefix = f"c{i}/"
            origin = (c * sx, r * sy)
            robot = cell(scene, prefix, origin)
            if not isinstance(robot, str) or robot not in scene.robots:
                raise ValueError(f"cell {prefix!r} must return the name of the robot it added, got {robot!r}")
            cells.append((prefix, robot))
            origins.append(origin)
    if sequence not in scene.sequence_names:
        scene.sequence(sequence).step("wait", transition=bt.seq.elapsed(wait_s))
    return Tiled(scene=scene, cell=cell, cells=cells, origins=origins, sequence=sequence)


def _relocate(spec: dict, prefix: str, robot_from: str, robot_to: str) -> dict:
    """A channel's lowered spec moved into copy `prefix` of the cell:
    obstacle, camera, LiDAR and signal names take the prefix, references
    to the task robot (`robot`, `<robot>/tcp`, `<robot>/<link>`) go to
    the copy's robot."""

    def ref(value: str) -> str:
        if value == robot_from or value.startswith(robot_from + "/"):
            return robot_to + value[len(robot_from):]
        return prefix + value

    out = dict(spec)
    if out.get("robot") == robot_from:
        out["robot"] = robot_to
    for key in ("obstacle", "camera", "lidar", "signal"):
        if key in out:
            out[key] = prefix + out[key]
    for key in ("a", "b"):
        if key in out:
            out[key] = ref(out[key])
    return out


def play(
    tiled: Tiled,
    task: Task,
    model: Any,
    *,
    duration_s: float = 6.0,
    dt: float = 0.01,
    publish: bool = False,
):
    """Runs one policy in every copy of a tiled cell at once and closes
    the run into a timeline — the learned behaviour as a picture (the
    studio, USD, a recording). `model(obs) -> action` is the decision over
    the flat float32 observation `task` packs (a trained network, an
    `rl.Policy`'s model, a scripted rule); the task's control turns each
    copy's action into its robot's command, the task's `done` retires a
    copy (it holds where it is), and `settle_s` is ticked off first.
    Stops at `duration_s` after the first observation, when every copy is
    done, or when the grid's programs end."""
    import botrail as bt

    if isinstance(model, Policy):
        model = model.model
    single_scene = single(tiled.cell)()
    k, channels = _bind(single_scene, task, dt)
    offsets = _offsets(channels)
    width = sum(c.dim for c in channels)
    control = json.dumps(task.control.lower())
    robot_from = task.robot
    specs = [
        _relocate(c.lower(robot_from), prefix, robot_from, robot)
        for prefix, robot in tiled.cells
        for c in channels
    ]
    settle = _settle_ticks(task.settle_s, dt)
    live = tiled.scene.open_rollout(
        [tiled.sequence],
        dt=dt,
        max_duration=duration_s + task.settle_s + 1.0,
        physics=task.physics,
    )
    for _, robot in tiled.cells:
        live.drive(robot=robot, group=task.group, max_velocity=getattr(task.control, "max_velocity", None))
        live.set_control(control, robot=robot, group=task.group)
    dims = live.set_observation(json.dumps(specs))
    assert len(dims) == len(channels) * len(tiled.cells), (dims, channels)
    rng = np.random.default_rng(0)
    render = _render_options(task, rng)
    if render is not None:
        live.set_lighting(render["light"], render["ambient"], render["shadows"])
    live.set_render_decimate(_decimate_cell(render, _auto_decimate(channels, single_scene.camera_fov)))
    if settle:
        live.tick(settle)
    t0 = live.t
    retired = [False] * len(tiled.cells)
    steps = 0
    while live.t - t0 < duration_s - 1e-9 and not live.finished and not all(retired):
        row = live.observe()
        for i, (_, robot) in enumerate(tiled.cells):
            if retired[i]:
                continue
            block = row[i * width:(i + 1) * width]
            obs = {key: _shaped(block[a:b], shape) for key, a, b, shape in offsets}
            info: Info = {"t": live.t - t0, "steps": steps, "channels": obs, "collision": bool(live.collisions())}
            if task.done is not None and task.done(obs, info):
                retired[i] = True
                continue
            flat = np.concatenate([obs[c.key].reshape(-1) for c in channels]).astype(np.float32)
            action = np.clip(np.asarray(model(flat), dtype=np.float64).reshape(-1), -1.0, 1.0)
            if action.shape[0] != task.control.dim:
                raise ValueError(f"the model returned {action.shape[0]} values, the control takes {task.control.dim}")
            live.act(action.tolist(), robot=robot)
        live.tick(k)
        steps += 1
    del bt
    return live.finish(publish=publish)


# ------------------------------------------------------------ randomisation
# Domain randomisation helpers for `Task.reset` — the setters a cell's
# things are moved and re-drawn with, on one prefix of a tiled scene or
# none. All draws come from the generator handed in.


def _set_down(scene, name: str, x: float, y: float, face: float, gap: float) -> None:
    """Moves obstacle `name`, as it is turned, so the middle of its footprint
    lands on (x, y) and its underside `gap` above the face at z = `face` —
    read off its bounds, so a primitive and a scan whose model frame sits
    wherever the scanner put it are set down alike."""
    (px, py, pz), _ = scene.obstacle_pose(name)
    lo, hi = scene.obstacle_bounds(name)
    scene.set_obstacle_pose(
        name,
        (x + px - (lo[0] + hi[0]) / 2, y + py - (lo[1] + hi[1]) / 2, face + gap + pz - lo[2]),
    )


def _footprint(scene, name: str) -> tuple[float, float]:
    lo, hi = scene.obstacle_bounds(name)
    return hi[0] - lo[0], hi[1] - lo[1]


def _pool(
    scene,
    rng: np.random.Generator,
    names: Sequence[str],
    k: int,
    spots: Optional[Sequence[tuple[float, float]]] = None,
    face: Optional[float] = None,
    *,
    prefix: str = "",
    jitter: float = 0.02,
    gap: float = 0.002,
    park: tuple[float, float, float] = (3.0, 3.0, 0.5),
) -> list[str]:
    """Draws `k` of the pool `names`: they are enabled and shown, the rest
    parked at `park` (spread along x), disabled and hidden — out of the
    physics, the pictures and the collision checks until a later draw
    brings them back. Given `spots` (x, y) and the `face` they are on, the
    drawn things are set down on `k` of the spots, shuffled: the middle of
    each footprint on its spot (jittered by up to `jitter` in x and y), the
    underside `gap` above the face, as it is turned. Without spots they
    stay where they are — place them yourself (`scatter` keeps things of
    any size apart). Returns the names drawn."""
    names = list(names)
    if not 0 <= k <= len(names):
        raise ValueError(f"pool: draw {k} of {len(names)} things")
    if spots is not None and len(spots) < k:
        raise ValueError(f"pool: {k} things for {len(spots)} spots")
    if spots is not None and face is None:
        raise ValueError("pool: spots are on a face — give its height (face=)")
    chosen = [names[i] for i in rng.choice(len(names), size=k, replace=False)]
    order = rng.permutation(len(spots))[:k] if spots is not None else [None] * k
    for name, s in zip(chosen, order):
        if s is not None:
            x, y = spots[s]
            dx, dy = rng.uniform(-jitter, jitter), rng.uniform(-jitter, jitter)
            _set_down(scene, prefix + name, x + dx, y + dy, face, gap)
        scene.set_obstacle_enabled(prefix + name, True)
        scene.set_obstacle_visible(prefix + name, True)
    for i, name in enumerate(n for n in names if n not in chosen):
        scene.set_obstacle_pose(prefix + name, (park[0] + 0.3 * i, park[1], park[2]))
        scene.set_obstacle_enabled(prefix + name, False)
        scene.set_obstacle_visible(prefix + name, False)
    return chosen


def _scatter(
    scene,
    rng: np.random.Generator,
    names: Sequence[str],
    region: tuple[tuple[float, float], tuple[float, float]],
    face: float,
    *,
    prefix: str = "",
    gap: float = 0.02,
    lift: float = 0.002,
    tries: int = 100,
    restarts: int = 20,
) -> dict[str, tuple[float, float]]:
    """Scatters `names` over `region` (`(x0, x1), (y0, y1)`) on the face at
    z = `face` (each underside `lift` above it) without overlap: footprints
    — read off each thing's bounds as it is turned — keep `gap` apart, so
    the physics plan's overlap audit stays empty. The biggest footprint goes
    down first, each thing gets up to `tries` draws, and a layout that
    boxes the next thing out starts over (up to `restarts` times) before
    the region is declared too small. Returns the footprint centres set."""
    (x0, x1), (y0, y1) = region
    sizes = {name: _footprint(scene, prefix + name) for name in names}
    for name, (lx, ly) in sizes.items():
        if x1 - x0 < lx or y1 - y0 < ly:
            raise ValueError(f"scatter: {name} ({lx:.3f} × {ly:.3f}) does not fit the region")
    order = sorted(names, key=lambda n: -sizes[n][0] * sizes[n][1])
    for _ in range(restarts + 1):
        placed: dict[str, tuple[float, float]] = {}
        for name in order:
            lx, ly = sizes[name]
            for _ in range(tries):
                x = rng.uniform(x0 + lx / 2, x1 - lx / 2)
                y = rng.uniform(y0 + ly / 2, y1 - ly / 2)
                clear = all(
                    abs(x - px) >= (lx + sizes[other][0]) / 2 + gap or abs(y - py) >= (ly + sizes[other][1]) / 2 + gap
                    for other, (px, py) in placed.items()
                )
                if clear:
                    placed[name] = (x, y)
                    break
            else:
                break
        if len(placed) == len(order):
            for name, (x, y) in placed.items():
                _set_down(scene, prefix + name, x, y, face, lift)
            return placed
    raise ValueError(f"scatter: {len(names)} things do not fit the region with {gap} m between them")


def _friction(scene, rng: np.random.Generator, names: Sequence[str], low: float = 0.4, high: float = 0.9, *, prefix: str = "") -> None:
    """Draws every thing's friction coefficient from `[low, high]`."""
    for name in names:
        scene.set_physics(prefix + name, friction=float(rng.uniform(low, high)))


randomize = SimpleNamespace(pool=_pool, scatter=_scatter, friction=_friction)


# ------------------------------------------------------------ rewards


def _reach(key: str, scale: float = 1.0):
    """`-scale · |translation|` of a `Relative` channel: closer is better."""

    def reward(obs: ObsDict, info: Info) -> float:
        return -scale * float(np.linalg.norm(obs[key][:3]))

    return reward


def _within(key: str, radius: float):
    """True once a `Relative` channel's translation is inside `radius`."""

    def done(obs: ObsDict, info: Info) -> bool:
        return bool(np.linalg.norm(obs[key][:3]) < radius)

    return done


def _bonus(key: str, radius: float, amount: float = 1.0):
    """`amount` while a `Relative` channel is inside `radius`, else 0."""

    def reward(obs: ObsDict, info: Info) -> float:
        return amount if np.linalg.norm(obs[key][:3]) < radius else 0.0

    return reward


def _collision_penalty(weight: float = 1.0):
    """`-weight` on any step the driven robot stands in a collision."""

    def reward(obs: ObsDict, info: Info) -> float:
        return -weight if info.get("collision") else 0.0

    return reward


def _combine(*terms):
    """The sum of reward terms."""

    def reward(obs: ObsDict, info: Info) -> float:
        return float(sum(term(obs, info) for term in terms))

    return reward


rewards = SimpleNamespace(
    reach=_reach,
    within=_within,
    bonus=_bonus,
    collision_penalty=_collision_penalty,
    combine=_combine,
)
