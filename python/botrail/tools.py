"""Multi-purpose hands: one bracket, several tools, each with its own tip.

The end-effector in a machine-tending photograph is rarely one thing. A
plate on the flange carries a gripper for the workpiece, a pin for the
buttons and a fork for the door handle, and the robot *switches* between
them by turning its wrist so the right one faces the job — no tool
changer, no second robot. What the cell needs from such a hand is a
**tip frame per tool** to aim at, and the bracket's own geometry in the
collision check.

`multi_tool` builds that bracket as a robot model (a joint-less URDF
welded on with `Robot.attach_tool`): a round plate on the flange, and on
it any number of pins, forks and gripper mounts, placed in the plate's
frame (+Z out of the flange). Each tool is a link with geometry and a
`<name>_<tool>_tip` frame whose +Z points along the tool — the axis a
press, a hook or a grasp approaches on — so the same idiom serves every
tool: `scene.set_tcp_target(pos, quat, link=tip)`.

    bracket = bt.tools.multi_tool("hand", [bt.tools.Mount("gripper"),
                                           bt.tools.Pin("pusher"),
                                           bt.tools.Fork("fork")])
    hand = bracket.attach_tool(coupling, flange="hand_gripper").attach_tool(gripper)
    robot = arm.attach_tool(hand)          # tcp = the gripper's; the pin
                                           # and fork tips keep their names

The composite's `tcp_link` stays the gripper's, so motions and the
studio gizmo behave as for any gripper; a pose taught for the pin or the
fork just names its tip. IK asked for a tip moves only the arm — the
gripper's fingers are off that chain and keep their value.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence, Union

from ._core import Robot

Point3 = tuple[float, float, float]

# Anodised aluminium bracket and blackened steel tools (linear RGB).
ALUMINIUM: Point3 = (0.55, 0.56, 0.58)
BLACK_STEEL: Point3 = (0.06, 0.06, 0.07)


@dataclass
class Mount:
    """Where a gripper (or any catalog tool) bolts on: a link whose +Z is
    the tool's mounting axis. `at` is the mounting face's centre in the
    plate frame, `direction` its outward normal, `spin` turns the tool
    about that normal (the finger pads' axis, on a parallel gripper)."""

    name: str = "gripper"
    at: Point3 = (0.0, 0.0, 0.012)
    direction: Point3 = (0.0, 0.0, 1.0)
    spin: float = 0.0


@dataclass
class Pin:
    """A pusher: a round pin of `length` and `diameter` from `at` along
    `direction`. Its tip frame sits at the free end, +Z along the pin —
    a button is pressed by putting the tip the button's travel into the
    cap along that axis."""

    name: str = "pusher"
    length: float = 0.060
    diameter: float = 0.012
    at: Point3 = (0.040, 0.0, 0.022)
    direction: Point3 = (1.0, 0.0, 0.0)
    color: Point3 = BLACK_STEEL


@dataclass
class Fork:
    """A door hook: two prongs `gap` apart reaching `reach` from a
    crossbar at `at` along `direction`, lying across `across`. The tip
    frame is the seat between the prongs, `seat` from the crossbar, +Z
    along the prongs — a handle bar is taken by driving the seat onto it
    along that axis, and the door goes wherever the fork then goes."""

    name: str = "fork"
    reach: float = 0.080
    gap: float = 0.030
    prong: float = 0.010
    seat: float = 0.050
    at: Point3 = (-0.040, 0.0, 0.022)
    direction: Point3 = (-1.0, 0.0, 0.0)
    across: Point3 = (0.0, 1.0, 0.0)
    color: Point3 = BLACK_STEEL


Tool = Union[Mount, Pin, Fork]


def _unit(v: Sequence[float]) -> Point3:
    x, y, z = (float(c) for c in v)
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        raise ValueError("tools: a direction must not be zero")
    return (x / n, y / n, z / n)


def _cross(a: Point3, b: Point3) -> Point3:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _perpendicular(a: Point3) -> Point3:
    pick = (1.0, 0.0, 0.0) if abs(a[0]) < 0.9 else (0.0, 1.0, 0.0)
    d = sum(p * q for p, q in zip(pick, a))
    return _unit(tuple(p - d * q for p, q in zip(pick, a)))


def _rpy(x: Point3, y: Point3, z: Point3) -> str:
    """URDF `rpy` (fixed-axis roll, pitch, yaw) of the frame whose columns
    are the unit vectors `x`, `y`, `z`."""
    r00, r10, r20 = x
    _r01, r11, r21 = y
    _r02, r12, r22 = z
    pitch = math.atan2(-r20, math.hypot(r00, r10))
    if abs(math.cos(pitch)) < 1e-9:
        roll, yaw = math.atan2(-r12, r11), 0.0
    else:
        roll, yaw = math.atan2(r21, r22), math.atan2(r10, r00)
    return f"{roll:.9f} {pitch:.9f} {yaw:.9f}"


def _frame(direction: Point3, spin: float = 0.0, x_hint: Optional[Point3] = None) -> tuple[Point3, Point3, Point3]:
    """An orthonormal frame with +Z along `direction`; +X from `x_hint`
    (projected) or a stable perpendicular, then turned by `spin` about Z."""
    z = _unit(direction)
    x = _perpendicular(z) if x_hint is None else None
    if x is None:
        h = _unit(x_hint)
        d = sum(p * q for p, q in zip(h, z))
        x = _unit(tuple(p - d * q for p, q in zip(h, z)))
    y = _cross(z, x)
    if spin:
        c, s = math.cos(spin), math.sin(spin)
        x, y = tuple(c * a + s * b for a, b in zip(x, y)), tuple(-s * a + c * b for a, b in zip(x, y))
    return x, y, z


def _xyz(p: Sequence[float]) -> str:
    return " ".join(f"{float(v):.9f}" for v in p)


def _rgba(color: Point3) -> str:
    return " ".join(f"{float(v):.4f}" for v in color) + " 1"


@dataclass
class _Urdf:
    name: str
    links: list[str] = field(default_factory=list)
    joints: list[str] = field(default_factory=list)

    def link(self, name: str, geometry: str = "", origin: str = "", color: Optional[Point3] = None) -> None:
        if geometry:
            material = f'<material name="{name}_m"><color rgba="{_rgba(color or ALUMINIUM)}"/></material>'
            body = (f"<visual>{origin}<geometry>{geometry}</geometry>{material}</visual>"
                    f"<collision>{origin}<geometry>{geometry}</geometry></collision>")
        else:
            body = ""
        self.links.append(f'<link name="{name}">{body}</link>')

    def fixed(self, parent: str, child: str, xyz: str, rpy: str = "0 0 0") -> None:
        self.joints.append(
            f'<joint name="{child}_joint" type="fixed"><parent link="{parent}"/>'
            f'<child link="{child}"/><origin xyz="{xyz}" rpy="{rpy}"/></joint>'
        )

    def text(self) -> str:
        body = "\n  ".join(self.links + self.joints)
        return f'<?xml version="1.0"?>\n<robot name="{self.name}">\n  {body}\n</robot>\n'


def multi_tool(
    name: str,
    tools: Sequence[Tool],
    *,
    plate: tuple[float, float] = (0.080, 0.012),
    color: Point3 = ALUMINIUM,
) -> Robot:
    """The bracket as a robot model — `multi_tool_urdf` loaded. See there."""
    return Robot.from_urdf_string(multi_tool_urdf(name, tools, plate=plate, color=color))


def multi_tool_urdf(
    name: str,
    tools: Sequence[Tool],
    *,
    plate: tuple[float, float] = (0.080, 0.012),
    color: Point3 = ALUMINIUM,
) -> str:
    """The bracket as URDF text — what `multi_tool` loads, and what a
    catalog asset of the hand is written from, so the made part and the
    catalogued one are the same geometry.

    A round plate `plate = (diameter, thickness)` whose
    root link `<name>_plate` sits on the robot's flange (+Z outward), with
    the `tools` on it. Every tool becomes links under `<name>_`:

    * a `Mount` — the link `<name>_<mount>` at its face, +Z along its
      normal: the `flange=` to `attach_tool` the gripper on;
    * a `Pin` — the pin's body `<name>_<pin>` and the frame
      `<name>_<pin>_tip` at its end, +Z along the pin;
    * a `Fork` — the crossbar and prongs under `<name>_<fork>` and the
      frame `<name>_<fork>_tip` at the seat between the prongs, +Z
      along them.

    Returns a joint-less `Robot` to weld on with `attach_tool`; `attach_tool`
    a gripper onto a mount first and the composite's TCP is the gripper's.
    A tool drawn through the plate's own axis is not refused — a hand is
    its author's business — but a `Mount` whose face lies inside the
    plate would bolt a gripper into the bracket, and that is."""
    diameter, thickness = float(plate[0]), float(plate[1])
    if diameter <= 0 or thickness <= 0:
        raise ValueError("multi_tool: the plate needs a positive diameter and thickness")
    for t in tools:
        if not isinstance(t, (Mount, Pin, Fork)):
            raise TypeError(f"multi_tool: {t!r} is not a Mount, Pin or Fork")
    names = [t.name for t in tools]
    if len(set(names)) != len(names) or "plate" in names:
        raise ValueError("multi_tool: tool names must be distinct, and not 'plate'")
    u = _Urdf(name)
    root = f"{name}_plate"
    u.link(root, f'<cylinder radius="{diameter / 2:.6f}" length="{thickness:.6f}"/>',
           f'<origin xyz="0 0 {thickness / 2:.6f}"/>', color)
    for tool in tools:
        link = f"{name}_{tool.name}"
        if isinstance(tool, Mount):
            if float(tool.at[2]) < thickness - 1e-9 and math.hypot(tool.at[0], tool.at[1]) < diameter / 2:
                raise ValueError(f"multi_tool: the mount {tool.name!r} sits inside the plate")
            x, y, z = _frame(tool.direction, tool.spin)
            u.link(link)
            u.fixed(root, link, _xyz(tool.at), _rpy(x, y, z))
        elif isinstance(tool, Pin):
            if tool.length <= 0 or tool.diameter <= 0:
                raise ValueError(f"multi_tool: the pin {tool.name!r} needs a positive length and diameter")
            x, y, z = _frame(tool.direction)
            u.link(link, f'<cylinder radius="{tool.diameter / 2:.6f}" length="{tool.length:.6f}"/>',
                   f'<origin xyz="0 0 {tool.length / 2:.6f}"/>', tool.color)
            u.fixed(root, link, _xyz(tool.at), _rpy(x, y, z))
            u.link(f"{link}_tip")
            u.fixed(link, f"{link}_tip", f"0 0 {tool.length:.6f}")
        elif isinstance(tool, Fork):
            if min(tool.reach, tool.gap, tool.prong) <= 0 or not 0 < tool.seat <= tool.reach:
                raise ValueError(f"multi_tool: the fork {tool.name!r} needs positive reach, gap and prong, "
                                 "and its seat within the reach")
            # Fork frame: +Z along the prongs, +X across them.
            x, y, z = _frame(tool.direction, x_hint=tool.across)
            span = tool.gap + 2 * tool.prong
            bar = f'<box size="{span:.6f} {tool.prong:.6f} {tool.prong:.6f}"/>'
            u.link(link, bar, f'<origin xyz="0 0 {tool.prong / 2:.6f}"/>', tool.color)
            u.fixed(root, link, _xyz(tool.at), _rpy(x, y, z))
            for side, sign in (("l", -1.0), ("r", 1.0)):
                prong = f"{link}_prong_{side}"
                u.link(prong, f'<box size="{tool.prong:.6f} {tool.prong:.6f} {tool.reach:.6f}"/>',
                       f'<origin xyz="0 0 {tool.reach / 2:.6f}"/>', tool.color)
                u.fixed(link, prong, f"{sign * (tool.gap + tool.prong) / 2:.6f} 0 0")
            u.link(f"{link}_tip")
            u.fixed(link, f"{link}_tip", f"0 0 {tool.seat:.6f}")
    return u.text()


def tip(name: str, tool: str) -> str:
    """The tip frame's link name for `tool` on the hand `name`."""
    return f"{name}_{tool}_tip"
