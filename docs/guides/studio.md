# The studio

`bt.studio(scene)` serves a 3D workbench for the scene on `127.0.0.1` and
opens your browser. The studio and your Python session hold **the same
scene**: drag the TCP gizmo and IK runs live with the result visible from
Python; call `scene.set_tcp_target(...)` and the browser's robot moves.
Everything the UI does, the API does — the two are one operation model over
one wire protocol.

```python
bt.studio(scene)                          # blocks until Ctrl-C
server = bt.studio(scene, block=False)    # keep the prompt (REPL / notebook)
server.url
server.stop()
```

![The studio](../assets/studio/overview.png)

The header names the robot and shows the connection dot. The viewport is a
full orbit camera over the cell, with the TCP gizmo on the end-effector and
the active TCP link named in the corner badge. Down the right side, the
panels — top to bottom: **ROBOT**, **SCENE**, **TCP**, **PLAN**, **MOTION**,
**SEQUENCE**, **OBSTACLES**, **JOINTS**.

## Placing the robot — ROBOT

The base pose, editable two ways: **Place base** for dragging it, or the
*place at frame* dropdown to snap it onto any named frame — the studio form of
`scene.set_robot_base_pose(*scene.frame(...))`. With several robots in the
scene, the panel's robot selector switches which instance the posing and
planning panels drive.

## The cell inventory — SCENE

The scene tree: robots, then the world's obstacles as imported (the counter
reads e.g. *147 obj · 5 frames*), then sensors and devices. Per obstacle, the
eye toggles display and the checkbox includes/excludes it from collision
checking (`set_obstacle_enabled`); sensors and devices carry a remove button.

## Posing — TCP and JOINTS

The TCP panel picks the IK link (the dropdown defaults to the model's TCP
link — on the Franka that is a fingertip, so pick the hand link for grasp
work) and switches the gizmo between **Move** and **Rotate**. Dragging solves
IK continuously; collision turns the offending geometry red as you go. The
JOINTS panel is the other door into the same state: one slider per joint.

## One-shot planning — PLAN

**Set goal** captures the current pose as the goal (a ghost robot marks it),
**Plan** solves, and the result plays back on a scrub bar:

![A planned trajectory in the studio](../assets/studio/plan.png)

The green readout is the plan: duration, waypoint count, planning time
(`2.03s · 2 wp · 147ms` above). Plans made from Python with
`broadcast=True` land in the same panel.

## Teaching motions — MOTION

The waypoint-segment editor, mirroring
[`add_segment`](motion-planning.md#named-motions-waypoint-segments): pose the
robot, then **+ Joint** or **+ Line** appends a segment ending at this
configuration (`upright` adds the orientation-cone constraint that keeps the
tool vertical). **Plan motion** solves the whole list rest-to-rest. **Save**
/ **Load** are `.botrail` [projects](projects.md); **Export .py** is
`generate_python()` — studio work exits as reviewable code.

## The process — SEQUENCE

Steps accumulate here the way `sq.step(...)` writes them: add a motion step
from a named motion, a one-second timer step, a grasp/release step for the
selected obstacle. Baking broadcasts the timeline to the dock:

![A baked sequence with the timeline dock](../assets/studio/sequence.png)

## The timeline dock

The bottom dock is the baked cycle as a timing chart: the cycle time
(*cycle 20.61s* above), one colored band per step, and one lane per signal —
internal relays, sensors, device running-states. The playback cursor drives
the viewport; the same scrub bar appears in PLAN during playback. Recordings
loaded with `play_usd_animation` — including two-robot bakes and Isaac
captures — play through the same dock.

## Serving details

```python
bt.studio(scene, host="127.0.0.1", port=0, open_browser=True, block=True)
```

`port=0` picks a free port. The server binds localhost and serves the bundled
UI; in a source checkout build it first (`./scripts/build_studio.sh`) or point
`BOTRAIL_STUDIO_DIR` at a built studio `dist/`. Several browsers can connect
to one scene — they all see the same state, live. And the studio also runs
[with no server at all](browser-only.md).
