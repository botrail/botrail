# Motion planning

Motions in botrail are **planned, not taught point by point** — you say where,
the planner finds a collision-free, time-parameterized path. That inversion is
what lets a layout change re-solve instead of breaking the cell.

## One-shot plans

```python
traj = scene.plan([0.6, -0.5, 0.8, 0.0, 0.4, 0.0])     # to joint positions
traj = scene.plan_to_pose((0.4, 0.1, 0.3))              # IK first, then plan
```

Under the hood: RRT-Connect with a deterministic seed, random-shortcut
smoothing, then time parameterization against the joint limits. The result is
a [`Trajectory`][botrail.Trajectory] — sample it, export it, or hand it to the
studio (with `broadcast=True`, the default, connected studios get it for
preview playback).

Planning is **deterministic**: the same scene state and goal produce the same
trajectory, every time. `seed` selects a *different* deterministic exploration
— useful when the default seed finds an ugly path — not a way to make it
reproducible; it already is.

## Named motions: waypoint segments

A one-shot plan answers "can it get there". A cell needs *named* motions that
sequences can start — built as segment lists:

```python
scene.add_segment("to_pallet", goal=drop_q)                # joint-space segment
scene.add_segment("to_pallet", goal=place_q,
                  kind="cartesian_line")                   # straight-line TCP move
scene.add_segment("to_pallet")                             # goal=None: capture the
                                                           # current configuration
traj = scene.plan_motion("to_pallet")
```

Segments append to the named motion (created when missing); `kind` is
`"joint"` (planned, collision-free) or `"cartesian_line"` (the TCP moves on a
straight line, followed by IK). Motions plan rest-to-rest at segment
boundaries. Manage them with `motion_names`, `motion_segments`,
`remove_segment`, and `clear_motion` — or interactively in the
[studio](studio.md)'s MOTION panel, which is the same list.

`goal=None` is the teach idiom: pose the robot — with the TCP gizmo, or
[`set_tcp_target`][botrail.Scene.set_tcp_target] — then capture.

## Path constraints

Constraints hold along a whole segment:

```python
scene.add_segment("carry", goal=drop_q,
                  # keep the tool's +z within 10° of world-down: a carried
                  # tray stays level
                  orientation_cone=((0, 0, 1), (0, 0, -1), 0.17))

scene.add_segment("thread", goal=exit_q,
                  # keep the TCP inside a world-aligned box: through a window
                  position_box=((0.2, -0.4, 0.3), (0.8, 0.4, 0.9)))
```

`orientation_cone=(axis_local, axis_world, angle_rad)` keeps a tool axis
inside a cone; `position_box=(min, max)` keeps the TCP inside a box. The
studio's `upright` toggle is the common orientation-cone case with one click.

## Trajectories

```python
traj.duration          # seconds, after time parameterization
traj.joint_names
traj.times, traj.positions, traj.velocities
traj.sample(t)         # cubic Hermite, clamped to the span
traj.segments          # the sparse planned waypoints per segment
traj.segment_ends      # where each motion segment ends on the time axis
```

`positions` is densified for time parameterization; `segments` is the sparse
planned path (shortcut waypoints for joint segments, IK follow points for
Cartesian ones) — the natural input for robot-program export, one move
command per waypoint. Exports are covered in [Export](export.md).

## In sequences

A sequence step that runs `bt.seq.motion("to_pallet")` plans the motion **at
that step**, against the world as it stands at that moment — other robots
frozen where they are, carried objects riding along. That is also why planned
motions cannot run while [conveyor tracking](attach-and-tracking.md) is
latched (their waypoints would be baked against a target that keeps moving);
guarded `ramp` moves fill that gap.
