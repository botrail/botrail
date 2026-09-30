# Cloth and garments

botrail can put a piece of cloth in a cell — a T-shirt or a plain sheet on a
table — and let the robots' grippers handle it: pinch a cuff, turn a sleeve
over, carry a hem across, hand a corner from one arm to the other. The cloth
is a simulated shell with its own contact against itself, the table and what
the grippers hold, and its motion rides the baked timeline like everything
else: the studio plays it, the USD recording carries it, and its positions are
there to assert on.

![Two UR5e arms fold a T-shirt on a table: the sleeves are turned in, then
the hem is turned over onto the shoulders](../assets/cloth_fold_hero.gif)
*`examples/cloth/tshirt_fold_demo.py`: two UR5e with Robotiq 2F-85 turn the
sleeves of a child's T-shirt in and fold the hem onto the shoulders. The
amber dots are the vertices a gripper holds.*

## What it answers, and what it does not

The cloth is simulated **after** the bake, against the finished cycle. Each
gripper's tool point follows its robot through the cycle and holds cloth while
a signal is on. The cloth never pushes a robot back and the robots plan as if
it were not there, so the cycle time, the reach and the interlocks are exactly
those of the cell without cloth. What the cloth adds is where the fabric ends
up.

| Question | Confidence |
|---|---|
| Does each hand reach the cuff, the corner, the fold line — and how long is the cycle? | certain — the ordinary reach and timing of the cell |
| Does the taught fold leave the garment folded: footprint, which edge lies where, what overlaps what? | high — this is what the shell and its contact are for |
| Does a flap stretch, snag or get dragged by the way it is carried? | high for the geometry of the carry; the cloth's stiffness is one fabric's |
| Does the hand really hold the cloth: slip, pinch force, a missed grasp? | not modeled — a grasp is ideal: the vertices near the tool point are taken |
| Does the cloth hang off the table's edge, or catch on a fixture? | not modeled — the support is a level plane, and only a gripper's pad pushes cloth |
| Is the robot loaded by the cloth? | not modeled — the coupling is one way |

## Put a cloth in the cell

```python
import botrail as bt

table = bt.parts.table(scene, "table", size=(0.8, 0.84, 0.72))   # adds the frame table/top
shirt = bt.cloth.tshirt("shirt", on="table/top", yaw=1.5708)     # the hem at -x, the shoulders at +x

marks = bt.cloth.landmarks(scene, shirt)       # {"cuff_left": (x, y, z), ...} and frames shirt/cuff_left, ...
# ... teach the arms against the landmarks; set a signal where a gripper closes ...

bt.cloth.add(scene, shirt, grippers=[
    bt.cloth.gripper("grip_north", robot="north", radius=0.05, compliance=0.02, soften=3),
    bt.cloth.gripper("grip_south", robot="south", radius=0.05, compliance=0.02, soften=3),
])
timeline = scene.simulate_sequence("fold")      # the cloth rides every bake from here on
```

A cloth lies flat on a level support. `on=` names the frame it lies on — a
table's `<name>/top` — which gives its centre, the support's height and its
heading; `position=(x, y, z)` does the same without a frame. At `yaw=0` the
width runs along `+x` and the length (a T-shirt's hem to its shoulders) along
`-y`.

| Cloth | Made with | Landmarks |
|---|---|---|
| T-shirt | `bt.cloth.tshirt(name, body=(0.50, 0.70), sleeve=(0.22, 0.20), layers="sewn" \| "single", neck="notch" \| "round", seams="shared" \| "stitched")` | `cuff_left/right`, `hem_left/right/center`, `shoulder_left/right`, `underarm_left/right`, `neck_front/back`, `chest` |
| Sheet | `bt.cloth.sheet(name, (width, length))` | `corner_nw/ne/sw/se`, `edge_n/s/w/e`, `center` (north is `+y`, east `+x` at `yaw=0`) |

`bt.cloth.landmarks` returns where those points lie before anything moves the
cloth and registers each as a frame, so a motion or an IK target can be taught
against `shirt/cuff_left` like against any other frame. Nothing is simulated
for it.

`bt.cloth.add` registers the cloth on the cell: every bake after it — a
`simulate_sequence` call, or the studio's simulate button — is followed by the
cloth's simulation, and the timeline that comes back carries its track.
`bt.cloth.remove(scene, name)` takes it out again. To try cloth settings on a
cycle that is already baked, `bt.cloth.animate(scene, timeline, cloth,
grippers)` runs one pass and returns the timeline with the track, leaving the
cell as it was.

## Grippers

A gripper is a signal and a tool point. The tool point is the robot's TCP, the
tip of `group=` (one arm of a dual-arm robot) or the frame of `link=`. While
the signal is on, the gripper holds:

- **Closing** takes the cloth vertices within `radius` of the tool point,
  through every layer — a pinch at a cuff lifts both layers of the sleeve.
  A gripper that closes with no cloth in reach holds nothing, and the track
  says so (`warnings`).
- **A hand-over** is a second gripper closing where the first one holds: it
  takes the whole patch over without the cloth being let go, and the first
  gripper opening afterwards drops nothing.
- **Opening** lets go. The cloth keeps the velocity it had.

`compliance` (m/N) decides how the patch is held. `0` pins it to the gripper.
A positive value holds each vertex on a spring, which is what a flap folded by
two hands needs: two rigid holds fight each other through the cloth as the
flap turns, and the solver gives up; `0.02` is what a T-shirt's hem takes.
`soften=n` weakens the hold tenfold per cloth step over the last `n` steps
before the gripper opens, so a taut flap settles onto what is under it instead
of snapping free.

`pad=(x, y, z)` gives the gripper a box, in its tool frame, that pushes cloth
it does not hold — a bar smoothing a sheet, a finger nudging an edge. Without
one a gripper passes through cloth. The cloth a gripper holds never collides
with its own pad.

The gripper's state is read at the cloth's sample times (every `step`, 0.1 s by
default). A signal pulse shorter than a step between two samples is not seen,
so give a grasp a beat: close, wait a few tenths, then move.

## Reading the result

```python
track = timeline.cloth("shirt")

track.position("hem_left", timeline.duration)   # where a landmark ended up
track.positions(12.0)                           # every vertex at t = 12 s
track.held(12.0)                                # the vertices a gripper holds then
track.failure                                   # None, or (time, reason) if the cloth stopped
track.warnings                                  # a gripper that closed on nothing, a step taken in parts

timeline.export_usd("fold.usda", fps=30)        # the cloth is a Mesh with time-sampled points
```

A track is sampled on the cloth's own clock — one sample per cloth step while
it moves, the two ends only of a span in which it lies still — and `positions`
blends between the samples around the time asked for, as the studio does. The
studio draws the cloth as a mesh and the held vertices as amber dots.

If the solver rejects a step — a flap landing hard on the layers under it, a
gripper pressing the patch it holds into the table — the pass retries it in
halves, down to eighths of a step, and says so in `warnings`. If an eighth is
rejected too it stops there: the bake still succeeds, the track ends at that
time and holds its last state, and `track.failure` says when and why.

## Teaching a fold

The demo's fold is the one a person makes on a table, and the things it has to
get right are about how cloth is carried:

- **A fold is a turn.** The hand carries its patch on an arc about the fold
  line, so the flap stays as long as it is. A straight carry stretches the
  flap or drags the garment along.
- **Let go above, not on.** The turn stops a few centimetres above the cloth
  it lands on and the flap drops the rest of the way. A hand that carries the
  patch down into the layer beneath it presses cloth into cloth.
- **Two hands on one flap hold it on springs**, and let go softly
  (`compliance`, `soften`).
- **The crease travels.** Nothing holds the garment down, so the fold line
  creeps as the flap comes over and the flap lands twice that far beyond where
  a fixed hinge would put it. Set the hinge back by the creep, read off the
  first bake — the way a fold is taught on a real cell.
- **Go slowly.** The demo turns a 25 cm flap over in eight seconds. The cloth's
  step is 0.1 s; a hand that moves a held patch several centimetres per step
  asks more of the solver than it gives.

## Resolution, cost and determinism

`spacing` is the mesh cell. 0.02 m is what the shell is qualified at: the
demo's child's T-shirt is about a thousand vertices and takes about a minute
of cloth simulation for its 38 s cycle; an adult shirt (2200 vertices) several
minutes. A coarser
mesh is not a quicker look at the same fold: cells wider than the cloth's
natural crease cannot fold at a seam, so a sleeve turned over drags the body's
side with it. Coarse meshes are for tests of plumbing, not of folds.

The pass runs on four threads and is deterministic on one machine: the same
cell bakes the same cloth. Debug builds keep the cloth solver optimized, so a
`maturin develop` build simulates cloth at the same speed as a release one.

## Limits

- The support is a level plane at the height the cloth was laid on: cloth does
  not fall off a table's edge, and obstacles of the scene do not collide with
  it. Robots pass through cloth except for a gripper's `pad`.
- Grasps are ideal. There is no pinch force, no slip and no force on the robot.
- One fabric: a plain cotton jersey's weight, thickness and stiffness, with
  `thickness`, `density` and `friction` to adjust. Cloths do not collide with
  each other.
- A cloth is part of the session, not of the project file: `save_project` does
  not write it, and a cell reloaded from a project needs its `bt.cloth.add`
  again. The studio draws a cloth in the playback of a bake; it is not an
  object of the scene tree that can be selected or moved.
- A streamed physics bake from the studio (the physics toggle) does not
  include cloth; the ordinary simulate button and every Python bake do.
