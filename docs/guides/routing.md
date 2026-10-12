# Cables and tubes

A cell is wired and piped: the robot cable from the arm's base to its
control box, the fieldbus from the box to the PLC cabinet, the air tube from
the valve to the hand. botrail routes them the way it does everything else
— you declare the route once, and what it is made of, and the rest is
derived from where things stand:

* the **centerline**, with every corner rounded to the line's minimum bend
  radius, re-resolved from the residents' current poses;
* the **lengths** — the run, the cut length (run plus end allowances plus
  slack), the purchase length (the shortest sold length that fits, or the
  cut rounded up to the order step);
* the **cable list** and the **BOM rows** (a line per route by its spec,
  identical specs merged with their lengths added; a line per clamp article;
  the ducts as the parts their generator pins);
* the **checks** — bends tighter than the line allows, a line leaving its
  connector sideways, a run too long for what is sold, a fixed span running
  into something, two routes crossing, a connection placed but not routed;
* the layout sheet's `ROUTE` layer, and `BasisCurves` in the exported USD.

Move the control box and the robot cable's length moves with it — and past
the 6 m its box ships with, the [requirements](selection.md) say so.

```python
import botrail as bt

cx, rt = bt.connections, bt.routing

# 1. Ports: where each line plugs in, which way it leaves (bt.connections)
cx.port(scene, "arm/cable", "arm", "power", "load",
        at=("arm", "base_link", (0.0, -0.085, 0.04)), exit=(0, -1, 0), straight_m=0.08)
cx.port(scene, "UR/robot_cable", "UR", "power", "supply", target_kind="io_node",
        at=("UR/body", (0.1, 0.134, -0.17)), exit=(0, 1, 0), straight_m=0.10)
cx.connect(scene, "UR/robot_cable", "arm/cable", name="robot_cable")

# 2. The line, as the data sheet states it — a rule every route made of it follows
rt.spec(scene, "ur_cable", media=["power"], od_mm=10.5, bend_r_fixed_mm=60,
        end_allowance_m=0.3, sold_lengths_m=[6, 12],
        manufacturer="Universal Robots", model="Robot cable")

# 3. What holds it: a floor duct (boxes + part + support in one call)
bt.parts.duct(scene, "duct", path=[(0.25, -0.45), (1.3, -0.45), (1.3, -0.72)],
              catalog="obo_bettermann/duct/lk4-60100/r1")      # 100 x 60 floor trunking

# 4. The route: the connection it carries, the line, the way
rt.route(scene, "robot_cable", carries="robot_cable", spec="ur_cable",
         via=["duct", (0.0, -0.22, 0.03)], orthogonal=True, slack_m=1.0)

print(rt.cable_list(scene).to_markdown())      # one row per route
report = rt.report(scene)                       # bends, exits, lengths, clashes
assert report.ok, report.to_markdown()
scene.check()                                   # the same findings, with everything else
```

`examples/engineering/routing_demo.py` is a whole small cell: a UR5e on a
pedestal with a suction hand, its control box, the PLC cabinet, a valve,
two ducts and four lines, its articles catalog packs (`--catalog-root DIR`
reads them from a local catalog root). The hand's two air tubes are dressed
the way a cobot's tool lines are in the field: a P-clip each on the robot's
base, then tied along the front of the upper arm and the underside of the
forearm (`rt.along`, a clip every 20 cm, the two tubes side by side, the
second tube's clips 3 cm further in so its loops nest round the first's
instead of lying in their plane), free only in the loops the axes need —
the base's turn, the elbow, the hand's 90° turn at the wrist. `--move-box 3`
moves the box to watch the cable list and
the `cable_m` requirement follow; `--shared-duct` runs the PLC link
through the robot cable's duct instead of its own and the report names the
sharing (`route_separation`); `--dress-conduit pafs21` dresses the two
tubes in a corrugated conduit held at the same two places on the arm, and
`--dress-conduit prfs16` in the smaller one, which they fill past the limit
(`duct_fill`, and the kit's row short of its bore).

## Articles from the catalog

A line, a clamp, a conduit or a kit, and a duct can each come from a
catalog spec pack: `catalog=` on [`spec`][botrail.routing.spec],
[`clamp`][botrail.routing.clamp] (and a route's `clamp=dict(catalog=...)`
for its auto clamps), [`carrier`][botrail.routing.carrier] and
`bt.parts.duct` takes the pack's id — or a package directory, the
builder's `build/<id>` — and the figures the pack states become the
defaults: a line's `od_mm`, bend radii, `mass_kg_m`, `media`, `max_length_m`
and how it is sold; a conduit's bore, diameter, bend radius, torsion, mass
and stroke; a clip's width as its holding length; a duct's inner section.
What you pass explicitly wins, what the pack does not state stays unknown
(a dress-pack kit nobody has measured leaves the bundle's fill unchecked
and the kit's requirement row saying so), and the pack's maker, part
number and reference go onto the BOM row. The catalog's first articles:
SMC TU0604BU tubing, Lapp ETHERLINE FESTOON PN Cat.5e, ABB Adaptaflex
PRFS16 and PAFS21 conduits, the Murrplastik FHS-SH-Set UR5 dress pack,
HellermannTyton H6P P-clips and OBO LK4 60100 floor trunking — their
figures from the makers' pages and distributors' tables, cited in each
pack, never guessed.

```python
rt.spec(scene, "tu0604", catalog="smc/tube/tu0604bu/r1", bend_r_moving_mm=20)   # the pack states the fixed radius
rt.carrier(scene, "dress", holders=[...], catalog="abb/conduit/adaptaflex-pafs21/r1", bend_r_moving_mm=60)
bt.parts.duct(scene, "duct", path=[...], catalog="obo_bettermann/duct/lk4-60100/r1")
```

Two more things a pack may state, when its maker's drawing says so. A
dress-pack kit that states where its holders go (`holders_at`, a link and
an offset per holder) places them itself: `rt.carrier(scene, "dress",
catalog=..., robot="arm")`. A product that states where its lines plug in
(`ports:` — the place on a link or on a body, the exit direction, the
straight length) answers [`bt.connections.ports_of`][botrail.connections.ports_of],
and [`port(..., from_catalog="robot_cable")`][botrail.connections.port] takes
that place as the default. The catalog states neither yet for the
products in this guide — positions are transcribed from drawings or
measured, never estimated — so the cell writes them; the mechanism is
there for the packs that will.

## Ports have a place

A port of the [connection plan](connections.md) takes part in the
electrical checks whether or not it has a place. To route it, give it one:
`at=` names the resident it sits on and the offset in that resident's frame
— `("obstacle_or_frame", (x, y, z))`, or `("robot", "link", (x, y, z))`
for a connector on a robot link — and `exit=` the direction the line leaves
the connector, in the same frame. `straight_m` is how much of the line must
stay straight out of the connector (the boot, the gland); the first bend
starts after it.

The port follows its resident: an obstacle's `set_obstacle_pose`, a frame,
a robot link's forward kinematics. That is why a route needs no re-drawing
when the cabinet moves.

## Specs are rules

[`bt.routing.spec`][botrail.routing.spec] declares a line once; every route
made of it follows. The values are the data sheet's — the outer diameter,
the minimum bend radius laid fixed (`bend_r_fixed_mm`, the fillet drawn at
every corner) and in continuous motion (`bend_r_moving_mm`), the mass per
metre, the media it may carry, the lengths it is sold in or the reel it
comes on, a maximum run (a 100 m Ethernet segment), the identity that goes
on the BOM row. A value you do not know you leave out, and the checks that
need it say `unknown` (`route_spec_unknown`) — nothing is assumed. SMC's
TU0604 polyurethane tube, for one, is listed at 15 mm minimum bend radius;
a continuous-flex cable is typically rated around 7.5 × its diameter. Those
numbers belong in the spec, not in botrail.

## Routes, way points, supports

[`bt.routing.route`][botrail.routing.route] names the connection(s) the
line carries — one, or several for a multi-core cable (I/O points and
cables are different counts) — its spec, and the way, in order from the
connection's **source** port to its **target** port:

| way point | written as | meaning |
|---|---|---|
| a point | `(x, y, z)` | the line passes here |
| a clamp | its name ([`clamp`][botrail.routing.clamp]) | the line is held here; on a robot link, a moving span starts or ends |
| a duct / tray | its name (`bt.parts.duct`, or [`routing.duct`][botrail.routing.duct] for a support alone) | the line enters the duct where its centerline is nearest the previous way point, follows it, and leaves where it is nearest the next |
| a body to run along | [`along("pedestal/column")`][botrail.routing.along], `along(("arm", "forearm_link"), pitch_m=0.15, face="-y")` | the line is laid on a face of the body's collision bounds, its radius plus the clamp's standoff off the surface, and held there by **auto clamps** every `pitch_m` (the route's `clamp_pitch_m`, 0.3 m) — `<route>/c0`, `c1`, … among the anchors, a BOM line of the route's `clamp` article; the face is `face` (`"+x"` … `"-z"`, in the body's frame) or else the side face nearest the way in and the way out — a side face: on a body longer one way (an arm link, a column) the run lies along it, never across its end; the entry and the exit are the neighbouring way points' projections, a lateral offset under a pitch goes into the way in, so the run is parallel to the body's edge, and `offset_m` moves the run sideways on the face (two lines along one face, side by side, each with its clips: `offset_m=0.007` and `-0.007`); on a robot link the run's ends are drawn in until they clear the robot's other links (the joint housings), and `inset_m=(way_in, way_out)` draws them in further — `(0.12, 0)` leaves the first 12 cm of an arm link to the loop that climbs to it |

A holder — a clamp, a carrier's holder, the auto clamps of a run — has a
**kind**: what it leaves the line free to do, the way the holders of a
real dress pack differ (a rigid P-clip, a rotating holder, a slide jaw, a
strap):

| `kind` | holds | leaves free | the model |
|---|---|---|---|
| `fixed` (default) | the place and the direction, no roll, no slide | — | the replay holds the line straight through it along its `direction` (the bisector of its two legs when unset; a run's clip along the run); the static rod holds its roll too |
| `rotating` | the place and the direction | the roll about the line's own axis | held like `fixed`; the twist between its span's holders leaves out the turn about the line there; the static rod lets the line roll |
| `sliding` | the place, across the line | sliding through, and the roll | not an anchor of the replay: a ring on its body (as long as the holder, a millimetre wider than the line) the line runs through, and the spans either side of it are one rope sharing their slack — `route_tension` is judged over that stretch |
| `strap` | the point | the direction, the roll | the replay holds one sample; the kinematic twist leaves out the roll |
| `retracting` | as `fixed`, on a carriage | the carriage runs along its rail (`axis`, in the body's frame, for `stroke_m`), pulled toward the start with `force_n` | the static solver runs the carriage: the line pulls it out as the arm reaches, the pull takes it back; the span's free length is measured to the rail, the report gives the travel (`retraction_m`) and `route_stroke` when it runs out of rail; the replay holds the carriage at the rail's start |

`rt.clamp("tie", at=..., kind="strap")`, `rt.carrier(holders=[..., dict(at=(robot, "forearm_link", (x, y, z)), kind="rotating", direction=(1, 0, 0))])`, `rt.route(..., clamp=dict(kind="strap"))` for the clips a run lays; a `cable.clamp` pack may state its `kind`. `direction` is in the body's frame.

A **carrier** ([`carrier`][botrail.routing.carrier]) is the dress pack: a
protective hose or 3D energy chain held in holders along the arm — the
upper arm, the forearm, the wrist — with a bore. A route passes it as one
way point (`via=["dress"]`) and takes its holders as anchors; every route
through it is a **bundle**: their cross-sections are checked against the
bore (`duct_fill`, past 50 %), power is kept apart from signal and network
lines (`route_separation`), and the hose is a line of the BOM
(`cable.carrier`) as long as the longest route dressed in it. A duct is a
bundle the same way.

`orthogonal=True` inserts axis-aligned elbows between way points (z, then
x, then y — down the pedestal, along the floor, over to the box), ignoring
components smaller than the bend radius so near-aligned points do not grow
jogs. Every corner is then rounded to the spec's fixed bend radius; a
corner whose neighbouring runs are too short for that radius is drawn with
the radius that fits and reported (`route_bend`).

A route is cut into **spans** by its anchors (the two ports, the clamps,
a carrier's holders, the auto clamps). A span whose anchors both sit still
is **fixed**: a closed-form polyline, checked here. A span with an anchor
on a robot link that moves with the joints (not the root, nor a link fixed
to it — a clip on the base casting sits still) or on an object a robot
carries is **moving**: it is drawn at its time-zero shape, translucent in the studio,
and judged over a baked cycle (below). A span whose two anchors sit on
*one* moving body — two clamps on the forearm — is **dressed**: it rides
the body, has no free length to size and no twist, and the replay only
watches what it touches. `slack_m` is the material beyond the measured
run — the service loop a moving span needs: one figure shared among the
route's moving spans, or a list with a figure per moving span in route
order (a loop growing toward the hand: `slack_m=[0.08, 0.12, 0.16]`).

The fixed spans are **colliders**: the motion planner keeps the arm off
them, the tick verification of a bake reports an arm that brushes one
(`route:<route>#<span>` in the pair), and `min_obstacle_distance` counts
them — the links a span's own ends sit on may touch it (a connector in the
arm's base). `collide=False` on a route takes it out of that set; the
static clash check runs either way. A fixed span is also watched over the
cycle: its design run is held against every arm as it moves, and an arm
that sweeps through it is `route_contact` with the time
(`route_clearance` under 20 mm).

## Lengths and the BOM

| figure | how |
|---|---|
| run (`length_m`) | the centerline, fillets included |
| cut length | run + `end_allowance_m` at each end + `slack_m` |
| purchase length | the shortest `sold_lengths_m` ≥ the cut, or the cut rounded up to `order_step_m`; none when the spec states neither — and `route_length` when nothing sold fits |

The [BOM](parts-and-bom.md) lists a line per route under `cable.<medium>`
(`cable.power`, `cable.signal`, `cable.network`) or `pneumatic.tube` /
`pneumatic.hose`, with its `length_m` (the cut), `purchase_length_m`,
`od_mm` and the spec's identity; routes of one spec merge into one row with
their lengths **added**, which is what purchasing wants. Clamps are a line
per article (`cable.clamp`) — the declared ones and the auto clamps of the
runs laid `along` a body, under the route's `clamp` article, so a column
clipped every 25 cm is a count on the BOM — ducts the `cable.duct` part their generator
pins — ducts of one model merge the same way, their run lengths added, as
does anything else bought by the metre under `cable.*` or `pneumatic.tube`. A part pinned to a route's name (`scene.set_part("robot_cable",
kind="route", model=...)`) overlays the spec's identity on that row. The
robot cable a placed controller needs — the `cable_m` row of
`scene.requirements()` — is the route's cut length when a route runs from
the arm to its controller, and the along-the-axes estimate otherwise.

[`bt.routing.cable_list`][botrail.routing.cable_list] is the table a wiring
design, purchasing or an ECAD import starts from: one row per route with
its line, ends, lengths, supports and status, as CSV, Markdown or JSON.
`export_cell` writes it (`<name>_cables.csv`) and the route report
(`<name>_routing.md`) beside the BOM whenever a cell has routes.

## The checks

[`bt.routing.report`][botrail.routing.report] resolves every route at the
current poses and checks it; `scene.check()` and `botrail check` carry the
same findings. Each has a code, a severity and, where it has a place, the
point and the arc length on the route:

| code | severity | meaning |
|---|---|---|
| `route_endpoint` | error | a port without a place, a resident or support that does not exist, a spec or connection the route names and nobody declared |
| `route_medium` | error | the line's `media` do not include the carried connection's medium |
| `route_spec_unknown` | warning | the spec states no `od_m` / `bend_r_fixed_m` (or no `bend_r_moving_m` on a route with a moving span) |
| `route_bend` | error | a corner rounder than the line allows: the fillet that fits is smaller than `bend_r_fixed_m`, or the corner is a hairpin |
| `route_exit` | warning | the line leaves a connector more than 60° off its exit direction |
| `route_length` | error / warning | the cut length exceeds the spec's `max_length_m`, fits none of the sold lengths (error); a moving span without `slack_m` (warning) |
| `route_clash` | error / warning | a span runs into an enabled obstacle (not its ducts, not the residents its anchors sit on) or into a robot link at the current pose — a span that rides the arm too, judged against every link of it but its mount (error); a clamp or a holder on a robot link holds the line inside the link's collision shape, so that link can be neither collided with nor judged for the spans it holds (warning) |
| `route_clearance` | warning | a fixed span passes within 20 mm of one |
| `route_crossing` | warning | fixed spans of two routes touch, outside a shared duct |
| `route_unrouted` | info | a connection whose ports both have a place and that no route carries — the wiring to-do |
| `duct_fill` | warning | the lines through a duct or a carrier take more than 50 % of its cross-section (lines of unknown diameter are named, not counted) |
| `route_separation` | warning | a duct or a carrier carries power together with signal or network lines |
| `route_tension` | error | over a cycle: a moving span's line (run + slack) is shorter than the free length its holders need at their farthest, with the time |
| `route_twist` | error | over a cycle: the holders turn about each other more than the spec's `torsion_deg_m` allows over the span |
| `route_bend_moving` | error | in the replay: the span bends tighter than `bend_r_moving_mm` |
| `route_stretch` / `route_sag` | warning | in the replay: a moving span strains past 25 % (nearly taut between its holders) / hangs farther off its chord than `sag_limit_m` |
| `route_contact` / `route_clearance` | error / warning | in the replay: the span touches / passes within 20 mm of a body that does not hold it; lying on the link one of its own clips is on (a stiff line leaves its clip along the arm, and rubs there only as far as its other end moves) is `route_contact` as a warning |
| `route_stroke` | warning | in the replay: a retracting holder's carriage runs out of its stroke |
| `route_exit_bend` | warning | on the plan: the first holder after a connector, on another body, sits within the connector's stub plus a bend radius — the connector takes the bend |
| `route_clamp_near_joint` | warning | on the plan: a declared holder sits on a joint housing (within 70 mm of the joint's axis) — the joint turns under it |
| `route_connector_holds` | warning | on the plan: a moving span of a cable ends at its connector on the moving body, with no holder before it to relieve the strain (a tube's push-in fitting is a holder) |
| `route_twist_room` | warning | on the plan: between a moving span's holders a joint turns the line about its own axis; room for ±180° at each such joint (a cobot's tool line) wants more twist than the line's torsion allows over the span — a swivel, a rotating holder or more line |
| `route_crossing` | warning | over the cycle: two replayed lines pass through one another (their centrelines come closer than their radii) — the replay's ropes do not see each other; the static solve keeps lines apart |

The connection plan's own failed checks (a supply range outside what the
load accepts, two supplies into one load) reach `scene.check()` as warnings
under `connection`.

## Moving spans over a cycle

A span that rides the arm is judged on the cycle, not at a pose. Give
[`bt.routing.report`][botrail.routing.report] the baked timeline (or
`scene.check(timeline=tl)`) and every moving span gets, from the
kinematics alone:

| figure | meaning |
|---|---|
| holders min–max | how far apart the span's two holders get over the cycle |
| free length needed | the farthest they get plus a bend radius at each end — what the line between them must measure; `route_tension` (error, with the time) when the span's run plus its slack is shorter |
| stroke | max − min: what a retraction system has to take up |
| twist | the largest relative rotation of the holders' frames over the cycle — bend and torsion together, the worst case for the torsion rating (how much of a turning flange's rotation the line takes as twist and how much as bend depends on its shape, which the kinematics do not have); `route_twist` against the spec's `torsion_deg_m` × the span's length |

These need no physics and are exact for the baked motion. For the shape,
[`bt.routing.animate`][botrail.routing.animate] replays the moving and the
dressed spans as ropes against the cycle — the [rope replay](rope.md), one
track per span named `route:<route>#<span>`, held at both ends by what
holds it in the plan — a port's `straight_m`, half a clamp's `length_m`,
over one sample spacing at least, laid the way the line runs through the
holder (the anchor's `tangent`: a port's stub, an auto clamp's run, a
declared clamp's bisector), so the rope leaves every holder the way the
holder points it — starting from its polyline with its slack hung in and
settling under gravity for `settle_s` before the cycle starts — straight at
rest, the way a tube or a cable comes off the reel, and a rod rather than
a string (`rest="straight"`, `rod_hz=120`, `bending_hz=120`: the
[rope](rope.md)'s settings, so a span holds a round loop between its
holders and folds where a tube would, not along the corners of the shape
it was laid in; the fixed spans, settled only to hang, stay ropes) —
colliding
with the obstacles around it (not one its starting line already passes
through — that is the static report's clash to name), the floor and walls
of the ducts it lies in (the cell models a duct as one box; the replay gets
the channel), a floor at `ground_z` and the robot it is held on — every
link of it, so a tube rests on the arm instead of falling through it, but
a holding link whose shape holds the held point (a clip seated in the
casting would push its own held particle away). The slack's dip is hung
on the side with the most room (a stiff line keeps the loop it is laid
in), and a link the starting shape still runs through is left out with a
warning, the report then naming the contact
(`arm_links=False` leaves the arm out; `settle_fixed=True` hangs the fixed
spans the same way, so nothing stands on its polyline). The stretch between two holders of a
carrier is one rope, the hose — `carrier:<name>#<i>`, the hose's diameter,
its `mass_kg_m` plus the lines inside, the largest slack any of them
brings — and every line in it reads that track; the hose's bend radius
(the larger of the line's and the carrier's) and torsion (the smaller)
are what it is judged by, once, in the carrier's name. The ropes are
computed side by side; they do not interact:

```python
tl = scene.simulate_sequence("cycle")
tl = bt.routing.animate(scene, tl, settle_fixed=True)
report = bt.routing.report(scene, tl)
report.spans                       # one row per moving span: the figures above and the replay's
tl.export_usd("cell.usda")         # the ropes as tubes, the spans no rope replays as curves
```

The replay adds, per moving span and with the time of the worst moment:
the tightest bend (`route_bend_moving` against `bend_r_moving_mm`), the
strain (`route_stretch`, a warning past 25 % — the rope is soft, so a span
pulled nearly taut between its holders reads as stretch; the free length
above is the figure to act on, read over a few particles against the rope's spacing so one
particle settling close to its neighbour is not taken for it), the sag off the chord between the holders
(`route_sag` past `sag_limit_m` — too much slack, or a clamp missing; the
limit is a working value to set per cell, not a standard's figure, and the
150 mm default is about where a service loop starts swinging into the next
axis's path) and, for every replayed span, the nearest body
(`route_contact` when touching, `route_clearance` under 20 mm). The arm a
span rides is one of those bodies: every link of it is judged but a holding
link whose shape holds the held point, which is the line's mount, and the
link a dressed span is laid along; a holding link that does not hold the
point is judged away from its clip — a port's straight stub, half a
clamp, two radii and a centimetre. The
rope's stiffness is the solver's native setting, not the line's: the
replay judges shape, clearance and slack; tension, torsion and fatigue
life are not figures it produces, and its bend radius (read over a short
window of particles, away from the holders) is how far the rope folded,
not the hose's rating. A fixed span hung with `settle_fixed` keeps its
numbers on the row but raises no finding for resting on what is under it. The studio's ROUTING tab lists
the cycle's findings with their times and seeks the playback to them; a
ring marks each for a moment either side of its time, riding the line
where the finding was read off a replayed rope (a bend, a sag, a contact)
and standing where the moment put it otherwise (a twist, a tension).

The static rod carries **torsion** too: between two holders that hold the
line's roll (a rigid clamp, a retraction's carriage, a connector on a
still body), the twist is the angle between one holder's material frame
carried along the line and the other's — zero as installed, counted from
there — and the rod is pushed toward the shape that carries less of it,
the way a twisted tube loops (`torsion_nm2`, GJ, `rigidity_nm2 / 1.4`
unless given). A holder that leaves the roll free (`rotating`, `sliding`,
`strap`, a swivel fitting) releases it. The track carries the twist per
frame, the span row reports the largest as `torsion_deg` beside the
kinematic `twist_deg` (bend and torsion together), and `route_twist`
judges the torsion when the track has it.

Lines laid within 15 cm of one another are solved **together** by the
static solver: at each frame every line keeps two radii off the others
where they stood at the last frame (and off itself, past its next
samples), and the ones within reach of another are then settled in turn
against the others at rest — so two tubes tied side by side or a loop
hanging past another line do not pass through each other. The replay's
ropes do, and `route_crossing` says where. Two lines held so that their
loops lie in one plane (tied side by side across the way a loop jumps)
still have to give way to each other: the solve puts one loop past the
other, and the twist that costs a line held in roll shows in `torsion_deg`
— stagger the clips, or bundle the pair.

`bt.routing.animate(scene, tl, solver="static")` draws the spans instead of
simulating them — the [rope](rope.md)'s static solver: at every frame each
span is the rod at rest between its holders, of least bending and
gravitational energy, its length what the plan gives it, held by every
holder as the replay holds it (a fixed port's stub and a run's clip in
direction too; a moving port — a swivel fitting — and a declared clamp at
their points), clear of every link of the arm and of the obstacles around
— a harness configurator's picture, with no dynamics, settling or spring
settings. `rigidity_nm2` is the flexural rigidity that
balances the bend against the weight (`None`: a polyurethane tube's for
each line's diameter). The holds are exact, so a clip pointing a stiff
line at a housing a centimetre away reads as the fold it would be, where
the replay's soft hold let the rope leave the clip at an angle; the report
judges the drawn frames as it does the replay's.

## Ways a route could take

[`suggest`][botrail.routing.suggest] lists the ways a route could run
through the ducts the cell has, best first: the way points you fixed —
points, clamps, carriers, runs along a body — stay where they are, in
order, and every ordered choice of up to three ducts is tried between
them, each candidate resolved and checked at the current poses. A
candidate carries its `via`, the ducts, the lengths, the bends and
supports, the static errors and warnings with their codes (a duct's
`duct_fill` / `route_separation` included — the shared duct comes back
with its warning), the run that lies outside any duct (`open_m`), a
`score` (length + the open run again + 0.3 per bend + 0.2 per support: a
metre in a duct beats a metre on the floor) and `current` for the way the
route takes now; the order is by errors,
then score, then warnings, the same every time for the same cell.
[`adopt`][botrail.routing.adopt] gives the route that way. Free space is
not searched: a candidate is a choice of ducts, not a path around an
obstacle — a point you place is how a route goes round something.

```python
for c in bt.routing.suggest(scene, "plc_link")[:3]:
    print(c["ducts"], c["length_m"], c["errors"], c["codes"])
bt.routing.adopt(scene, "plc_link", 0)
```

## Choosing the dress pack

A carrier is a line of [`scene.requirements()`](selection.md): the cell
asks it for a bore the bundle fits at 50 % fill (`inner_mm`) and, given
the baked cycle, for the hose length between its holders (`free_length_m`:
the holders' farthest distance plus a bend radius at each end, span by
span), the stroke a retraction would take up (`stroke_m`), the twist per
metre of hose (`torsion_deg_m`) and, where the replay ran, the bend radius
it must be rated for (`bend_r_moving_mm`, a maximum). Each is compared
with what the carrier states — `inner_mm`, `od_mm`, `bend_r_moving_mm`,
`torsion_deg_m`, `stroke_m` as declared, the hose length between the
holders as laid — and the row is what `bt.catalog.search_for(row)` turns
into a search for a kit (the `cable.carrier` catalog is still to come; a
kit nobody has catalogued answers as unstated). These figures are derived
and searched for, not linted: the route checks already say the same of
the lines inside the hose.

```python
req = scene.requirements(timeline=tl)
print(req["dress"].to_dict())            # inner_mm ≥ 8.5, free_length_m ≥ 0.6, stroke_m ≥ 0.04, …
bt.catalog.search_for(req["dress"])      # kits whose bore, hose and torsion cover it
```

Where to put a holder is the other half. [`sweep_holder`][botrail.routing.sweep_holder]
moves one holder of a carrier over candidate coordinates in its own frame
and tables, at each place, the stroke of the spans it ends, the hose length
the carrier needs, the twist and the number of route errors — read from
the kinematic report of the same baked cycle, nothing re-baked, the plan
put back afterwards. It returns a [study table](../reference/api/study.md),
so `best("stroke_m")` is the place a retraction has least to do:

```python
table = bt.routing.sweep_holder(scene, tl, "dress", 1, x=[-0.20, -0.26, -0.32, -0.38])
print(table.to_markdown())
best = table.best("stroke_m", where=lambda r: r["errors"] == 0)
```

## In the studio

Routes draw as tubes at their diameter, in the medium's colour (black
power, blue signal, green network, light-blue pneumatic) unless the spec or
the route says otherwise; a span that rides a robot is translucent — its
shape at the scene's pose. The aids show a cone at each port along its
exit, a ring at each clamp, and a ring where a check found something — red
for an error, amber for a warning.

While a baked cycle plays, each cable is drawn once. A span the cycle
replays (`bt.routing.animate`) is drawn by its rope, not by its design
tube; a span that rides a robot has no tube at all, because its design
shape is known only at the scene's pose and the playback has moved the
arm on — and the marks on the arm go with it. **design line** on the
ROUTING tab brings the design back beside the replay: every span the
playback hides as a thin dashed line in the route's colour, the marks on
the arm with it, all at the scene's pose. Dropping the bake draws the
cell as authored again. **ROUTING**
on the overlay tabs is the cable list with the findings under it; a row
lights its route in the viewport, and so does a route in the scene tree.
Everything re-resolves when an obstacle, a frame, a robot base or — for
the moving spans — a joint moves. **edit** on a route's row opens its way:
each way point as a line (a point with its coordinates, a support by
name, a run along a body with its pitch), to move up or down, delete, or
add after — a point, a duct, a clamp, a carrier, a body to run along; a
point way point gets a move gizmo in the viewport while its line is
selected. **candidates** lists the ways the route could take through the
cell's ducts (as [`suggest`][botrail.routing.suggest] does) and **adopt**
takes one. Every edit goes to the host as the route's new way, the
routes and the BOM come back re-resolved, and **Save** writes it with the
project — so what is drawn in the studio is what Python reads back.

## Where it stops

Routes measure and count; they do not size. No conductor cross-section,
voltage drop or protection; no tube flow or pressure drop; no tension,
torsion or fatigue figure — the twist and the free length are kinematic
facts of the holders, the shape is the [rope replay](rope.md)'s, and there
the stiffness is the solver's, not the cable's. A fixed span is checked
against the robots at their current pose, not along a baked cycle; a
moving span's replay collides with its own arm's other links and what is
around it, not with the links that hold it. Ducts and
clamps are counted and passed through, not sized: how many lines a duct
holds is a later check. Branching harnesses are two routes.
