# Spray painting

botrail can check a spray program the way a paint engineer would — is the
gun the right distance away, square enough on, and pointed at the part —
and then bake the cycle and integrate the film it leaves: microns per
patch, in-spec area, holidays, the paint bill, and where the overspray
went. All of it deterministic, so it regression-tests in CI like the rest
of a cell.

![A hood section coated by a bell: the film builds up stroke by stroke,
read against spec](../assets/painting_hero.gif)
*`examples/painting_hood_demo.py`: a wrapped raster on a curved hood, per-
stroke triggered, the film building up against a 20–30 µm spec (neutral on
target, blue thin, red thick) and the `spraying` lane in the dock.*

## What it answers, and what it does not

The film model is **calibrated geometry, not fluid dynamics**. An applicator
carries the footprint measured on a coupon at a known standoff; the
integrator projects that footprint onto the surface along the spray axis,
scales it for range and incidence, and integrates over the time the gun was
actually spraying — on the *baked* trajectory, so a stroke that lost speed
is a stroke that laid on more paint. There is no air flow and no
electrostatic field, so an ESTA bell's wrap around an edge is not modeled,
and absolute microns are only as good as the coupon fed in.

| Question | Confidence |
|---|---|
| Does every stroke reach, and does the gun clear the fixtures? | certain — the ordinary reach and clearance checks |
| Is the standoff / incidence within the shop's rules? | certain — geometry (`check_paint`) |
| Are there holidays; is the lap ripple in band? | high — coverage is what the model is good at |
| The film's *relative* distribution (lap streaks, starved ends, dwell) | high |
| Absolute film thickness | as good as the calibration |
| Electrostatic wrap, sags, dry spray, colour | not modeled |

Two lessons the model teaches quickly: paint is conserved, so on a gentle
curve the standoff and angle rules move the *mean* film very little (they
protect what the geometry does not carry — transfer efficiency, sags, dry
spray); and the arm's slowdowns land wherever the turnarounds are, so
overtravel is what keeps them off the part.

## The applicator

```python
import botrail as bt

# From the shop's static-pattern coupon: radius/film pairs, meters, after
# spraying `seconds` at `standoff`. Shape *and* delivery rate come off it.
pattern = bt.paint.from_profile("coupon.csv", standoff=0.25, seconds=3.0)
bell = bt.paint.applicator(pattern, transfer_efficiency=0.85)

# Or an analytic fit, before anyone has sprayed anything.
bell = bt.paint.applicator(bt.paint.bell(0.16), standoff=0.25,
                           flow=25e-6 * 0.15 * 0.064 / 0.85, transfer_efficiency=0.85)
fan = bt.paint.applicator(bt.paint.fan(0.30, 0.08), standoff=0.25, flow=200e-6)
```

`bell` is axisymmetric (a rotary atomizer), so the 5-DOF solver keeps the
spin about the tool axis free; `fan` is a flat fan and wants its spin
pinned across the direction of travel (`spin="fan"` on the generators).
The tool frame convention is the toolpath solver's: the TCP's `+Z` runs from
the nozzle tip toward the gun body, paint travels along `-Z`.

## Strokes from the surface

Painting has no CAM. Rasters come from the surface and the shop's rules —
pattern width, lap overlap, gun speed, standoff, overtravel:

```python
tp = bt.paint.strokes((0.24, 0.18), standoff=0.25, pattern_width=0.16,
                      overlap=0.6, speed=0.15, overtravel=0.10, frame="part")

tp = bt.paint.wrap_strokes(0.5, 0.24, standoff=0.25, pattern_width=0.16,
                           overlap=0.6, speed=0.15, overtravel=0.10,
                           arc=(-0.37, 0.37), center=(0, 0, -0.5), axis="x",
                           frame="part", brush="top")
scene.add_toolpath("coat", tp)
```

`strokes` rasters a flat area; `wrap_strokes` wraps the same raster onto a
cylinder so the gun stays radial. Both return an ordinary
[toolpath](../reference/api/scene.md), authored in a part frame — move the
fixture and the program re-solves.

## Brushes: the program's own trigger

A *brush* (ABB's word) is a named process setting: an applicator, a flow
multiplier, and the trigger's lead and lag. Declared on the scene,
referenced from strokes:

```python
scene.define_applicator("bell", bell)
scene.define_brush("primer", applicator="bell", flow=0.6)
scene.define_brush("top", applicator="bell", flow=1.0, lead=0.25, lag=0.25)
```

Once any stroke of a toolpath names a brush, the program triggers per
stroke: the laps spray with their brush and a feed move without one runs at
speed with the gun **off** — which is how `wrap_strokes(brush=...)` leaves
the turnarounds dry. Two triggers decide when paint flows and both must
agree: the PLC's enable signal, and the program's own strokes. The approach
the rollout plans in from wherever the robot stood, and the rapids, never
spray the part however the enable was authored.

## Check before you bake

```python
report = scene.check_paint("coat", "hood", standoff=(0.23, 0.27),
                           max_incidence=math.radians(10))
report.ok, report.in_band_ratio, report.on_target_ratio
report.spans("too_far")            # stretches of the path, meters along it
```

Pure geometry — no robot involved, so it is the same answer whichever arm
ends up carrying the gun. Off-target stretches (a raster's overtravel) are
reported but do not fail the check: whether the gun should be closed there
is a triggering question. In the studio the findings sit on the path as
coloured points.

## Bake, and read the film

```python
sq = scene.sequence("cycle")
sq.step("purge", actions=[bt.seq.set_signal("purge")], transition=bt.seq.elapsed(2.0))
sq.step("ready", actions=[bt.seq.set_signal("purge", False), bt.seq.set_signal("gun_on")])
sq.step("spray", actions=[bt.seq.toolpath("coat")], transition=bt.seq.done())
sq.step("close", actions=[bt.seq.set_signal("gun_on", False)])
tl = sq.simulate()

film = tl.spray_coat("hood", gate="gun_on", spec=(20e-6, 30e-6),
                     facing=(0, 0, 1))
film.mean, film.sigma, film.in_spec_ratio, film.uncoated_area
film.sprayed_volume, film.deposited_volume, film.effective_transfer_efficiency
film.overspray()          # {"bench": 3.8e-6, "mask": 1e-7}: where the rest went
film.sprayed_by_brush()   # per brush
```

`facing` names the job by the way it faces (the top of a panel), so the
statistics do not depend on how far the raster overtravels. `spec` turns
the film map diverging — neutral on target, blue thin, red thick — and gives
you `in_spec_ratio`, the headline number. The baked twin of the pre-bake
check is `tl.paint_report(...)`.

Because the bake is deterministic, these are your tests:

```python
def test_hood_makes_spec():
    tl = bake()
    film = tl.spray_coat("hood", gate="gun_on", spec=SPEC, facing=(0, 0, 1))
    assert film.in_spec_ratio > 0.99
    assert film.uncoated_area == 0.0
    assert scene.check_paint("coat", "hood", **RULES).ok
```

## Show it

```python
scene.show_film(film)                                # the film map, with its key
tl = scene.animate_paint(tl, "hood", gate="gun_on", spec=SPEC,
                         facing=(0, 0, 1), trigger_signal="spraying")
scene.add_spray_cone("jet", "spraying", scene.robots[0], length=0.25, radius=0.08)
tl.export_usd("cell_painting.usdc")
```

`animate_paint` re-walks the coat in stages and swaps the visible mesh along
the timeline, so the film builds up during playback — in the studio, in the
exported USD (visibility-switched stages), and in a replayed recording. It
also writes the effective trigger as a signal lane (`spraying` = enable AND
program), which is what the timing chart shows and what the spray cone
follows. Declare the signal and the cone with the cell: a timeline exports
the scene it was baked from.

Two readings of the same film, picked with `style`: **amount** (a sequential
ramp, light to dark — how much paint is there; pass `paint_color=` and it
runs from a light wash to the paint's own colour, so the part visibly takes
the paint as the coat goes on) and **spec** (diverging over the band —
neutral on target, blue thin, red thick: the verdict). `spray_coat` and
`show_film` default to *spec* when a spec was given, `animate_paint` to
*amount*. Bare, never-sprayed patches wear the part's own colour. The spray
cone's ring is the pattern's footprint at the calibrated standoff — the
range the gun works over on the part.

Two worked examples: `examples/painting_demo.py` (a flat panel: calibration,
lap overlap, gun speed) and `examples/painting_hood_demo.py` (a curved hood:
the pre-bake check, brushes and the trigger, the paint bill, the build-up).
