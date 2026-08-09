# Sequences

A sequence is the cell's process, written the way a PLC writes one: a list of
**steps**, each with entry **actions** and a **transition condition**,
evaluated on a fixed scan cycle. If you have read a step-ladder or SFC
program, you already know this model.

```python
sq = scene.sequence("cycle")
sq.step("feed",  actions=[bt.seq.start("belt")], transition=bt.seq.signal("eye"))
sq.step("stop",  actions=[bt.seq.stop("belt")])
sq.step("pick",  actions=[bt.seq.motion("approach")])
sq.step("work",  transition=bt.seq.elapsed(0.5))

tl = scene.simulate_sequence("cycle")        # or sq.simulate()
```

## The scan model

The rollout advances in fixed ticks (`dt=0.01` s by default). Each scan: fire
the current step's entry actions (on the first scan of the step), evaluate its
transition, move on when it holds. Sensors update, devices advect, signals
latch — all on the same clock. The discreteness is not an approximation to
apologize for; it is the PLC execution model, and it is what makes the bake
[deterministic](../concepts/determinism.md).

## Steps

```python
sq.step(name, actions=[...], transition=...)
```

Omit `transition` and the obvious default is supplied: a step that starts a
motion or ramp waits for it (`done()`); a step that starts nothing passes
`immediately()`. That is why `stop`-style steps take zero time in the step
table.

Re-calling `scene.sequence(name)` starts that sequence over from zero steps —
a builder accumulates, it does not append across calls. A scene holds any
number of sequences (`sequence_names`, `remove_sequence`); the two-arm demo
keeps its `--clash` variant alongside the real one.

## Actions and conditions

The full vocabulary lives in the [`bt.seq` reference](../reference/api/seq.md).
The shape of it:

| | |
| --- | --- |
| Drive the robot | `motion(name)` — planned; `ramp(targets, duration)` — guarded, fixed-time |
| Handle parts | `attach` / `detach`, `track` / `untrack` |
| Drive devices | `start` / `stop` / `set_speed` / `move_to` |
| Signal | `set_signal(name, value)` |
| Wait on | `done()`, `robot_done(robot)`, `elapsed(s)`, `signal(name, value)`, `device_done(device)` |
| Combine | `all_of(...)` — series contacts; `any_of(...)` — parallel contacts |

Internal signals are declared up front (`scene.define_signal("carrying")`) —
PLC internal relays, written by actions, read by transitions, and visible as
waveform lanes on the baked timeline.

## Motions plan at their step

`bt.seq.motion("x")` does not replay a pre-planned path. The motion is planned
**when the step starts**, against a snapshot of the world at that moment:
whatever the robot is carrying rides along, and other robots stand frozen
where they happen to be. A cell edit upstream of a step therefore changes what
the step plans — which is the point.

## Several robots

Actions name their robot (`bt.seq.motion("far_to_pick")` on a motion authored
with `robot="far"`, `bt.seq.ramp(..., robot="far")`), and steps interleave
freely. Two idioms carry all the coordination:

* release early — `transition=bt.seq.immediately()` on the step that starts a
  transfer, so the sequence moves on while the motion runs;
* re-synchronize — `bt.seq.robot_done("far")` to wait for a specific arm to
  land, and zone-sensor interlocks to keep contested space exclusive.

The rollout checks robot-against-robot collision every tick; a meeting is a
hard, timestamped error, not a warning. The
[Two arms, one belt](../tutorials/two-robots.md) tutorial builds this up
properly.

## The bake

```python
tl = scene.simulate_sequence("cycle", dt=0.01, max_duration=120.0)
```

One call, one [`SequenceTimeline`][botrail.SequenceTimeline]: cycle time, step
spans, signal waveforms, per-robot joint tracks, object motion. Deterministic
— same scene in, bit-identical timeline out — and therefore
[assertable](timeline-assertions.md). Connected studios receive the bake and
show it in the timeline dock.

## Parallel programs

A line is not one sequence. Each station runs its own cycle and the transfer
is a program of its own — the PLC picture is one POU per station, and that is
exactly what runs here:

```python
tl = scene.simulate_sequences(["station_1", "station_2", "transfer"])
```

Every scan tick advances *every* program, in list order, over one shared
world. Determinism survives untouched: the scan order is fixed, so a signal
written by an earlier program is seen by a later one in the same tick, and
the bake stays bit-identical. The result is still a single timeline; step
spans carry `program/step` names.

Programs coordinate the way PLC programs do — through the world, not through
each other:

* **signals** — a station sets `st1_done`, the transfer waits
  `bt.seq.all_of(bt.seq.signal("st1_done"), ...)`, and releases the stations
  by dropping its own `moving` flag;
* **sensors** — a zone or beam is readable from any program;
* **`robot_done` / `device_done`** — idle tests work across programs.

Reading is free; *driving* is owned. Every robot, device, and written signal
must be commanded by at most one of the programs, validated before the first
tick — two programs ramping one robot is not a scheduling problem to referee
at runtime, it is an authoring error, the same as two PLC programs writing
one coil. A deadlock (a gate on a signal nobody sets) surfaces as the timeout
naming where every unfinished program is stuck.

## Indexed transfer

A transfer line moves in pitches, and a pitch is a *distance*:

```python
sq.step("index",
        actions=[bt.seq.advance("line", 5.2)],
        transition=bt.seq.device_done("line"))
```

`advance` runs a stopped conveyor for exactly that many metres along its
velocity direction and stops; the final scan tick moves exactly the
remainder, so the pitch never picks up a fraction of a scan period. This is
what retires the `start → elapsed(pitch / v) → stop` pattern and its
off-by-one-scan arithmetic — a body lands on the station datum to numerical
precision, every cycle, which is precisely what taught poses need.
