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
| Signal | `set_signal(name, value)`, `set_word(name, value)` |
| Wait on | `done()`, `robot_done(robot)`, `elapsed(s)`, `signal(name, value)`, `device_done(device)`, `word(name) >= …` |
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

## Two arms of one robot

A dual-arm robot ([arms of one robot](robots.md#arms-of-one-robot)) drives its
arms independently: a motion on one arm and a ramp on the other bake side by
side, and each arm has its own done test:

```python
sq.step("both", actions=[bt.seq.motion("left_pick"), bt.seq.motion("right_pick")],
        transition=bt.seq.immediately())
sq.step("left landed", transition=bt.seq.robot_done("oa", group="left"))
sq.step("grip", actions=[bt.seq.attach("part", group="left")])
sq.step("both idle", transition=bt.seq.robot_done("oa"))          # every arm
```

The rule underneath is **one driver per joint**: a motion, ramp or toolpath
owns the joints it moves for as long as it runs, and a step that starts a
second driver on a joint already in flight is a hard error naming the move and
its end time — never a silent overwrite. Grasps and tracks are per arm too
(`attach(..., group=)`, `track(..., group=)`, `untrack(group=)`), which is what
a two-handed carry is made of: one arm `attach`es the load, the other `track`s
it and follows wherever the first arm's planned motion takes it. Zone sensors
can watch one arm (`watch_groups=[("oa", "left")]`).

Two programs may share one dual-arm robot, an arm each — the PLC idiom of one
task per arm — and interlock through signals and zones exactly as two robots
would. A meeting of the two arms is a `GroupCollision` with the time and the
links, whether one program drives them or two. The
[Two arms, one robot](../tutorials/dual-arm.md) tutorial builds a kitting cell
this way.

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

## Rungs and gates

A step chain fires its actions once, on entering a step. What a PLC program
keeps *continuously* true — a self-holding run relay, an output that is on
while a step is active, an alarm that latches — is written as **rungs**:
contact networks scanned every scan, top to bottom, after the program's
step transitions. A program is a step chain, rungs, or both.

```python
sig, both, either, neg = bt.seq.signal, bt.seq.all_of, bt.seq.any_of, bt.seq.not_

lad = scene.sequence("common")                                   # rungs alone: the common circuit
lad.rung(both(either(sig("pb_start"), sig("run")), neg(sig("pb_stop"))), comment="run").coil("run")
lad.rung(both(sig("pb_stop"), sig("run"))).set("stop_req")       # -(S)-, and .reset(...) for -(R)-

out = scene.sequence("outputs")                                  # the output circuit
at = lambda step: bt.seq.step_active("auto", step)               # a step relay as a contact
out.rung(both(sig("run"), either(at("to st1"), at("to st2")))).coil("conv")   # a belt's run coil
out.rung(at("dwell")).ton("t_dwell", 0.8)                        # on-delay: read it as sig("t_dwell")
out.rung(at("clear")).ctu("made", 10)                            # up-counter: tl.word("made.count")
out.rung(sig("pb_reset")).reset("made")                          # … cleared by an RST of its signal
```

| output | what it does |
|---|---|
| `.coil(target)` | the target follows the rung: an internal signal, a conveyor's or a source's run coil (the device's name), a cylinder's solenoid (`"press/extend"`) |
| `.set(signal)` / `.reset(signal)` | latch on / off while the rung conducts |
| `.ton(signal, seconds)` | `signal` comes on once the rung has conducted that long, and drops with it |
| `.ctu(signal, preset)` | counts the rung's rising edges; `signal` is on from `preset` until a `.reset(signal)`; the count is the word lane `<signal>.count` |

A rung's network is any condition except `done()` and `elapsed()` — those
belong to a step; time a rung with `.ton`. `bt.seq.not_(…)` inverts a
contact or a whole network, `bt.seq.step_active(sequence, step)` reads a
step relay. Timers and counters declare their signal by use; coils and
latches write signals you `define_signal`. Ownership is the PLC's: every
target has one driver (a second coil on it is refused as a double coil),
and a signal is written by one program. Programs scan in the order they
are simulated, so a common circuit goes first and an output circuit last —
and within a program **rung order is scan order**: a relay written on rung
3 is seen by rung 4 in the same scan and by rung 2 in the next, which is
exactly the kind of thing a bake shows before a machine does.

The step chain takes **gates**:

```python
sq = scene.sequence("auto")
sq.step("wait", transition=sig("run"))
...
sq.gates(enable=sig("advance"), reset=sig("pb_reset"), cycle=True)
```

`enable` joins every transition (a step-mode switch, a pause): the chain
advances only while it holds. `reset` puts the chain back in — and holds it
in — its first step. `cycle` starts it over when it ends, in the same scan.

Rungs and a cycling chain never end, and a bake ends when its step chains
do — so they bake beside a program that does end, typically the
**operator's script**: a sequence that sets the push-button relays on a
timeline (`set_signal("pb_start")`, wait, release …). It makes the bake one
deterministic story and says when it is over; `simulate_sequences` refuses
a set of programs none of which ends. Live, the same cell runs for as long
as you like ([virtual commissioning](virtual-commissioning.md)).

Everything downstream follows: a rung's reads and coils are points of the
[I/O map](io-map.md) like a step's (with the rung's comment where a step's
name would be), the [ST export](virtual-commissioning.md) writes rungs as
assignments, `TON` / `CTU` blocks and the gates around the transitions, the
studio's [ladder](studio.md#the-ladder-ld) draws them under the step
ladder and lights them, and `tl.waiting(t)` names a contact a gated step
waits on. The PLCopen export writes step chains only and refuses a program
with rungs by name. This is the evaluation of a rung model with five
outputs — not an IEC 61131-3 runtime, and no vendor's instruction set: a
hand-written program still runs on its controller, with the cell as its
plant.

`examples/commissioning/ladder_line/run_demo.py` is a transfer line written
this way — common circuit, gated step chain, output circuit, an operator's
script (start, cycle stop, jog, step mode, reset) — baked, looped back
through its PLC's exchange bit for bit, and run as botrail's ST on OpenPLC.

## Words and analog inputs

Contacts and coils carry bits. What a process is run on — a setpoint, a
pressure, a count of milliseconds — is a **word**: a number the cell
defines, programs compare and rungs compute.

```python
from botrail.seq import word

scene.define_word("dp_limit", 80.0)                  # a REAL register
scene.define_word("t_valve", 0, integer=True)        # an INT: every write rounds

lad = scene.sequence("filter")
lad.rung(word("dp") >= word("dp_limit")).set("alarm")             # a comparison contact
lad.rung(stroking).move("t_valve", word("t_valve") + 10)          # one instruction, every scan it conducts
lad.rung(bt.seq.not_(stroking)).move("t_valve", 0)                # a transfer (MOV)
sq.step("retune", actions=[bt.seq.set_word("dp_warn", 40)])       # a step moves a word in on entry
sq.step("wait", transition=word("lot") == 10)                     # … and waits on one
```

`word(name)` compares with `>= > <= < == !=` — a contact like any other,
free to sit in `all_of` / `any_of` / `not_` — and computes with `+ - * /`,
**one instruction at a time**: `word("a") + 1`, not `(a + b) * c`. A longer
formula is one rung per instruction through an intermediate word, as on a
PLC. `.move(word, value)` writes while its rung conducts, every scan; put
an edge on the rung (`rising("tick")`) to count or to accumulate once. Two
whole operands (integer words, whole constants) make integer arithmetic —
`/` truncates toward zero, as `INT / INT` does; anything else is `f64`.
Dividing by a word that reads zero fails the bake by rung. Like a signal, a
word is written by one program.

Besides the words you define, a program reads the ones the plant records:
`"<device>.encoder"`, `"<sensor>.id"`, a counter's `"<signal>.count"` — and
**analog inputs**:

```python
scene.define_word("dp_true", 0.0)                    # what is really there
scene.add_analog("dp", source="dp_true", range=(0, 100), unit="kPa",
                 raw=(0, 4000), place="dpt/body")
```

An analog input is a transmitter on a 4–20 mA loop: it measures the word
lane `source` over `range` (the value at 4 mA and at 20 mA) into a module
whose counts run over `raw`. Every scan the plant walks the loop —
measured value → current → counts → the value the controller scales the
counts back to — and records two lanes: `dp.raw`, the INT on the wire, and
`dp`, the scaled value programs compare. The quantisation is therefore in
the bake too: `dp` moves in steps of one count (0.025 kPa here), and reads
what was measured a scan ago. Past its range the transmitter saturates
(3.8 … 20.5 mA); an **open loop** — `bt.io.open("dp")`, in a scenario or
injected live — reads 0 mA, a quarter of the span under the bottom count,
which is what a wire-break rung compares against
(`word("dp.raw") < -100`). `tl.analog(t)` reads the loop at an instant and
`scene.io_list("analog")` writes it as a sheet
([I/O map](io-map.md#the-tables)).

What the transmitter measures has to come from somewhere, and botrail has
no fluid solver. The equipment's behaviour is written the way the control
is — as a program, marked as **the plant's own**:

```python
proc = scene.sequence("process").plant()
flowing = bt.seq.all_of(bt.seq.signal("pump"), bt.seq.signal("valve/extended"))
proc.rung(flowing).move("clog", word("clog") + 0.08)     # an integrator: the element clogs
proc.rung(flowing).move("dp_true", word("clog"))
proc.rung(bt.seq.not_(flowing)).move("dp_true", 0)
```

A `.plant()` program runs with the plant in every mode — a bake, a
loopback, a live session with the PLC outside — and stays out of the I/O
map: what it reads and writes is physics, not wiring. The `pump` it reads
is then simply an output coil of the PLC that writes it.

`examples/commissioning/filter_unit/run_demo.py` is a filter unit run on
numbers this way: two pressure setpoints, a wire-break rung, a valve
watchdog counting milliseconds against a setpoint, a peak hold — baked,
looped back bit for bit, changed by hand while it runs, and run as ST on
OpenPLC ([virtual commissioning](virtual-commissioning.md#words-on-the-controller)).

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

## Process presentation

A weld cell reads better when the weld shows. Two idioms, both driven by
the baked timeline (never affecting it):

```python
scene.define_signal("st1_arc", False)          # the weld controller's output
scene.add_weld_flash("flash_st1_lh", signal="st1_arc", robot="st1_lh")
```

`add_weld_flash` binds an arc flash to a signal at a robot's TCP: while the
signal is true during playback, the studio draws an additive flash with a
point light there (deterministic flicker — the same bake renders the same
frames), and the USD export places a small emissive prim per current-on
interval, blinking via animated visibility, so the arc shows in usdview
too. Author the signal from the sequence that owns the weld — set it on the
weld step, clear it on the release.

Spot marks need no new machinery at all: a mark is a small dark obstacle
fed onto the seam by a [source](sensors-and-devices.md#sources-and-sinks-endless-supply-finite-pool)
started from the release step, and returned by the line's tail sink for the
next body. One mark per spot is the whole magazine — the recirculation is
what resets it between cycles.
