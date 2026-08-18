# Offline commissioning: PLCopen XML and the trace diff

botrail never talks to a running controller — an online loop would cost
the determinism the rest of the toolchain stands on. What it does instead
is the two halves of commissioning that can be done *offline*, on files:

1. **Hand the logic over.** [`Scene.plcopen`][botrail.Scene.plcopen] writes
   the sequences as **PLCopen XML** (IEC 61131-10, TC6 v2.01) — SFC
   programs whose variables are the I/O map's points — for the PLC IDE
   (Beremiz, OpenPLC Editor, CODESYS through its import).
2. **Compare what came back.** [`SequenceTimeline.diff`][botrail.SequenceTimeline.diff]
   takes the controller's I/O log as a **trace** and compares it with the
   bake, edge by edge, by name.

```python
scene.export_plcopen("cell.plcopen.xml", name="pick cell")   # → the control engineer

trace = bt.trace.load("plc_log.csv", io=scene.io_map())      # tags / field names → point names
d = tl.diff(trace, tolerance=0.05, align_on="beam_pick")
print(d.to_markdown())
assert d.ok
```

Together with the [I/O list](io-map.md) and the
[robot program](export.md#robot-programs) — all derived from the same
scene — this is the control hand-over: what the PLC runs, what it is wired
to, what the robot executes, and the check that the machine did what the
design said.

## What the PLCopen file contains

One `<pou pouType="program">` per sequence (`sequences=` for a subset), in
SFC:

| botrail | PLCopen SFC |
|---|---|
| step | `<step>` (the first one `initialStep`) with an `<actionBlock>` for its entry actions and a `<transition>` for its condition |
| `select` arms | a `selectionDivergence` after the branching step, one transition per arm (authored order = priority; `otherwise` renders as `NOT (the others)`), the arms' steps, and a `selectionConvergence` into the next step |
| the last step | a `jumpStep` back to the first — a production PLC cycles (`cycle=False` parks the program in a final step) |
| `signal(x)` / `all_of` / `any_of` / `elapsed(t)` | ST: `x`, `NOT x`, `(a AND b)`, `(a OR b)`, `Step.T >= T#500ms` |
| `rising(x)` / `falling(x)` | an `R_TRIG` / `F_TRIG` instance the waiting step polls (`x_rise(CLK := x);`) and the transition tests (`x_rise.Q`) |
| `done()` / `robot_done(r)` / `device_done(d)` | the started function block's `done`, the robot's `done` input, the device's `_done` input |
| `start(dev)` / `stop(dev)` / `set_speed` / `move_to` / `goto` / `advance` | `dev_run := TRUE/FALSE;`, `dev_speed := …;`, `dev_position := …;`, `dev_station := n;` with a `dev_dispatch` boolean action, a `dev_index` boolean action |
| `set_signal(s, v)` | `s := TRUE/FALSE;` |
| `motion(m)` / `ramp` / `toolpath` / `attach` / `detach` / `track` / `untrack` | a call to a **stub function block** (`FB_StartMotion`, `FB_StartRamp`, `FB_StartToolpath`, `FB_Attach`, `FB_Detach`, `FB_Track`, `FB_Untrack`) with `start := TRUE`, awaited on `.done` — **or**, where the I/O map says the program drives the robot from another host, the `robot_start` boolean action with a `robot_program := n;` word, awaited on `robot_done` |
| a `Source` / `Sink` command | a comment: a magazine models an endless line, it is not equipment the PLC drives |

The **stubs** are shipped as function-block POUs with the body
`done := start;` — so the file runs in an IDE's simulator as-is, and the
control engineer replaces each body with the real controller interface (a
fieldbus handshake, a job number). Instances are one per robot and kind
(`panda_motion`, `panda_attach`); the motion's name comes in as a STRING.

**Variables** are the points of the [derived I/O map](io-map.md), declared
once as resource globals — `BOOL` for contacts and coils, `REAL` for
speeds and positions, `INT` for station and program words — with the
signal's initial value, and with `AT %IX0.0`-style addresses where the
binding lands on a PLC-family node (`plc`, `safety_plc`, `remote_io`).
Points bound on a robot controller get no address: they are the robot's
own I/O. Each program declares what it uses as `VAR_EXTERNAL`; the
configuration instantiates every program in one cyclic task
(`task_interval_ms`, default 10). Names are IEC identifiers
(`beam pick` → `beam_pick`, `far.done` → `far_done`), the same names the
robot script export uses.

The file is deterministic (fixed timestamps), so it hashes into the
[cell report](layout-and-report.md#the-cell-report) like every other
deliverable, and it validates against the TC6 v2.01 XSD (the repository's
tests do so whenever the schema is available). What it is *not*: a full ST
program, a vendor project, or a claim of compliance — it is a standard
file with the cell's logic in it and stubs where the machine begins.

## The trace diff

A trace is what the controller recorded: `{signal name: [(t, value), …]}`
— a CSV with `t,name,value` columns (`time` / `signal` / `tag` / `state`
are accepted; values `1/0`, `true/false`, `on/off`, `high/low`) or a dict
built any other way. [`bt.trace.load`][botrail.trace.load] reads it, and
`io=` renames binding tags and field-device names (`BEAM1`) to the bake's
point names.

`tl.diff(trace)` compares every signal both sides carry:

* **matched** edges — a rising or falling edge in the bake with one in the
  trace within `tolerance` seconds (default 50 ms), and the largest offset
  among them;
* **missing** edges — the bake switched, the trace never did;
* **extra** edges — the trace switched, the bake never did.

`d.ok` is "nothing missing, nothing extra"; `d.findings()` names each
deviation with its time (`missing_edge`, `extra_edge`, plus `not_in_trace`
/ `not_in_bake` for signals only one side carries — listed, not judged);
`to_markdown()` / `to_json()` render it. `align_on="beam_pick"` sets the
trace's clock against the bake's from the first rising edge of that signal
(a controller log starts whenever it starts); `signals=[…]` picks what to
judge.

```text
# Trace diff — MISMATCH

tolerance 0.050 s, alignment shift +0.000 s, largest matched offset 0.000 s.

| signal | matched | missing | extra | max offset (s) |
|---|---|---|---|---|
| vacuum | 0 | 2 | 0 | 0.000 |
| part_at_pick | 1 | 1 | 1 | 0.000 |
| conv | 2 | 0 | 2 | 0.000 |

Findings:

- vacuum: the bake rose at 5.550 s, the trace never did
- part_at_pick: the bake rose at 5.550 s, the trace never did
- part_at_pick: the trace rose at 5.850 s, the bake never did
- conv: the trace rose at 9.000 s, the bake never did
```

[`bt.trace.from_timeline(tl)`][botrail.trace.from_timeline] is the perfect
log — the bake as a trace — for tests and for writing the *expected* trace
out next to the program (`bt.trace.to_csv`).

## Getting a trace

Any log with a timestamp, a name and a level per row will do: a PLC's
trend export, a data logger on the I/O, URSim / the robot controller's own
recording. The repository does not include a logger — the controllers'
tools are better at that — and it does not include an online link on
purpose. The comparison is the deliverable: it says, by name and by
second, where the machine and the design parted.
