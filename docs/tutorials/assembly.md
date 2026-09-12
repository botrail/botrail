# Bolt a cover down

*Walks through [`examples/assembly/cover_bolting_demo.py`](https://github.com/botrail/botrail/blob/main/examples/assembly/cover_bolting_demo.py)
— a bolted joint as data, the screwdriver's program beside the arm's, and
the hand-over set with the tightening sheet, the interlock table and the
FAT rows. Fetches the UR5e, OnRobot RG6, robot stand and control box from
the catalog (cached after the first run), with dimensioned references for
the OnRobot Dual Quick Changer v3 and side-mounted Screwdriver 103961.*

A cobot at a bench takes a gearbox cover by its bearing boss, sets it on
the housing's two dowels, and screws it down: six M5 screws from a
presenter, in the star order the joint derives, each run down by the
screwdriver on the same wrist as the gripper. Nothing in it is a thread
simulation — the joint is a drawing's figures, the rundown is the tool's
own stroke timed from the thread's pitch, and the driver's answer is a
signal — which is exactly what the reviewer of a screwdriving cell wants
to read: the numbers, the order, the guards, and what the cell does when
a screw does not go in.

```bash
python examples/assembly/cover_bolting_demo.py cover_bolting.usdc
```

```text
cover bolting: cycle 89.11s, 6 screws in the order ['h5', 'h4', 'h0', 'h2', 'h1', 'h3']
...
fastening report:
  #1 h5 screw0: off 0.01 mm, tilt 0.00°, down 20.0 mm in 4.46s, ok (1) — align=pass tilt=pass seated=pass engagement=pass depth=pass torque=pass
  #2 h4 screw1: off 0.01 mm, tilt 0.00°, down 20.0 mm in 4.46s, ok (1) — align=pass tilt=pass seated=pass engagement=pass depth=pass torque=pass
  ...
cover fit: off 0.00 mm, -0.00 mm high, tilt 0.00° — position=pass tilt=pass seated=pass
per screw: 11.7s (driver 4.65s of it at 340 rpm)
min clearance over the cycle: 1.2 mm at 72.55s
...
  scenario baseline         ok
  scenario nok_once         ok
  scenario nok_twice        refused — timed out after 119.11s; programs waiting at: assemble/halt0, driver/idle2
  scenario estop_pressed    refused — timed out after 119.11s; programs waiting at: assemble/engage0, driver/idle0 — forced: panel/estop=true
  scenario feeder_empty     refused — timed out after 119.11s; programs waiting at: assemble/feed0, driver/idle0
  scenario start_wire_open  refused — timed out after 119.11s; programs waiting at: assemble/run0, driver/idle0 — forced: driver/start=false
wrote the document set to cover_bolting_deliverables/ (14 files hashed in the report)
```

![A UR5e with a gripper and a screwdriver on one wrist, the screwdriver on a cover screw](../assets/studio/assembly.png)

## The joint as the drawing states it

The housing's side of the joint is six threaded holes and two dowels;
the cover's side is six clearance holes and two dowel holes; between
them a screw, a torque and the thread engagement the joint needs.
[`bt.assembly.joint`][botrail.assembly.joint] takes exactly that and
derives the rest — a frame per hole, the seat, the tightening order, the
engagement per hole:

```python
--8<-- "examples/assembly/cover_bolting_demo.py:276:287"
```

The figures are checked before anything moves:
[`bt.assembly.check`][botrail.assembly.check] compares the screw with the
holes and the torque with the tool, and `--length 16` is refused at build
— an M5×16 through a 10 mm cover engages 6 mm of thread where the joint
needs 10. With `--catalog` the same joint is read from the workpiece
pack's `mounting` block instead, and no hole coordinate is typed in the
script ([ordering from the catalog](../guides/assembly.md#ordering-from-the-catalog)).

## Two controllers, one handshake

The driver is a program of its own — [`bt.assembly.driver`][botrail.assembly.driver]
writes it: on `start` it finds the drive, runs the screw down for the
time M5×0.8 takes at 340 rpm, pre-tightens, final-tightens at 40 rpm and
answers `ok` or `nok`; two `fault_*` lanes let a scenario make it answer
NOK. Its I/O node models the handshake while the arm's program runs on
the UR control box ordered from its pack. The driver's `DI`/`DO` addresses
are abstract simulation channels: the purchased OnRobot configuration
requires an externally powered Compute Box and robot integration, as
described in the [product notes](https://github.com/neka-nat/botrail/blob/main/examples/assembly/cover_bolting_demo.md).
An E-stop signal on the operator panel guards both simulated programs:

```python
--8<-- "examples/assembly/cover_bolting_demo.py:308:320"
```

Once the programs are written, [`Scene.auto_assign_io`][botrail.Scene.auto_assign_io]
gives every wire of the handshake an address on its node — the I/O list
and the PLCopen export carry them, and the report's I/O row reads
`0 unbound`:

```python
--8<-- "examples/assembly/cover_bolting_demo.py:328:330"
```

## The program

The arm's program is two calls. [`place`][botrail.assembly.place] fits
the cover — approach, descend, close, attach, carry, seat, release — and
declares that the cover is meant to meet the housing, its dowels and the
bench (`touch`), so setting it down is not a collision.
[`fasten`][botrail.assembly.fasten] appends the six screws in the
joint's order: request one, pick it with the bit, carry it over its
hole, engage, raise `start`, ramp the shank down the screw's length while
the driver runs, wait for the answer. `after=[placement.seated]` is the
first interlock — the first screw is picked only once the cover's
release has raised its `seated` signal:

```python
--8<-- "examples/assembly/cover_bolting_demo.py:490:502"
```

On the interlock table the two guards read as rows, not as conventions
— the first pick waits for the cover seated and a screw present, and no
`start` goes to the driver while the E-stop is in:

```text
| to_pick0 | motion `motion screw_to_pick (arm)` | `(feeder/present AND assemble/cover_seated)` | feed0 | ... |
| start0   | signal `driver/start := TRUE`       | `(DONE(screw_engage_h5) AND NOT panel/estop)` | engage0 | panel/estop (sensor) |
```

## The FAT rows

Each fault is a scenario, and the run of each must *refuse* the cycle —
stall at a named step — rather than let it through. A NOK once is
retried and passes; a NOK twice halts the cell with the screw released
where it stands; the E-stop in admits no start; an empty presenter
presents nothing; a broken START wire leaves the driver waiting for a
signal the arm did raise:

```python
--8<-- "examples/assembly/cover_bolting_demo.py:334:343"
```

The scenario table of the report is the FAT sheet: `2/6 passed`, and for
each refused row the step the programs were waiting at and the fault that
was forced.

## The hand-over set

`deliver` writes the set next to the USD — the layout sheet, the bill
(six screws as one line with a count), the I/O list, the handshake
specification, the PLCopen file with the driver's program on a resource
of its own, the interlock table, the **tightening sheet**
([`export_sheet`][botrail.assembly.export_sheet]), the fastening report
as JSON — and then the cell report, which hashes them and carries an
assembly section of its own
([`report_section`][botrail.assembly.report_section]):

```python
--8<-- "examples/assembly/cover_bolting_demo.py:528:549"
```

The tightening sheet is one line per screw in the order it is driven,
the design figures beside the baked ones:

```text
| order | hole | screw  | thread | length_mm | class | head     | engagement_mm | torque_nm | rundown_s | step | offset_mm | seated_mm | drive_s | attempts | result |
| 1     | h5   | screw0 | M5x0.8 | 20        | 8.8   | ISO 4762 | 10.0          | 4.50      | 3.75      | run0 | 0.01      | 20.0      | 4.46    | 1        | ok     |
```

and the report's assembly section states the joint, the driver's timing
and the fit above the same rows:

```text
## Assembly

Joint `cover_joint`: `set/cover` onto `set/housing`, 6 × ISO 4762 M5x20-8.8, order h5, h4, h0, h2, h1, h3.
Torque 4.00–5.00 N·m, driver `driver` set to 4.50 N·m (tool 0.15–5 N·m); thread engagement 10.0 mm against 10.0 mm needed; 1 retry allowed, then `assemble/halted`.
Driver time per screw 4.65 s: find 0.20 s, run 3.75 s (17 mm at 340 rpm, pitch 0.8), pre-tighten 0.20 s, final 0.50 s (0.33 turn at 40 rpm).
Fit of `set/cover`: off 0.00 mm, -0.00 mm along the seat, tilt 0.00° — position pass, tilt pass, seated pass.
Baked result: 6/6 screws driven.
```

All of it is also `cover_bolting_report.json`, so a regression test reads
the joint the way it reads the footprint:

```python
scene, tl, joint, driver, feeder, placement, fastening = demo.bake()
report, runs, rows = demo.deliver(scene, tl, fastening, placement, out_dir)
assert all(r["result"] == "ok" for r in report.sections[0]["json"]["screws"])
assert set(runs.errors) == {"nok_twice", "estop_pressed", "feeder_empty", "start_wire_open"}
assert report.io["unbound"] == 0
```

## What the check refuses

Two more runs are worth making. `--misalign 2` teaches the last screw
2 mm off its hole: the engage move is collision-checked and the screw is
allowed inside the cover only within its hole's window, so the bake
refuses it by name — `step 115 (assemble/engage5): ... collision: screw5
x set/cover`. And `--catalog` takes the presenter, screws and
housing-and-cover set from their packs. Both modes use the same OnRobot
tooling: the bill records catalog ids for catalogued parts and product
numbers and sources for the authored references. The requirements table
checks the tool's torque, screw length and thread range against the
screws it drives ([the guide](../guides/assembly.md)).
