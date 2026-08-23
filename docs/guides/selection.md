# Selecting parts: requirements, the check, and the catalog

botrail does not choose parts. What it does is derive **what every line of
the bill of materials must be able to do** from the cell it sits in, compare
that with **what the chosen part says it can do**, and report where the two
disagree — or where nobody has said. Choosing stays with you, your agent, or
the vendor you send the open questions to. `bt.catalog.search` gives that
choice a list of products that exist.

```python
req = scene.requirements()        # one row per BOM line
print(req.to_markdown())

report = scene.check()            # I/O lint + sequences + parts + requirements
assert report.ok, report.to_markdown()

bt.catalog.search("gripper.parallel", stroke_mm=150, payload_kg=2.3)
```

## What the cell asks of a line

`scene.requirements()` walks the BOM and derives, for each line, the numbers
the cell already implies. Every requirement is a minimum and says where it
came from:

| line kind | requirement | derived from |
|---|---|---|
| robot | `payload_kg` | the tool's mass plus the heaviest part the robot grasps (now, or in an `attach` of a counted sequence) |
| robot | `reach_mm` | the farthest taught segment goal from the base, measured at the flange a catalog robot declares (the TCP for a plain URDF), plus `margin` (10 % by default) |
| tool (`<robot>/tool`) | `payload_kg` | the heaviest grasped part |
| tool, `gripper.parallel` | `stroke_mm` | the smallest side of the grasped parts — the least the fingers must open |
| beam sensor | `sensing_range_mm` | the beam's span |
| light curtain (`sensor.light_curtain`) | `range_mm`, `protective_height_mm` | the beam's span, the post height |
| zone sensor | `range_mm` | the half-diagonal of the zone |
| conveyor | `length_mm`, `width_mm`, `speed_mps`, `load_kg` | the transport zone along and across the belt, the belt speed, the mass of the parts starting on it |
| linear axis | `stroke_mm`, `speed_mps`, `load_kg` | the range, the speed, the mass of what it carries |
| vehicle | `max_speed_mps` | the travel speed |
| I/O node | `di`, `do`, `ai`, `ao`, `safe_di`, `safe_do` | the points assigned to it |
| pedestal / table | `load_kg` | the robots standing on it (base inside its top, at its height) and their tools |
| power supply (`power_supply`) | `output_a` | the sum of `current_a` over the other lines |

A number the cell cannot supply is **not guessed**: a grasped part with no
`mass_kg` leaves the payload out and adds a note (`requirement_incomplete`)
saying which part to give a mass. Requirements are geometric and
deterministic; there is no sizing, no safety evaluation, no choosing.

## What the part says

The "provided" column comes from the line's attributes — the specs a catalog
package carries (`Robot.from_catalog`, `attach_tool`, `bt.parts.*(catalog=...)`,
`bt.catalog.Product.identify`) or the numbers you type on `set_part`:

```python
scene.set_part("belt", manufacturer="MISUMI", model="GVL-900-200",
               length_mm=900, width_mm=200, max_speed_mps=0.2, load_kg=10)
```

A requirement key is answered by the attribute of the same name, or one of
its aliases (`speed_mps` reads `max_speed_mps` / `speed_max_mps`; `load_kg`
reads `capacity_kg` / `payload_kg`; see `bt.select.ALIASES`). The vocabulary
is shared with the catalog's manifests, so a value arrives in the same column
whichever way it came.

Each requirement ends in one of four states:

| status | meaning | finding |
|---|---|---|
| `ok` | the part states a value that satisfies the requirement | — |
| `short` | the part states a value that does not | `spec_short` (error) |
| `unknown` | the line is identified (maker / model / catalog) but states no value | `spec_unknown` (warning) |
| `unidentified` | nobody has said what the part is | carried into the `unidentified_part` note — the question to ask a vendor |

I/O nodes are the exception: their capacity is the I/O report's business
(`unbound`, `capacity`), so their rows appear in the table but raise no spec
finding.

## The check

`scene.check()` is every static check of the cell in one list — what
`botrail check` prints: the I/O lint, each sequence walked for dangling
references, unidentified lines with what the cell asks of them, and the
requirement comparison. `ok` is false only on errors; unknowns and notes are
information, not failure.

```python
report = scene.check()
for f in report.findings:
    print(f.severity, f.code, f.message)
```

```
error   spec_short          ur5e/tool: stroke_mm 85 < required 150 (smallest side of carton (150 mm))
warning spec_unknown        belt: needs load_kg >= 2.3 (1 part(s) on the belt at start) but the part does not say — add load_kg= on set_part or pick a catalog item
info    unidentified_part   eye (sensor.photoelectric) has no maker, model or catalog reference — needs sensing_range_mm >= 500
info    requirement_incomplete  ur5e: payload counts no mass for lid — give them mass_kg on set_part
```

A `spec_short` is the cell catching a wrong pick (a 2F-85 cannot open past a
150 mm carton). An `unidentified_part` with its `needs ...` is the line of a
request for quotation. Both are recomputed every time the cell changes: make
the carton heavier and the robot's payload requirement moves with it.

## Finding candidates

`bt.catalog.search` reads the catalog's published index — category, maker,
numeric specs, validation level — and filters it the way a requirement reads:

```python
bt.catalog.search("gripper.parallel", stroke_mm=150, payload_kg=2.3)   # minimums
bt.catalog.search("manipulator", reach_mm=900, payload_kg=6, level="V3")
bt.catalog.search("gripper", mass_kg__max=1.0, ip_rating="IP54")        # a maximum, a string spec
bt.catalog.search_for(scene.requirements()["eye"])                      # straight from a requirement row
```

A product that does not state a filtered key does not match — unknown is not
a pass. Results come ordered by validation level, then by closeness to the
minimums (the snuggest fit first), then by id, so the same query always
returns the same list. An empty list is information too: nothing in the
catalog satisfies the cell as drawn, so change the cell (grasp the carton
across its other side) or send the requirement out as a question.

Writing the pick back is one call. The identity and the numbers come along,
so the next `requirements()` reads them:

```python
cands = bt.catalog.search_for(req["eye"])
cands[0].identify(scene, "eye")        # set_part(catalog=(id, revision), maker, model, specs...)
assert scene.requirements()["eye"].status == "ok"
```

The index is fetched with `huggingface_hub` (`pip install botrail[catalog]`)
and pinned to a dataset commit; when the hub cannot be reached the newest
copy already in the Hugging Face cache is used, and `bt.catalog.index(path=...)`
or `BOTRAIL_CATALOG_INDEX` reads a local file (a catalog builder's
`dist/index.json`). `set_part(catalog=...)` with an id that is not in the
catalog is still allowed — it just stays `unknown` until the part's numbers are
typed, which is the point: a made-up model number cannot pass for a verified
one.

## Working with an agent

The loop an agent runs is the loop a person runs, with the guesswork removed:

1. read `scene.requirements().to_json()` — what each line needs and why;
2. for a line that is `unidentified` or `short`, call `bt.catalog.search_for(row)`
   and pick from what comes back — never from memory;
3. write the pick with `Product.identify(scene, target)`;
4. `scene.check()` until no errors remain; a line with no candidates stays an
   `unidentified_part` finding — the question to hand to a human or a vendor.

See [Working with agents and automation](agents.md) for the rest of the
protocol, and [Parts and the BOM](parts-and-bom.md) for where the identities
live.
