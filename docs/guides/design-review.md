# Review the available design information

`scene.check().ok` means that the static checks found no errors. A cell
with unidentified equipment, missing specifications or unrecorded electrical
properties can still satisfy that condition. `bt.review` makes those gaps
visible alongside the evidence available from a [cell report](layout-and-report.md).

```python
import botrail as bt

review = bt.review(scene, stage="design")
print(review.to_markdown())
review.save("design_review.json")
for item in review.blockers():
    print(item.id, item.target, item.status, item.next_action)
```

The review reads the scene; it does not simulate or assign missing values.
It preserves the existing `check()`, `requirements()`, `cell_report()` and
`bom.total()` contracts. The new report includes the static check as
`review.check`, the optional cell report as `review.cell_report`, and `items`.

## Reading an item

| result | meaning |
|---|---|
| `pass` | the named comparison passed, or the named execution/measurement was supplied |
| `fail` | an explicit inconsistency or static error was found |
| `unknown` | inputs are incomplete, or the supplied observation cannot establish the requested conclusion |
| `not_run` | no relevant execution, comparison or document evidence was supplied |
| `not_applicable` | no applicable target, or an explicit author-supplied reason for excluding this item |

Each item has an `id`, `group`, `target`, `message`, `basis`, `evidence`,
`next_action`, `required` and `blocking`. `review.counts` counts each status.
Equipment identity and specification comparisons are separate items: a model
number does not fill a missing payload rating. A BOM line for which no
specification comparison was derived is `not_run`.

Electrical items check assignments and declared field/channel `voltage`
and `logic`. Missing values on either end are `unknown`. A known mismatch
is `fail` even when the existing I/O lint calls it a warning. Voltage uses
the same 0.5 V tolerance as that lint, not voltage-range or circuit analysis.
Internal/cosmetic points need no physical channel. Numeric interfaces remain
explicitly unevaluated.

## Choosing the review scope

| stage | required groups by default |
|---|---|
| `concept` (default) | `checks` |
| `design` | `checks`, `equipment`, `specifications`, `connections`, `simulation` |

`required` adds group names or exact item IDs. Other groups are `totals`,
`scenarios` and `deliverables`.

```python
review = bt.review(scene, stage="concept", required=["connections", "totals"])
assert review.ready, review.to_markdown()
```

Explicit failures block either stage, including in optional groups.
Required items with `unknown` or `not_run` also block. Concept reviews may
leave equipment selection and specifications unresolved; these items stay
visible. Additional requirements never remove a stage's defaults. Unknown
group/item names raise `ValueError`.

**`ready` applies to the listed review scope.** It does not certify a
complete design, infer omitted equipment, evaluate project-specific cycle
budgets or clearance margins, or establish safety performance. Project
acceptance conditions still belong in the project's tests.

## Known subtotals

By default the review counts whole-BOM `mass_kg`. Each subtotal contains
`known_subtotal`, `target_qty`, `known_qty`, `targets` and `missing`.
Five panels with known mass and four components without it are explicitly
reported as five of nine items contributing. All values missing yields
`None`; a supplied zero is known. Non-numeric, negative or non-finite values
are listed as missing/invalid.

```python
review = bt.review(scene, required=["totals"], totals={
    "mass_kg": None,                       # the whole BOM
    "current_a": ["valve", "photo_eye"],  # these declared loads only
})
print(review.totals["current_a"])
```

Include every name of a merged BOM row when selecting it: its quantity
cannot be split reliably from names alone. Unknown names are rejected.
`totals={}` requests no subtotal. The review does not infer which equipment
uses a particular utility; select the applicable loads explicitly.

These are sums of declared values. For supply-specific capacity checks, use
the [physical connection plan](connections.md). It sums directly connected
loads and preserves unknown consumption. The review includes its findings
under `connections:physical:...`, and checks each power supply's specification
against these budgets. A supply without a declared supply port stays
`unknown`. The former whole-BOM current requirement is removed.

## Simulation evidence

```python
runs = scene.simulate_scenarios(["pick"])
measured = scene.cell_report(scenarios=runs)
review = bt.review(scene, report=measured, sequences=["pick"], stage="design")
review.save("design_review.md")
```

A program without a matching baseline cycle in the supplied report is
`not_run`. Clearance is a separate measurement: a completed bake does not
fill a skipped scan. No robots or no environment obstacles makes that
measurement explicitly `not_applicable`.

Executed scenarios retain the observed completion/failure but remain
`unknown`: neither outcome alone establishes expected-behaviour acceptance.
Before execution they are `not_run`. Caller-supplied deliverable digests remain
`unknown` for revision consistency: hashes alone do not establish a common
design revision. No deliverable records means `not_run`.

Use `bt.review(scene, manifest="deliverables/rev1/cell_manifest.json")` to
verify a [batch export](layout-and-report.md#the-document-set-as-one-thing)
against the current cell before using its report. The package supplies the
program scope unless explicitly supplied; a different scope blocks the review.
Changed inputs/files block the review, generated files with verified provenance
pass their revision check, and external attachments remain unknown. Export
warnings and PLCopen stubs are separate unresolved items. Add
`required=["deliverables"]` to make resolving those items a review requirement.

When using `report=` directly, correspondence with the scene is the caller's
responsibility. Neither report mode evaluates FAT expectations. Requiring
`scenarios` or `deliverables` keeps unevaluated conclusions visible as blockers.

## Sources and follow-up work

Use the report's item IDs to attach a source, assumption or follow-up:

```python
review = bt.review(scene, annotations={
    "connections:(unhosted):input:eye:voltage": {
        "source_kind": "manufacturer",
        "reference": "sensor_datasheet.pdf p.3",
        "assumptions": "Confirm the ordered revision",
        "owner": "electrical design",
        "next_action": "Confirm the sensor output voltage",
    },
})
```

`source_kind` accepts `manufacturer`, `user_input`, `assumption`, `derived`,
`measured` or `unknown`. Manufacturer/measured sources require a `reference`;
a catalog identity alone does not establish a specification's source.
Optional `due` records the follow-up date as text. Annotation values must
be nonempty strings; unknown IDs/fields are rejected. A reference supplies
no missing specification value and does not resolve an `unknown` result.

An annotation can contain `not_applicable` with a reason, for example when
PNP/NPN comparison is inapplicable to a dry contact reviewed separately.
The original status stays in `evidence.observed_status`. Explicit failures
cannot be excluded this way. Review options are separate inputs; they are
not added to the `.botrail` project.

## Command line

```bash
botrail review cell.py
botrail review cell.py --stage design --simulate --report review.md
botrail review cell.py --scenarios --require scenarios
botrail review cell.py --config review_options.json --markdown
```

`--config` accepts a JSON object with `required`, `totals` and/or `annotations`,
as in Python. Repeatable `--require` adds to configured requirements.
`--scenarios` implies simulation; `--sequence` selects programs. JSON is
printed by default. Exit codes are 0 for a ready review scope, 1 for
unresolved items or a failed bake, and 2 for invalid input/configuration.
Existing `botrail check` exit codes are unchanged.

::: botrail.review.review

::: botrail.ReviewReport

::: botrail.ReviewItem
