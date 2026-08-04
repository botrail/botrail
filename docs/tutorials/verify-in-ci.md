# Verify the cell in CI

*Walks through [`python/tests/test_cell_regression.py`](https://github.com/botrail/botrail/blob/main/python/tests/test_cell_regression.py)
— the regression suite this repository runs against its own cell, and the
pattern to copy into yours.*

The bake is deterministic: the same scene produces a bit-identical timeline
every run. That is not a performance footnote — it is what turns cell numbers
into test assertions. This tutorial reads a real test file that exercises the
idea end to end. It runs on the primitive-geometry arm from the checkout, no
downloads:

```bash
python -m pytest python/tests/test_cell_regression.py -v
```

```text
test_cell_cycle_regression PASSED
test_bake_is_deterministic_in_process PASSED
test_layout_change_shifts_the_cycle_deterministically PASSED

3 passed in 0.39s
```

0.39 seconds for three full cell simulations — cheap enough to run on every
commit.

## The cell under test

The same conveyor-beam-approach cell as
[Your first cell](../getting-started/first-cell.md), written as a function of
its layout:

```python
--8<-- "python/tests/test_cell_regression.py:32:56"
```

`build_cell(beam_x=...)` is the whole trick. A cell authored as a function can
be baked at any variant — which is what the third test below does.

## What to assert

The main test walks through every kind of check a timeline supports. Taking
them in order:

**A golden cycle, and a budget — two different assertions:**

```python
GOLDEN_CYCLE = 7.45
CYCLE_BUDGET = 8.0

assert tl.duration == pytest.approx(GOLDEN_CYCLE, abs=0.25)
assert tl.duration <= CYCLE_BUDGET
```

The golden catches *change* ("this edit moved the cycle"); the budget catches
*regression* ("the cycle no longer fits the takt"). The tolerance is worth
reading carefully — as the file's comment puts it, it absorbs libm-level drift
between machines, **not** behavior changes: a replan that adds a detour shifts
the cycle by far more than 0.25 s.

**The process happened, in order:**

```python
assert [name for name, _, _ in tl.step_spans] == ["feed", "stop", "approach", "work", "home"]
```

**Sensor timing, against an analytic value:**

```python
feed = tl.step_span("feed")
assert tl.signal("eye").rising_edges() == [feed.end]
assert feed.end == pytest.approx(1.9, abs=0.011)
```

The crate travels 0.475 m at 0.25 m/s = 1.9 s, quantized up to one 10 ms scan
tick — hence `abs=0.011`, one tick plus change. When you can compute the
expected number from the layout, do: this assertion documents the physics of
the cell, not just its history.

**Handshakes as waveform spans:**

```python
assert tl.signal("belt").high_spans() == [(0.0, feed.end)]
assert tl.signal("present").high_spans() == [
    (tl.step_span("stop").start, tl.step_span("home").start)
]
```

Devices are signal lanes too — `signal("belt")` is the conveyor's running
state. So "the belt ran exactly through feed" and "`present` covers stop→work"
are one-line checks.

**Clearance, over the whole cycle:**

```python
clearance = tl.min_clearance()
assert clearance > 0.3
assert clearance.pair is None
```

[`min_clearance()`][botrail.SequenceTimeline.min_clearance] samples the tightest
robot-to-environment approach across the cycle — a measure the rollout itself
never takes. `pair` names the touching links only while in contact, so
`pair is None` is the "and nothing ever touched" half of the check.

**The cycle ends where it should:**

```python
assert tl.sample(tl.duration) == pytest.approx(HOME, abs=1e-6)
```

## Determinism is exact, so test it exactly

```python
--8<-- "python/tests/test_cell_regression.py:101:110"
```

`==`, not `approx`. Two bakes of the same scene are bit-identical — durations,
step spans, signal edges, and every sampled configuration. If this test ever
fails, something nondeterministic crept into the pipeline, and every other
golden in the suite is on notice.

## A layout edit becomes a diff

```python
--8<-- "python/tests/test_cell_regression.py:113:123"
```

Move the beam 0.25 m downstream and the crate needs exactly one more second at
0.25 m/s — nothing else about the cell changes, and the test asserts precisely
that. This is the workflow in miniature: **a layout edit shows up as a
cycle-time diff a test can catch**, before it surprises the shop floor.

## Running it in your CI

The suite is ordinary pytest against the published wheel, so the workflow is
two steps:

```yaml
jobs:
  cell:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with:
          python-version: "3.12"
      - run: pip install botrail pytest
      - run: python -m pytest tests/ -q
```

Keep the golden values in the test file, next to the tolerance and the comment
explaining it. When a deliberate layout change moves the cycle, the failing
test *is* the review artifact: the diff updates the golden, and the reviewer
sees exactly what the edit cost.

## The complete test file

??? example "python/tests/test_cell_regression.py"

    ```python
    --8<-- "python/tests/test_cell_regression.py"
    ```

## Next

[Parameter sweeps](parameter-sweep.md) run the same loop as a study — many
variants, one table — and feed the budgets you assert here.
