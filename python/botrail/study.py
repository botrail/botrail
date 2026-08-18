"""Parameter studies over a cell: sweeps and deterministic optimisation.

A cell authored as a function of its parameters can be baked at every
variant and read by the numbers that matter — that is what makes layout
studies a loop and cycle time a regression test. `sweep` runs the grid and
tables the numbers; `optimize` searches it (a full grid, or a coordinate
descent on the grid) for the best feasible point. Neither uses a random
number: every row is a deterministic bake, the same every run, so a study
is as assertable as a single cell.

    result = bt.sweep(build_cell, grid={"velocity": [0.1, 0.2, 0.3], "lane_y": [0.4, 0.5, 0.6]},
                      metrics=lambda tl: {"cycle": tl.duration, "clearance": float(tl.min_clearance())})
    print(result.to_markdown())
    print(result.pivot("lane_y", "velocity", "cycle"))

    best = bt.optimize(build_cell, space={"velocity": (0.1, 0.4, 0.05), "lane_y": (0.3, 0.7, 0.05)},
                       objective="cycle", constraints={"clearance": (">=", 0.3)}, metrics=...)
    print(best.params, best.row["cycle"])

`build(**params)` returns a `Scene` (baked with `sequence=` / `sequences=`,
default: every sequence together) or a `SequenceTimeline` (used as is);
`metrics(timeline)` returns a dict of numbers (default `{"cycle":
duration}`). A variant that does not bake — a layout the planner cannot
solve — is a row with `ok=False` and its `error`, not an exception: the
table says where the cliff is.
"""

from __future__ import annotations

import csv
import io as _io
import itertools
import json
import math
import multiprocessing
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterable, Optional, Sequence, Union

Params = dict[str, Any]
Row = dict[str, Any]

# ------------------------------------------------------------ evaluation


def _default_metrics(timeline) -> dict[str, float]:
    return {"cycle": float(timeline.duration)}


def _bake(scene, sequence, sequences, max_duration: float):
    """Bakes a scene the way the study was told to."""
    import botrail as bt

    if isinstance(scene, bt.SequenceTimeline):
        return scene
    if sequence is not None:
        return scene.simulate_sequence(sequence, max_duration=max_duration)
    names = list(sequences) if sequences is not None else list(scene.sequence_names)
    if not names:
        raise ValueError("the built scene has no sequence to bake (pass sequence= or sequences=)")
    if len(names) == 1:
        return scene.simulate_sequence(names[0], max_duration=max_duration)
    return scene.simulate_sequences(names, max_duration=max_duration)


def evaluate(build, params: Params, *, metrics=None, sequence=None, sequences=None, max_duration: float = 120.0) -> Row:
    """One variant: build, bake, measure. Returns the row (`params` ∪
    `metrics` ∪ `ok` / `error`); a variant that fails to build or bake is a
    row with `ok=False`."""
    row: Row = dict(params)
    try:
        built = build(**params)
        timeline = _bake(built, sequence, sequences, max_duration)
        measured = (metrics or _default_metrics)(timeline)
        if not isinstance(measured, dict):
            raise TypeError(f"metrics must return a dict of numbers, got {type(measured).__name__}")
        for key, value in measured.items():
            row[key] = float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else value
        row["ok"] = True
        row["error"] = None
    except Exception as e:  # noqa: BLE001 — the study reports, it does not abort
        row["ok"] = False
        row["error"] = f"{type(e).__name__}: {e}"
    return row


def _evaluate_star(args):
    build, params, metrics, sequence, sequences, max_duration = args
    return evaluate(build, params, metrics=metrics, sequence=sequence, sequences=sequences, max_duration=max_duration)


def _worker_init(paths: list[str]) -> None:
    """A spawned worker sees the parent's import path, so `build` — a
    module-level function of the calling script — unpickles there."""
    sys.path[:] = paths


# ----------------------------------------------------------------- Sweep


def _num(v: Any) -> str:
    """A parameter value as the user wrote it (`0.1`, `2`, `"fast"`)."""
    if v is None:
        return ""
    if isinstance(v, bool):
        return str(v)
    if isinstance(v, float):
        return "nan" if math.isnan(v) else f"{v:g}"
    return str(v)


def _metric(v: Any) -> str:
    """A measured value at a table's precision: two decimals from 1 up,
    three below (a cycle reads `10.30`, a clearance `0.530`)."""
    if v is None:
        return ""
    if isinstance(v, bool):
        return str(v)
    if isinstance(v, (int, float)):
        f = float(v)
        if math.isnan(f):
            return "nan"
        if abs(f) >= 1e6 or (f != 0.0 and abs(f) < 1e-3):
            return f"{f:.4g}"
        return f"{f:.2f}" if abs(f) >= 1.0 else f"{f:.3f}"
    return str(v)


@dataclass
class Sweep:
    """The rows of a study, in evaluation order: each is the variant's
    parameters, the metrics measured (missing on a failed row), `ok` and
    `error`."""

    params: list[str]
    metrics: list[str]
    rows: list[Row] = field(default_factory=list)

    def __len__(self) -> int:
        return len(self.rows)

    def __iter__(self):
        return iter(self.rows)

    @property
    def ok(self) -> list[Row]:
        """The rows that baked."""
        return [r for r in self.rows if r.get("ok")]

    @property
    def failed(self) -> list[Row]:
        return [r for r in self.rows if not r.get("ok")]

    def where(self, predicate: Callable[[Row], bool]) -> "Sweep":
        """The rows a predicate keeps (failed rows are never kept)."""
        return Sweep(self.params, self.metrics, [r for r in self.ok if predicate(r)])

    def best(self, metric: Union[str, Callable[[Row], float]], *, minimize: bool = True,
             where: Optional[Callable[[Row], bool]] = None) -> Optional[Row]:
        """The row with the smallest (or largest) `metric` among the rows
        that baked and satisfy `where`; ties go to the earlier row (grid
        order), so the answer is stable."""
        candidates = [r for r in self.ok if where is None or where(r)]
        if not candidates:
            return None
        key = (lambda r: r[metric]) if isinstance(metric, str) else metric
        pick = min if minimize else max
        return pick(candidates, key=key)

    def pareto(self, minimize: Sequence[str] = (), maximize: Sequence[str] = ()) -> list[Row]:
        """The non-dominated rows for several objectives at once — the
        trade-off front (a shorter cycle against a wider clearance)."""
        goals = [(m, 1.0) for m in minimize] + [(m, -1.0) for m in maximize]
        if not goals:
            raise ValueError("pareto: name at least one metric to minimize or maximize")
        rows = self.ok
        front = []
        for r in rows:
            dominated = False
            for other in rows:
                if other is r:
                    continue
                le = all(s * other[m] <= s * r[m] for m, s in goals)
                lt = any(s * other[m] < s * r[m] for m, s in goals)
                if le and lt:
                    dominated = True
                    break
            if not dominated:
                front.append(r)
        return front

    def pivot(self, rows: str, cols: str, metric: str, *, missing: str = "—") -> str:
        """A two-axis view as a Markdown table: one row per value of
        parameter `rows`, one column per value of `cols`, `metric` in the
        cells (`missing` where the variant failed or was not run)."""
        row_values = _unique(r[rows] for r in self.rows)
        col_values = _unique(r[cols] for r in self.rows)
        table: dict[tuple, Any] = {}
        for r in self.rows:
            table.setdefault((r[rows], r[cols]), r.get(metric) if r.get("ok") else None)
        head = f"| {rows} \\ {cols} | " + " | ".join(_num(c) for c in col_values) + " |"
        out = [head, "|---|" + "---|" * len(col_values)]
        for rv in row_values:
            cells = []
            for cv in col_values:
                v = table.get((rv, cv))
                cells.append(_metric(v) if v is not None else missing)
            out.append(f"| {_num(rv)} | " + " | ".join(cells) + " |")
        return "\n".join(out) + "\n"

    # ---- rendering ---------------------------------------------------

    def _columns(self) -> list[str]:
        return list(self.params) + list(self.metrics) + ["ok", "error"]

    def to_csv(self) -> str:
        out = _io.StringIO()
        w = csv.writer(out, lineterminator="\n")
        cols = self._columns()
        w.writerow(cols)
        for r in self.rows:
            w.writerow([_csv_cell(r.get(c)) for c in cols])
        return out.getvalue()

    def to_markdown(self) -> str:
        """The rows as a Markdown table — parameters as written, metrics at
        table precision; the `ok` / `error` columns only when a row failed
        (CSV and JSON always carry them)."""
        cols = list(self.params) + list(self.metrics)
        if self.failed:
            cols += ["ok", "error"]
        lines = ["| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
        for r in self.rows:
            cells = []
            for c in cols:
                v = r.get(c)
                if c in self.params:
                    cells.append(_num(v))
                elif c == "error":
                    cells.append(v or "")
                elif c == "ok":
                    cells.append(str(bool(v)))
                else:
                    cells.append(_metric(v) if v is not None else "—")
            lines.append("| " + " | ".join(cells) + " |")
        return "\n".join(lines) + "\n"

    def to_json(self) -> str:
        return json.dumps({"params": self.params, "metrics": self.metrics, "rows": self.rows}, indent=2)

    def save(self, path: Union[str, Path], format: Optional[str] = None) -> None:
        path = Path(path)
        fmt = format or {".csv": "csv", ".md": "md", ".json": "json"}.get(path.suffix)
        if fmt == "csv":
            path.write_text(self.to_csv())
        elif fmt in ("md", "markdown"):
            path.write_text(self.to_markdown())
        elif fmt == "json":
            path.write_text(self.to_json())
        else:
            raise ValueError(f"Sweep.save: unknown format for {path.name!r} — use .csv, .md or .json (or format=)")

    def __repr__(self) -> str:
        return f"Sweep({len(self.rows)} rows, {len(self.failed)} failed, params={self.params}, metrics={self.metrics})"


def _unique(values: Iterable[Any]) -> list[Any]:
    seen: list[Any] = []
    for v in values:
        if v not in seen:
            seen.append(v)
    return seen


def _csv_cell(v: Any) -> Any:
    if v is None:
        return ""
    if isinstance(v, bool):
        return "true" if v else "false"
    return v


def _grid_points(grid: dict[str, Iterable[Any]]) -> list[Params]:
    names = list(grid)
    values = [list(v) for v in grid.values()]
    return [dict(zip(names, combo)) for combo in itertools.product(*values)]


def sweep(
    build: Callable[..., Any],
    grid: Optional[dict[str, Iterable[Any]]] = None,
    *,
    points: Optional[Iterable[Params]] = None,
    metrics: Optional[Callable[[Any], dict[str, Any]]] = None,
    sequence: Optional[str] = None,
    sequences: Optional[Sequence[str]] = None,
    max_duration: float = 120.0,
    workers: int = 1,
) -> Sweep:
    """Bakes `build(**params)` at every point of `grid` (the Cartesian
    product of the lists, in order — the last parameter varies fastest) or
    at the explicit `points`, and returns the table. `workers > 1` bakes in
    parallel processes — `build` and `metrics` must then be importable
    (module-level) functions; the rows still come back in grid order, so
    the result does not depend on scheduling."""
    if (grid is None) == (points is None):
        raise ValueError("sweep: pass exactly one of grid= or points=")
    pts = _grid_points(grid) if grid is not None else [dict(p) for p in points]
    if not pts:
        raise ValueError("sweep: the grid is empty")
    names = list(pts[0].keys())
    for p in pts:
        if list(p.keys()) != names:
            raise ValueError("sweep: every point must name the same parameters, in the same order")
    jobs = [(build, p, metrics, sequence, sequences, max_duration) for p in pts]
    if workers and workers > 1:
        # Spawned (not forked) workers: the extension already runs native
        # threads in the parent, and forking such a process is what hangs.
        ctx = multiprocessing.get_context("spawn")
        with ProcessPoolExecutor(max_workers=workers, mp_context=ctx, initializer=_worker_init,
                                 initargs=(list(sys.path),)) as pool:
            rows = list(pool.map(_evaluate_star, jobs))
    else:
        rows = [_evaluate_star(j) for j in jobs]
    metric_names = _unique(k for r in rows if r.get("ok") for k in r if k not in names and k not in ("ok", "error"))
    return Sweep(params=names, metrics=metric_names, rows=rows)


# -------------------------------------------------------------- optimize


@dataclass
class Optimum:
    """What `optimize` found: the best feasible parameters and their row,
    every evaluated row (a `Sweep`, in evaluation order), and how it got
    there."""

    params: Optional[Params]
    row: Optional[Row]
    evaluated: Sweep
    method: str
    iterations: int
    objective: str

    @property
    def ok(self) -> bool:
        return self.row is not None

    def to_json(self) -> str:
        return json.dumps(
            {"ok": self.ok, "method": self.method, "objective": self.objective, "iterations": self.iterations,
             "params": self.params, "row": self.row, "evaluated": len(self.evaluated)},
            indent=2,
        )

    def __repr__(self) -> str:
        if not self.ok:
            return f"Optimum(none feasible, {len(self.evaluated)} evaluated, method={self.method!r})"
        return f"Optimum({self.params} → {self.objective}={_metric(self.row.get(self.objective))}, {len(self.evaluated)} evaluated, method={self.method!r})"


def _axis(name: str, spec: Any) -> list[Any]:
    """The values of one axis: an explicit list, or `(lo, hi, step)`."""
    if isinstance(spec, tuple) and len(spec) == 3 and all(isinstance(v, (int, float)) for v in spec):
        lo, hi, step = spec
        if step <= 0 or hi < lo:
            raise ValueError(f"optimize: axis {name!r} needs (lo, hi, step) with step > 0 and hi >= lo")
        n = int(round((hi - lo) / step))
        values = [round(lo + i * step, 10) for i in range(n + 1)]
        # Snap the last value onto `hi` when the step does not divide.
        if values[-1] < hi - 1e-9:
            values.append(hi)
        return values
    values = list(spec)
    if not values:
        raise ValueError(f"optimize: axis {name!r} is empty")
    return values


def _constraint_fn(constraints) -> Callable[[Row], bool]:
    if constraints is None:
        return lambda r: True
    if callable(constraints):
        return constraints
    ops = {
        ">=": lambda a, b: a >= b, ">": lambda a, b: a > b, "<=": lambda a, b: a <= b,
        "<": lambda a, b: a < b, "==": lambda a, b: a == b, "!=": lambda a, b: a != b,
    }
    checks = []
    for metric, (op, bound) in constraints.items():
        if op not in ops:
            raise ValueError(f"optimize: unknown constraint operator {op!r} on {metric!r}")
        checks.append((metric, ops[op], bound))

    def feasible(r: Row) -> bool:
        return all(m in r and fn(r[m], b) for m, fn, b in checks)

    return feasible


def optimize(
    build: Callable[..., Any],
    space: dict[str, Any],
    *,
    objective: Union[str, Callable[[Row], float]] = "cycle",
    minimize: bool = True,
    constraints: Union[None, dict[str, tuple[str, float]], Callable[[Row], bool]] = None,
    metrics: Optional[Callable[[Any], dict[str, Any]]] = None,
    sequence: Optional[str] = None,
    sequences: Optional[Sequence[str]] = None,
    max_duration: float = 120.0,
    method: str = "grid",
    start: Optional[Params] = None,
    max_evals: int = 500,
    workers: int = 1,
) -> Optimum:
    """Searches `space` — `{name: [values]}` or `{name: (lo, hi, step)}` —
    for the feasible point with the best `objective` (a metric name, or a
    callable over the row). Two deterministic methods:

    * `"grid"` — every point, in grid order, then the best feasible one
      (ties to the earlier point). Exhaustive and parallel (`workers`).
    * `"descent"` — coordinate descent on the grid: from `start` (default:
      the grid's middle point), try each parameter one step down and one
      step up, move to the best feasible improvement, repeat until nothing
      improves or `max_evals` is spent. Far fewer bakes on a large space;
      finds a local optimum, and says so in `method`. Infeasible neighbours
      are stepped over, not into.

    Both return every evaluated row, so the search itself is a table."""
    axes = {name: _axis(name, spec) for name, spec in space.items()}
    names = list(axes)
    feasible = _constraint_fn(constraints)
    score = (lambda r: r[objective]) if isinstance(objective, str) else objective
    sign = 1.0 if minimize else -1.0
    objective_name = objective if isinstance(objective, str) else getattr(objective, "__name__", "objective")

    cache: dict[tuple, Row] = {}
    order: list[Row] = []

    def evaluate_at(index: tuple[int, ...]) -> Row:
        if index in cache:
            return cache[index]
        params = {n: axes[n][i] for n, i in zip(names, index)}
        row = evaluate(build, params, metrics=metrics, sequence=sequence, sequences=sequences, max_duration=max_duration)
        cache[index] = row
        order.append(row)
        return row

    def value(row: Row) -> Optional[float]:
        if not row.get("ok") or not feasible(row):
            return None
        try:
            return sign * float(score(row))
        except (KeyError, TypeError, ValueError):
            return None

    metric_names: list[str] = []
    if method == "grid":
        result = sweep(build, grid=axes, metrics=metrics, sequence=sequence, sequences=sequences,
                       max_duration=max_duration, workers=workers)
        best_row = None
        best_value = None
        for r in result.rows:
            v = value(r)
            if v is not None and (best_value is None or v < best_value):
                best_row, best_value = r, v
        params = {n: best_row[n] for n in names} if best_row else None
        return Optimum(params=params, row=best_row, evaluated=result, method="grid", iterations=1,
                       objective=objective_name)

    if method != "descent":
        raise ValueError(f"optimize: unknown method {method!r} — use 'grid' or 'descent'")

    # ---- coordinate descent ------------------------------------------
    if start is not None:
        index = tuple(_nearest_index(axes[n], start[n]) for n in names)
    else:
        index = tuple(len(axes[n]) // 2 for n in names)
    current = evaluate_at(index)
    current_value = value(current)
    iterations = 0
    # An infeasible start: walk the grid in order until something is
    # feasible (bounded by max_evals), then descend from there.
    if current_value is None:
        for combo in itertools.product(*[range(len(axes[n])) for n in names]):
            if len(order) >= max_evals:
                break
            row = evaluate_at(combo)
            v = value(row)
            if v is not None:
                index, current, current_value = combo, row, v
                break
    if current_value is not None:
        improved = True
        while improved and len(order) < max_evals:
            improved = False
            iterations += 1
            best_move = None
            for k, n in enumerate(names):
                for delta in (-1, 1):
                    j = index[k] + delta
                    if j < 0 or j >= len(axes[n]):
                        continue
                    trial = index[:k] + (j,) + index[k + 1:]
                    if len(order) >= max_evals and trial not in cache:
                        continue
                    row = evaluate_at(trial)
                    v = value(row)
                    if v is not None and v < current_value - 1e-12 and (best_move is None or v < best_move[0]):
                        best_move = (v, trial, row)
            if best_move is not None:
                current_value, index, current = best_move
                improved = True
    metric_names = _unique(k for r in order if r.get("ok") for k in r if k not in names and k not in ("ok", "error"))
    evaluated = Sweep(params=names, metrics=metric_names, rows=list(order))
    if current_value is None:
        return Optimum(params=None, row=None, evaluated=evaluated, method="descent", iterations=iterations,
                       objective=objective_name)
    return Optimum(params={n: current[n] for n in names}, row=current, evaluated=evaluated, method="descent",
                   iterations=iterations, objective=objective_name)


def _nearest_index(values: list[Any], target: Any) -> int:
    if target in values:
        return values.index(target)
    try:
        return min(range(len(values)), key=lambda i: abs(float(values[i]) - float(target)))
    except (TypeError, ValueError):
        raise ValueError(f"optimize: start value {target!r} is not on the axis {values!r}") from None


__all__ = ["Sweep", "Optimum", "sweep", "optimize", "evaluate"]
