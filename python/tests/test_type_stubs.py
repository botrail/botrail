"""The hand-written type stub has to track the pyo3 bindings.

`_core.pyi` is all that IDEs and type checkers see, and it is maintained by
hand: a method added to `crates/botrail-py/src/lib.rs` stays invisible to
tooling until someone remembers the stub, and every other test still passes.
These checks compare the stub against the built extension (names *and*
parameter names) and against the Rust source, so a stale `.so` cannot hide a
missing entry either.
"""

import ast
import re
from pathlib import Path
from typing import Optional

import pytest

from botrail import _core

STUB = Path(_core.__file__).with_name("_core.pyi")
LIB_RS = Path(__file__).resolve().parents[2] / "crates" / "botrail-py" / "src" / "lib.rs"


def _stub_tree() -> ast.Module:
    return ast.parse(STUB.read_text(encoding="utf-8"))


def _stub_classes() -> dict:
    """`{class name: {member name: FunctionDef}}` for the stub's classes."""
    out = {}
    for node in _stub_tree().body:
        if isinstance(node, ast.ClassDef):
            out[node.name] = {
                m.name: m for m in node.body if isinstance(m, ast.FunctionDef)
            }
    return out


def _public(names) -> set:
    return {n for n in names if not n.startswith("__")}


def _arg_names(fn: ast.FunctionDef) -> list:
    """Declared parameter names, `self` dropped (stubs never see it in the
    runtime signature)."""
    a = fn.args
    names = [x.arg for x in [*a.posonlyargs, *a.args, *a.kwonlyargs]]
    if a.vararg:
        names.append("*" + a.vararg.arg)
    if a.kwarg:
        names.append("**" + a.kwarg.arg)
    return [n for n in names if n != "self"]


def _runtime_arg_names(text_signature: str) -> Optional[list]:
    """Parameter names out of a pyo3 `__text_signature__`, or None when it
    carries no information (`*args, **kwargs`)."""
    sig = text_signature.replace("$self", "self")
    fn = ast.parse(f"def _f{sig}: ...").body[0]
    names = _arg_names(fn)
    if any(n.startswith("*") for n in names):
        return None
    return names


def test_stub_covers_module_level_names() -> None:
    tree = _stub_tree()
    declared = {n.name for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef))}
    exported = _public(dir(_core))
    assert exported - declared == set(), "exported but missing from the stub"
    assert declared - exported - {"__version__"} == set(), "in the stub but not exported"


@pytest.mark.parametrize("cls_name", sorted(_stub_classes()))
def test_stub_members_match_runtime(cls_name: str) -> None:
    stub_members = _stub_classes()[cls_name]
    cls = getattr(_core, cls_name)
    assert _public(dir(cls)) == _public(stub_members), (
        f"{cls_name}: stub members drifted from the compiled class"
    )


@pytest.mark.parametrize("cls_name", sorted(_stub_classes()))
def test_stub_parameter_names_match_runtime(cls_name: str) -> None:
    cls = getattr(_core, cls_name)
    for name, fn in _stub_classes()[cls_name].items():
        # pyo3 hangs the constructor signature off the class, not __init__.
        holder = cls if name == "__init__" else getattr(cls, name, None)
        text_signature = getattr(holder, "__text_signature__", None)
        if text_signature is None:  # properties, and anything pyo3 leaves bare
            continue
        expected = _runtime_arg_names(text_signature)
        if expected is None:
            continue
        assert _arg_names(fn) == expected, f"{cls_name}.{name}: parameter names differ"


def test_stub_covers_rust_source() -> None:
    """Guards against a stale build: the stub is compared to what the pyo3
    source declares today, not just to whatever `.so` happens to be loaded."""
    if not LIB_RS.exists():  # running against an installed wheel
        pytest.skip("botrail-py sources not available")

    src = LIB_RS.read_text(encoding="utf-8")
    stub_classes = _stub_classes()
    missing = []
    for cls_name, body in re.findall(r"#\[pymethods\]\s*\nimpl (\w+)\s*\{(.*?)\n\}", src, re.S):
        declared = set(stub_classes.get(cls_name, {}))
        for fn in re.findall(r"^\s{4}fn (\w+)", body, re.M):
            if fn.startswith("__"):
                continue
            # `#[new]` surfaces as the constructor.
            name = "__init__" if fn == "new" else fn
            if name not in declared:
                missing.append(f"{cls_name}.{name}")
    assert missing == [], f"declared in lib.rs but absent from the stub: {missing}"
