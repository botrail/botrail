"""The launch view belongs to the browser URL; malformed views never start a server."""

from types import SimpleNamespace
from urllib.parse import parse_qs, urlparse

import pytest
from botrail import _launcher


def test_launch_opens_and_prints_the_requested_view(monkeypatch, capsys):
    server = SimpleNamespace(url="http://127.0.0.1:12345")
    opened = []
    monkeypatch.setattr(_launcher, "_studio_dir", lambda: "/studio")
    monkeypatch.setattr(_launcher._core, "serve_studio", lambda *args: server)
    monkeypatch.setattr(_launcher.webbrowser, "open", opened.append)
    result = _launcher.studio(None, block=False, view=((5.9, 4.7, 4.0), (1.25, 0, 0.65)))
    assert result is server
    assert parse_qs(urlparse(opened[0]).query) == {"view": ["5.9,4.7,4.0,1.25,0.0,0.65"]}
    assert opened[0] in capsys.readouterr().out
    _launcher.studio(None, block=False)
    assert opened[-1] == server.url


@pytest.mark.parametrize("view", [
    ((1, 2), (0, 0, 0)), ((1, 2, 3),), ((1, 2, 3), (1, 2, 3)),
    ((float("nan"), 2, 3), (0, 0, 0)), ((1, 2, 3), (0, float("inf"), 0)),
])
def test_invalid_launch_view_is_rejected_before_startup(monkeypatch, view):
    def unexpected(*args):
        pytest.fail("invalid view started a server")

    monkeypatch.setattr(_launcher._core, "serve_studio", unexpected)
    with pytest.raises(ValueError, match="view needs"):
        _launcher.studio(None, block=False, view=view)
