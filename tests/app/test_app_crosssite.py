"""Layer (a): cross-site protection (contracts.md 6.1): JSON-only POSTs, Origin and Host checks,
no CORS. A page on another site cannot drive the dog through the user's browser."""
from __future__ import annotations

import json

import pytest

from apphelp import RawStream


def raw(h, method, path, host="", headers=(), body=b""):
    r = RawStream(h.port, path, host=host, method=method, headers=headers, body=body)
    try:
        is_json = r.headers.get("content-type", "").startswith("application/json")
        return r.status, r.headers, (r.body_json() if is_json and method != "HEAD" else None)
    finally:
        r.close()


ARM = json.dumps({"armed": True}).encode()


@pytest.mark.parametrize("ctype", [None, "text/plain", "text/plain;charset=UTF-8",
                                   "application/x-www-form-urlencoded", "multipart/form-data; boundary=x",
                                   "application/jsonx", "json"])
def test_post_needs_application_json(h, ctype):
    hdrs = [] if ctype is None else [("Content-Type", ctype)]
    s, _, b = raw(h, "POST", "/api/arm", headers=hdrs, body=ARM)
    assert s == 415 and b["ok"] is False and "JSON" in b["error"]
    assert h.app.armed is False


def test_post_json_with_charset_is_accepted(h):
    s, _, b = raw(h, "POST", "/api/arm", headers=[("Content-Type", "application/json; charset=utf-8")],
                  body=ARM)
    assert (s, b) == (200, {"ok": True, "armed": True})


def test_estop_also_needs_json(h):
    # ESTOP is never gated, but it is still a POST: a form on another site must not reach it either
    s, _, _ = raw(h, "POST", "/api/estop", headers=[("Content-Type", "text/plain")], body=b"{}")
    assert s == 415 and h.dog.estops == []


@pytest.mark.parametrize("origin", ["http://evil.example", "http://127.0.0.1:1", "https://127.0.0.1:{port}",
                                    "http://localhost.evil:{port}", "null", "http://192.168.137.1:{port}",
                                    "http://127.0.0.1:{port}.evil", ""])
def test_foreign_origin_is_refused(h, origin):
    o = origin.format(port=h.port)
    s, _, b = raw(h, "POST", "/api/arm", headers=[("Content-Type", "application/json"), ("Origin", o)], body=ARM)
    assert s == 403 and b["ok"] is False and "origine" in b["error"]
    assert h.app.armed is False


@pytest.mark.parametrize("origin", ["http://127.0.0.1:{port}", "http://localhost:{port}", None])
def test_own_origin_or_none_is_accepted(h, origin):
    hdrs = [("Content-Type", "application/json")]
    if origin is not None:
        hdrs.append(("Origin", origin.format(port=h.port)))
    s, _, b = raw(h, "POST", "/api/arm", headers=hdrs, body=ARM)
    assert (s, b) == (200, {"ok": True, "armed": True})


BAD_HOSTS = ["evil.example", "evil.example:{port}", "127.0.0.1", "127.0.0.1:1", "localhost",
             "192.168.137.1:{port}", "[::1]:{port}", "127.0.0.1:{port}.evil", None]


@pytest.mark.parametrize("host", BAD_HOSTS)
def test_foreign_host_refused_on_post(h, host):
    hv = None if host is None else host.format(port=h.port)
    s, _, b = raw(h, "POST", "/api/arm", host=hv, headers=[("Content-Type", "application/json")], body=ARM)
    assert s == 403 and b["ok"] is False and "host" in b["error"]
    assert h.app.armed is False


@pytest.mark.parametrize("path", ["/api/events", "/stream.mjpg", "/api/state", "/", "/api/catalog", "/api/map"])
@pytest.mark.parametrize("host", ["evil.example:{port}", None])
def test_foreign_host_refused_on_get(h, path, host):
    # DNS rebinding: a page on evil.example resolved to 127.0.0.1 sends Host: evil.example
    h.post("/api/camera", {"on": True})
    h.arm()
    hv = None if host is None else host.format(port=h.port)
    s, hdrs, b = raw(h, "GET", path, host=hv)
    assert s == 403 and b["ok"] is False
    assert h.app.armed is True          # GET / with a foreign Host does not even reach the disarm
    assert h.app.hub.count() == 0 and h.app.camera.viewers == 0


@pytest.mark.parametrize("host", ["127.0.0.1:{port}", "localhost:{port}", "LOCALHOST:{port}"])
def test_own_host_accepted(h, host):
    s, _, b = raw(h, "GET", "/api/state", host=host.format(port=h.port))
    assert s == 200 and b["target"] == "stub:50052"


def test_no_cors_headers_anywhere(h):
    o = [("Origin", "http://evil.example")]
    replies = [raw(h, "GET", "/api/state", headers=o), raw(h, "GET", "/", headers=o),
               raw(h, "POST", "/api/arm", headers=o + [("Content-Type", "application/json")], body=ARM),
               raw(h, "POST", "/api/arm", headers=[("Content-Type", "application/json")], body=ARM),
               raw(h, "OPTIONS", "/api/arm", headers=o + [("Access-Control-Request-Method", "POST")])]
    for s, hdrs, _ in replies:
        assert not [k for k in hdrs if k.startswith("access-control-")], hdrs
    assert replies[-1][0] == 405        # no preflight answer: the browser blocks the cross-site POST


def test_other_methods_405(h):
    for m in ("PUT", "DELETE", "PATCH"):
        s, hdrs, b = raw(h, m, "/api/arm", headers=[("Content-Type", "application/json")], body=ARM)
        assert s == 405 and hdrs["allow"] == "POST" and b["ok"] is False
    s, hdrs, _ = raw(h, "HEAD", "/")
    assert s == 405 and hdrs["allow"] == "GET"
    assert h.app.armed is False


def test_ui_page_cannot_be_framed(h):
    s, hdrs, _ = raw(h, "GET", "/")
    assert s == 200 and hdrs["x-frame-options"] == "DENY"
    assert "frame-ancestors 'none'" in hdrs["content-security-policy"]
