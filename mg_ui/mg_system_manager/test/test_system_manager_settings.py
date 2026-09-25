import pytest
from starlette.websockets import WebSocketDisconnect


def test_settings_empty_by_default(client):
    assert client.get("/settings").json() == {}


def test_settings_roundtrip(client):
    body = client.post("/settings", json={"teleop": {"maxLinear": 0.3}}).json()

    assert body == {"success": True, "message": ""}
    assert client.get("/settings").json() == {"teleop": {"maxLinear": 0.3}}


@pytest.mark.parametrize("origin", [
    "http://localhost:8080",
    "http://192.168.1.20:8080",
    "http://10.0.0.5:5173",
    "http://172.20.1.1:8080",
    "http://100.101.102.103:8080",
    "http://tablet.local:8080",
])
def test_cors_allows_local_network_origins(client, origin):
    response = client.get("/status", headers={"Origin": origin})

    assert response.headers["access-control-allow-origin"] == origin


@pytest.mark.parametrize("origin", [
    "http://example.com:8080",
    "http://192.168.1.20:9000",
    "http://172.32.0.1:8080",
    "https://192.168.1.20:8080",
    "http://192.168.1.20:8080.evil.com",
])
def test_cors_rejects_other_origins(client, origin):
    response = client.get("/status", headers={"Origin": origin})

    assert "access-control-allow-origin" not in response.headers


def test_cors_allows_explicitly_configured_origin(client, settings):
    assert settings.is_origin_allowed("http://localhost:8080")
    assert not settings.is_origin_allowed("http://example.com:3000")


def test_logs_websocket_rejects_foreign_origin(client):
    with pytest.raises(WebSocketDisconnect) as excinfo:
        with client.websocket_connect(
                "/logs/stream", headers={"origin": "http://example.com:8080"}):
            pass

    assert excinfo.value.code == 1008


def test_logs_websocket_accepts_lan_origin(client):
    with client.websocket_connect(
            "/logs/stream", headers={"origin": "http://192.168.1.20:8080"}) as ws:
        ws.send_text('{"services": []}')
