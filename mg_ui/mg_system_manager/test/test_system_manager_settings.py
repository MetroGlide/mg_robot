def test_settings_empty_by_default(client):
    assert client.get("/settings").json() == {}


def test_settings_roundtrip(client):
    body = client.post("/settings", json={"teleop": {"maxLinear": 0.3}}).json()

    assert body == {"success": True, "message": ""}
    assert client.get("/settings").json() == {"teleop": {"maxLinear": 0.3}}


def test_cors_allows_configured_origin(client):
    response = client.get(
        "/status", headers={"Origin": "http://localhost:8080"})

    assert response.headers["access-control-allow-origin"] == (
        "http://localhost:8080")
