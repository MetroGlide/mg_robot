"""HTTP エンドポイント一覧の契約テスト。

Web UI が呼ぶ API を意図せず変えないためのスナップショット。API を変更したときは、
Web UI の呼び出し側(frontend/src)も合わせて直し、routes.json を更新する。
更新するには UPDATE_ROUTES_SNAPSHOT=1 を付けてこのテストを実行する。
WebSocket(/logs/stream)は OpenAPI に含まれないため、別のテストで接続を確認している。
"""
import json
import os
from pathlib import Path

SNAPSHOT = Path(__file__).with_name("routes.json")


def _routes(app) -> dict[str, list[str]]:
    paths = app.openapi()["paths"]
    return {
        path: sorted(method.upper() for method in methods)
        for path, methods in sorted(paths.items())
    }


def test_routes_match_snapshot(client):
    routes = _routes(client.app)

    if os.environ.get("UPDATE_ROUTES_SNAPSHOT"):
        SNAPSHOT.write_text(
            json.dumps(routes, indent=1) + "\n", encoding="utf-8")

    assert routes == json.loads(SNAPSHOT.read_text(encoding="utf-8"))
