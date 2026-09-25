import json
import logging
from pathlib import Path

logger = logging.getLogger(__name__)


class SettingsStore:
    """UI 設定を JSON ファイルに保存する。"""

    def __init__(self, directory: Path) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        self._file = directory / "ui_settings.json"

    def load(self) -> dict:
        if not self._file.exists():
            return {}
        try:
            return json.loads(self._file.read_text(encoding="utf-8"))
        except Exception as e:
            logger.error("settings load failed: %s", e)
            return {}

    def save(self, body: dict) -> None:
        tmp = self._file.with_suffix(".tmp")
        tmp.write_text(json.dumps(body, ensure_ascii=False), encoding="utf-8")
        tmp.rename(self._file)
