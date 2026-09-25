import json
import logging
import threading
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


class SettingsStore:
    """UI 設定を JSON ファイルに保存する。

    設定はトップレベルのキー単位で更新し、他のキーは変更しない。
    """

    def __init__(self, directory: Path) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        self._file = directory / "ui_settings.json"
        self._lock = threading.Lock()

    def load(self) -> dict:
        with self._lock:
            return self._read()

    def merge(self, patch: dict[str, Any]) -> None:
        """patch のキーだけを更新する。値が None のキーは削除する。"""
        with self._lock:
            data = self._read()
            for key, value in patch.items():
                if value is None:
                    data.pop(key, None)
                else:
                    data[key] = value
            self._write(data)

    def _read(self) -> dict:
        if not self._file.exists():
            return {}
        try:
            data = json.loads(self._file.read_text(encoding="utf-8"))
        except Exception as e:
            broken = self._file.with_suffix(".corrupt")
            logger.error("settings load failed: %s (moved to %s)", e, broken)
            self._file.replace(broken)
            return {}
        if not isinstance(data, dict):
            logger.error("settings file is not an object: %s", self._file)
            return {}
        return data

    def _write(self, data: dict) -> None:
        tmp = self._file.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
        tmp.replace(self._file)
