from typing import Any

from fastapi import APIRouter, Body, Depends

from mg_system_manager.dependencies import get_settings_store
from mg_system_manager.settings_store import SettingsStore

router = APIRouter()


@router.get("/settings")
def get_settings(store: SettingsStore = Depends(get_settings_store)):
    return store.load()


@router.patch("/settings")
def patch_settings(
    patch: dict[str, Any] = Body(...),
    store: SettingsStore = Depends(get_settings_store),
):
    """指定したキーだけを更新する。値が null のキーは削除する。"""
    store.merge(patch)
    return {"success": True, "message": ""}
