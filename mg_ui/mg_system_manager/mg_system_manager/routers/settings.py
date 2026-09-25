from fastapi import APIRouter, Depends, Request

from mg_system_manager.dependencies import get_settings_store
from mg_system_manager.settings_store import SettingsStore

router = APIRouter()


@router.get("/settings")
def get_settings(store: SettingsStore = Depends(get_settings_store)):
    return store.load()


@router.post("/settings")
async def post_settings(
    request: Request, store: SettingsStore = Depends(get_settings_store)
):
    try:
        store.save(await request.json())
    except Exception as e:
        return {"success": False, "message": str(e)}
    return {"success": True, "message": ""}
