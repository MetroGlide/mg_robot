import logging

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from mg_system_manager.config import Settings
from mg_system_manager.docker_ops import ComposeRunner
from mg_system_manager.routers import (
    logs,
    maps,
    rosbag,
    scenario_stack,
    services,
    settings as settings_router,
    simulation,
)
from mg_system_manager.settings_store import SettingsStore

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s %(message)s",
)


def create_app(
    settings: Settings,
    runner: ComposeRunner | None = None,
    settings_store: SettingsStore | None = None,
) -> FastAPI:
    app = FastAPI()
    app.state.settings = settings
    app.state.runner = runner if runner is not None else ComposeRunner(settings)
    app.state.settings_store = (
        settings_store if settings_store is not None
        else SettingsStore(settings.settings_dir))

    app.add_middleware(
        CORSMiddleware,
        allow_origins=settings.allowed_origins,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    for module in (services, settings_router, maps, simulation,
                   scenario_stack, rosbag, logs):
        app.include_router(module.router)
    return app
