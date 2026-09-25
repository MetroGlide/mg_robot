from fastapi import Request

from mg_system_manager.config import Settings
from mg_system_manager.docker_ops import ComposeRunner
from mg_system_manager.settings_store import SettingsStore


def get_settings(request: Request) -> Settings:
    return request.app.state.settings


def get_runner(request: Request) -> ComposeRunner:
    return request.app.state.runner


def get_settings_store(request: Request) -> SettingsStore:
    return request.app.state.settings_store
