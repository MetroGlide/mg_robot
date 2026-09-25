from fastapi import APIRouter, Depends

from mg_system_manager.config import SERVICE_LAYERS, SERVICES
from mg_system_manager.dependencies import get_runner
from mg_system_manager.docker_ops import ComposeRunner
from mg_system_manager.responses import result

router = APIRouter()


@router.get("/status")
def get_status(runner: ComposeRunner = Depends(get_runner)):
    return runner.get_status()


@router.get("/services")
def list_services():
    return {
        "layers": [{"id": id_, "label": label} for id_, label in SERVICE_LAYERS],
        "services": [
            {
                "key": spec.key,
                "label": spec.label,
                "layer": spec.layer,
                "hardware": spec.hardware,
            }
            for spec in SERVICES
        ],
    }


def _add_routes(service: str) -> None:
    name = service.replace("-", "_")

    def start(runner: ComposeRunner = Depends(get_runner)):
        return result(*runner.up(service))

    def stop(runner: ComposeRunner = Depends(get_runner)):
        return result(*runner.stop(service))

    def restart(runner: ComposeRunner = Depends(get_runner)):
        return result(*runner.restart(service))

    for action, handler in (("start", start), ("stop", stop),
                            ("restart", restart)):
        handler.__name__ = f"{action}_{name}"
        router.add_api_route(
            f"/{service}/{action}", handler, methods=["POST"])


for _spec in SERVICES:
    _add_routes(_spec.key)
