import uvicorn

from mg_system_manager.app import create_app
from mg_system_manager.config import Settings

if __name__ == "__main__":
    uvicorn.run(create_app(Settings.from_env()), host="0.0.0.0", port=8001)
