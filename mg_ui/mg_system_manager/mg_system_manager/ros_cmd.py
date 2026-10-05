import logging
import os
import subprocess

logger = logging.getLogger(__name__)


def run_local_ros2_cmd(cmd: str, timeout: int = 30) -> tuple[bool, str]:
    """system-manager コンテナ内で ROS 2 コマンドを実行する。"""
    full_cmd = (
        "source /opt/ros/humble/setup.bash && "
        "source /root/ros2_ws/install/setup.bash && "
        f"{cmd}"
    )
    # 親プロセスの Python 環境変数をクリアして ROS 2 の Python 実行環境との衝突を防ぐ
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    env.pop("PYTHONHOME", None)

    try:
        result = subprocess.run(
            ["bash", "-c", full_cmd],
            capture_output=True,
            text=True,
            timeout=timeout,
            env=env,
        )
    except Exception as e:
        return False, str(e)
    output = result.stdout.strip() + "\n" + result.stderr.strip()
    return result.returncode == 0, output.strip()
