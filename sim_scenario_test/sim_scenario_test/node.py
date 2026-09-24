from __future__ import annotations

import json
import os
import threading
import traceback
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from sim_scenario_test.context import Clock, ScenarioContext
from sim_scenario_test.engine.result import (
    EXIT_CODES,
    CheckResult,
    Outcome,
    ResultStatus,
    ScenarioResult,
)
from sim_scenario_test.engine.runner import ScenarioEngine
from sim_scenario_test.loader import load_scenario
from sim_scenario_test.nav2 import Nav2Interface
from sim_scenario_test.poses import PoseResolver, TfRobotPoseSource
from sim_scenario_test.registry import DEFAULT_REGISTRY


class ScenarioRunnerNode(Node):
    """シナリオを 1 本実行して終了するノード。"""

    def __init__(self):
        super().__init__("scenario_runner")
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("profile", "")
        self.declare_parameter("result_file", "")
        # 負の値ならシナリオの seed を使う
        self.declare_parameter("seed", -1)

        scenario_file = self.get_parameter("scenario_file").value
        if not scenario_file:
            raise RuntimeError("parameter 'scenario_file' is required")
        self._result_file = self.get_parameter("result_file").value
        self._result_pub = self.create_publisher(String, "~/result", 10)
        self._done = threading.Event()
        self._result: Optional[ScenarioResult] = None

        scenario, profile = load_scenario(
            scenario_file, self.get_parameter("profile").value)
        self.get_logger().info(f"loaded scenario '{scenario.name}' from {scenario_file}")

        seed = self.get_parameter("seed").value
        world_vars = profile.world_vars(scenario.world, "scenario.world")
        backend_cls = DEFAULT_REGISTRY.get("backend", profile.sim.backend, "profile.sim").impl
        poses = PoseResolver(
            profile.frames.map_in_world.as_pose(),
            TfRobotPoseSource(self, profile.frames.map, profile.frames.base),
        )
        self._ctx = ScenarioContext(
            node=self,
            registry=DEFAULT_REGISTRY,
            scenario=scenario,
            profile=profile,
            clock=Clock(self),
            backend=backend_cls(self, world_vars["sim_world"]),
            poses=poses,
            nav2=Nav2Interface(self, profile.nav2, profile.frames),
            seed=seed if seed >= 0 else None,
        )
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @property
    def done(self) -> threading.Event:
        return self._done

    @property
    def result(self) -> Optional[ScenarioResult]:
        return self._result

    def _run(self) -> None:
        try:
            self._result = ScenarioEngine(self._ctx).execute()
        except Exception:
            message = traceback.format_exc()
            self.get_logger().fatal(f"unexpected exception:\n{message}")
            self._result = ScenarioResult(
                name=self._ctx.scenario.name,
                status=ResultStatus.ERROR,
                checks=[CheckResult("execution", ResultStatus.ERROR, message)],
                outcome=Outcome(),
            )
        payload = json.dumps(self._result.to_dict(), ensure_ascii=False, indent=2)
        msg = String()
        msg.data = payload
        self._result_pub.publish(msg)
        if self._result_file:
            os.makedirs(os.path.dirname(os.path.abspath(self._result_file)), exist_ok=True)
            with open(self._result_file, "w") as f:
                f.write(payload)
            self.get_logger().info(f"result written to {self._result_file}")
        self._done.set()


def main() -> int:
    rclpy.init()
    exit_code = EXIT_CODES[ResultStatus.ERROR]
    node = None
    executor = MultiThreadedExecutor()
    try:
        node = ScenarioRunnerNode()
        executor.add_node(node)
        while rclpy.ok() and not node.done.is_set():
            executor.spin_once(timeout_sec=0.1)
        if node.result is not None:
            exit_code = EXIT_CODES[node.result.status]
    except KeyboardInterrupt:
        pass
    except Exception:
        traceback.print_exc()
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code
