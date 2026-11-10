"""シナリオ YAML (version 2.0) のデータモデル。"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Literal, Optional

from sim_scenario_test.spec import (
    ActionCall,
    DriverCall,
    ExpectationCall,
    MonitorCall,
    TriggerCall,
)
from sim_scenario_test.sim.model import ModelSpec


@dataclass
class ObstacleSpec:
    model: ModelSpec


@dataclass
class TimelineEntry:
    """when が成立した時点で do を並行実行する。until が成立すると実行中の do を打ち切る。

    required が true のまま一度も発火しなかった場合は ERROR とする
    (障害物が出ないまま PASS する偽陽性を防ぐため)。
    """
    when: TriggerCall
    do: List[ActionCall] = field(default_factory=list)
    until: Optional[TriggerCall] = None
    name: str = ""
    required: bool = True


@dataclass
class Scenario:
    version: Literal["2.0"]
    name: str
    world: str
    run: DriverCall
    profile: str = ""
    description: str = ""
    tags: List[str] = field(default_factory=list)
    # 走行開始からの sim 時間の上限 (秒)。超えると走行を打ち切り FAILED とする
    timeout: Optional[float] = None
    # 乱数 (spawn の jitter 等) の種。--seed / seed パラメータで上書きできる
    seed: int = 0
    # プロファイルの stack.args を、このシナリオ用に上書き・追加する launch 引数 (run / run-all のみ)。
    # 値は変数展開される (例: waypoints_load_path: "pkg://.../data/x.yaml")
    stack_args: Dict[str, str] = field(default_factory=dict)
    # プロファイルの sim.launch.args を、このシナリオ用に上書き・追加する launch 引数。
    # シミュレータを起動し直す実行でだけ使い、起動済みのシミュレータへの attach では使わない
    sim_args: Dict[str, str] = field(default_factory=dict)
    # 走行の間 rosbag に記録するトピック。結果のディレクトリの bag/ に入る (シミュレータを起動し直す実行でだけ)
    record_topics: List[str] = field(default_factory=list)
    obstacles: Dict[str, ObstacleSpec] = field(default_factory=dict)
    setup: List[ActionCall] = field(default_factory=list)
    timeline: List[TimelineEntry] = field(default_factory=list)
    monitors: List[MonitorCall] = field(default_factory=list)
    # 省略時はローダーが reached_all を補う
    expect: List[ExpectationCall] = field(default_factory=list)
    teardown: List[ActionCall] = field(default_factory=list)
