from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Literal

from sim_scenario_test.errors import ScenarioValidationError


@dataclass
class ModelSpec:
    """シミュレータに配置するモデルの定義。

    type:
      fuel      : Gazebo Fuel のモデル (uri: "Owner/models/Name" または URL)
      local     : ローカルの SDF ファイル (path。pkg:// と変数展開に対応)
      primitive : 箱・円柱・球 (shape, size。z はモデル中心の高さ)
    static は fuel / primitive に適用する (local は SDF 内の定義に従う)。
    """
    type: Literal["fuel", "local", "primitive"]
    uri: str = ""
    path: str = ""
    shape: Literal["", "box", "cylinder", "sphere"] = ""
    size: Dict[str, float] = field(default_factory=dict)
    static: bool = True

    def validate(self, where: str) -> None:
        if self.type == "fuel" and not self.uri:
            raise ScenarioValidationError(f"{where}: 'uri' is required for fuel")
        if self.type == "local" and not self.path:
            raise ScenarioValidationError(f"{where}: 'path' is required for local")
        if self.type == "primitive" and not self.shape:
            raise ScenarioValidationError(f"{where}: 'shape' is required for primitive")
