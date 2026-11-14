"""自己位置推定の評価で注入する故障の定義 (YAML) を読み込む共通モジュール。

評価ツール (eval_localization.py) と故障注入ノード (fault_injector.py) の両方が使う。
時刻はすべて再生開始からの経過秒で表す。

例:
  faults:
    - {type: kidnap,    at: 120.0, offset: {x: 3.0, y: 0.0, yaw: 0.5}}
    - {type: gnss_bias, start: 200.0, end: 230.0, bias: {x: 2.0, y: 0.0}}
    - {type: gnss_drop, start: 300.0, end: 360.0}
    - {type: odom_scale, start: 400.0, end: 420.0, v: 1.0, w: 1.3}
    - {type: scan_drop, start: 500.0, end: 520.0, sector_deg: [-90.0, 90.0]}
    - {type: amcl_drift, start: 600.0, end: 660.0, rate: {x: 0.05, y: 0.0, yaw: 0.0}}
    - {type: scan_thin, start: 0.0, end: 100.0, keep_every: 4}
    - {type: scan_range, start: 0.0, end: 100.0, max_range: 8.0}
    - {type: gnss_noise, start: 0.0, end: 100.0, sigma: 1.0, tau: 30.0}
    - {type: scan_jitter, start: 0.0, end: 100.0, ratio: 0.3, depth_m: 0.8}

scan_jitter は、毎回のスキャンで ratio の割合のビームを選び、半分は欠測に、残りは距離を 0〜depth_m 伸ばす
(植栽の葉が揺れて、手前と奥の当たりが入れ替わる状況)。
gnss_noise は、東・北に標準偏差 sigma [m] の雑音を足す。tau [s] (省略で 0) を与えると、偏りが tau 程度続く
1 次のガウス・マルコフ過程にする (単独測位の誤差の形)。0 なら毎回独立。
    - {type: odom_scale, start: 0.0, end: 100000.0, v: 1.0, w: 1.15, min_abs_w: 0.15}

区間で作用する故障は、min_abs_w [rad/s] を指定すると、ロボットの旋回の速さがそれ以上の間だけ作用する
(時刻を決めずに角へ注入できる。再生と Gazebo で同じ定義を使える)。

amcl_drift は AMCL を少しずつずらす (rate は 1 秒あたりの地図座標でのずれ)。再生評価では interval 秒
(既定 1.0) ごとに、AMCL の推定を rate * interval だけずらした姿勢を /initialpose に送る。
オフライン評価 (eval_localization_offline.py) では、AMCL の標本を直接ずらす。

gnss_accuracy は、GNSS が申告する水平の精度を std_m [m] に書き換える (NavSatFix の position_covariance と
NavPVT の h_acc)。位置は変えないので、gnss_bias や gnss_noise と組み合わせて「良い精度を申告したまま誤る」
「誤差と申告がともに悪くなる」を作る。
    - {type: gnss_accuracy, start: 0.0, end: 100.0, std_m: 0.5}

gnss_bias_field は、場所で変わる GNSS のずれ (地図の歪みや、地図と GNSS の手合わせの誤差) を作る。points の
地図座標 [x, y] に置いたずれ [bx, by] (東, 北) [m] を、距離 length_m [m] (省略で 10) のガウスの重みで
なめらかに補間する。場所は、ずらす前の GNSS の位置を anchor (地図の原点の緯度経度) からの東・北の距離に直して決める
(地図の座標が東・北に揃っているとみなす)。
    - {type: gnss_bias_field, start: 0.0, end: 100000.0, anchor: {latitude: 35.0, longitude: 139.0},
       length_m: 8.0, points: [[0.0, 0.0, 1.5, -1.0], [10.0, -20.0, -2.5, 0.5]]}

同じ種類の故障が同時に作用するときは、定義の順で最初のものだけを使う (gnss_bias と gnss_bias_field は別の種類
なので足し合わせる)。trigger の故障で常時の故障を一時的に置き換えるときは、trigger の故障を先に書く。

区間の故障は、start と end の代わりに trigger: <名前> を与えると、時刻ではなく外からの指示で入れ切りする
(Gazebo のシナリオで、ロボットが指定の場所に来たときに入れる)。trigger は allow_triggers を指定して
読み込んだときだけ使える。再生評価は時刻でしか入れられないので、trigger があれば読み込みで失敗させる。
    - {type: gnss_bias, trigger: building_east, bias: {x: 3.0, y: 0.0}}
"""

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import yaml

# 種類ごとの必須パラメータ。kidnap は瞬間的、それ以外は区間 [start, end] で作用する
REQUIRED_PARAMS: Dict[str, List[str]] = {
    "kidnap": ["offset"],
    "gnss_bias": ["bias"],
    "gnss_drop": [],
    "odom_scale": ["v", "w"],
    "scan_drop": ["sector_deg"],
    "amcl_drift": ["rate"],
    "scan_thin": ["keep_every"],
    "scan_range": ["max_range"],
    "gnss_noise": ["sigma"],
    "scan_jitter": ["ratio", "depth_m"],
    "gnss_accuracy": ["std_m"],
    "gnss_bias_field": ["anchor", "points"],
}
INSTANT_TYPES = {"kidnap"}


@dataclass
class Fault:
    type: str
    start: float
    end: float
    params: Dict[str, Any] = field(default_factory=dict)
    # 外からの指示で入れ切りする故障の名前 (時刻で入れる故障は None)
    trigger: Optional[str] = None

    @property
    def is_instant(self) -> bool:
        return self.type in INSTANT_TYPES


def parse_faults(data: Dict[str, Any], allow_triggers: bool = False) -> List[Fault]:
    """辞書 ({"faults": [...]}) から Fault のリスト (開始時刻順) を作る。"""
    faults: List[Fault] = []
    for i, raw in enumerate(data.get("faults", [])):
        ftype = raw.get("type")
        if ftype not in REQUIRED_PARAMS:
            raise ValueError(f"faults[{i}]: 未知の type です: {ftype}")
        params = {k: v for k, v in raw.items() if k not in ("type", "at", "start", "end", "trigger")}
        for key in REQUIRED_PARAMS[ftype]:
            if key not in params:
                raise ValueError(f"faults[{i}] ({ftype}): 必須パラメータ '{key}' がありません")
        if "trigger" in raw:
            if not allow_triggers:
                raise ValueError(f"faults[{i}] ({ftype}): trigger はこの用途では使えません (時刻で指定してください)")
            if ftype in INSTANT_TYPES or "start" in raw or "end" in raw:
                raise ValueError(f"faults[{i}] ({ftype}): trigger は区間の故障に、start・end の代わりに与えます")
            faults.append(Fault(type=ftype, start=0.0, end=math.inf, params=params, trigger=str(raw["trigger"])))
            continue
        if ftype in INSTANT_TYPES:
            if "at" not in raw:
                raise ValueError(f"faults[{i}] ({ftype}): 'at' が必要です")
            start = end = float(raw["at"])
        else:
            if "start" not in raw or "end" not in raw:
                raise ValueError(f"faults[{i}] ({ftype}): 'start' と 'end' が必要です")
            start, end = float(raw["start"]), float(raw["end"])
            if end <= start:
                raise ValueError(f"faults[{i}] ({ftype}): end は start より後にしてください")
        faults.append(Fault(type=ftype, start=start, end=end, params=params))
    faults.sort(key=lambda f: f.start)
    return faults


def load_faults(path: str, allow_triggers: bool = False) -> List[Fault]:
    with open(path, "r", encoding="utf-8") as f:
        return parse_faults(yaml.safe_load(f) or {}, allow_triggers)
