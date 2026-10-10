"""監視するトピックの設定 (YAML) の読み取り。ROS に依存しない。

watchdog と topic_monitor が同じ書き方で、監視するトピックを指定する。

    monitored_sources: [top_lidar]
    top_lidar:
      topic: scan_top_lidar              # 必須。相対名はノードの名前空間で解決される
      type: sensor_msgs/msg/LaserScan    # 必須。メッセージの型
      enabled: true
      expected_hz: 10.0                  # 期待する受信頻度。0 なら頻度は調べない
      stale_timeout_sec: 1.0             # この時間、途絶えたら異常 (省略時はノード共通の値)
      armed_on_first_message: false      # 初回のメッセージを受け取るまで監視しない
      qos: best_effort                   # best_effort / reliable
      ok_field: ''                       # 真偽値のフィールド名。false を受信したら異常 (例: data)
      fault_level: error                 # 異常のときの診断のレベル (error / warn)
"""
from dataclasses import dataclass
from typing import Any, Callable

from mg_utils.monitoring.health import FAULT_LEVELS

QOS_NAMES = ('best_effort', 'reliable')


@dataclass(frozen=True)
class TopicSourceConfig:
    id: str
    topic: str
    msg_type: str
    enabled: bool
    expected_hz: float
    stale_timeout_sec: float
    armed_on_first_message: bool
    qos: str
    ok_field: str
    fault_level: str

    @property
    def raw(self) -> bool:
        """メッセージをデシリアライズせずに受け取る (内容を見ないので負荷が小さい)"""
        return not self.ok_field


def parse_source_config(
    source_id: str,
    get: Callable[[str, Any], Any],
    default_stale_timeout_sec: float,
) -> TopicSourceConfig:
    """`<source_id>.<キー>` の値を get(キー, 既定値) で読み、検証する。

    必須の topic と type は、既定値を None として get に渡す (無ければ get が例外にする)。
    """
    topic = get('topic', None)
    msg_type = get('type', None)
    if not topic:
        raise ValueError(f'{source_id}.topic が指定されていない')
    if not msg_type:
        raise ValueError(f'{source_id}.type が指定されていない')
    qos = get('qos', 'reliable')
    if qos not in QOS_NAMES:
        raise ValueError(f'{source_id}.qos は {list(QOS_NAMES)} のいずれか: {qos}')
    fault_level = get('fault_level', 'error')
    if fault_level not in FAULT_LEVELS:
        raise ValueError(f'{source_id}.fault_level は {list(FAULT_LEVELS)} のいずれか: {fault_level}')
    expected_hz = float(get('expected_hz', 0.0))
    stale_timeout = float(get('stale_timeout_sec', default_stale_timeout_sec))
    if expected_hz > 0.0 and stale_timeout * expected_hz <= 1.0:
        raise ValueError(
            f'{source_id}.stale_timeout_sec ({stale_timeout}) は、期待する周期 '
            f'(1/{expected_hz} s) より長くする')
    return TopicSourceConfig(
        id=source_id,
        topic=topic,
        msg_type=msg_type,
        enabled=bool(get('enabled', True)),
        expected_hz=expected_hz,
        stale_timeout_sec=stale_timeout,
        armed_on_first_message=bool(get('armed_on_first_message', False)),
        qos=qos,
        ok_field=get('ok_field', ''),
        fault_level=fault_level,
    )
