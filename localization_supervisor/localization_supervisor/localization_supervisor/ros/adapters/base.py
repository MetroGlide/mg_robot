"""入力のアダプタの約束: ROS のメッセージから情報源の標本を作り、SampleStore に入れる。

ノード (購読) もオフライン評価 (rosbag の読み込み) も、同じアダプタを使う。
アダプタは購読するトピック (Subscription) を宣言し、届いたメッセージを handler で受け取る。
座標変換が必要なアダプタは、AdapterContext.lookup で TF を引く。
"""
from dataclasses import dataclass
from typing import Any, Callable, List, Mapping, Optional

from ...core.geometry import Pose2
from ...core.interfaces import ParamSpec, Plugin, SourceKind
from ...core.snapshot import SampleStore


@dataclass
class Subscription:
    topic: str
    msg_type: type
    handler: Callable[[Any], None]
    # 'default' | 'sensor' (best effort) | 'latched' (transient local。地図など)
    qos: str = 'default'


class AdapterContext:
    """アダプタが使う、TF を引く窓口。"""

    def lookup(self, parent: str, child: str, stamp: Optional[float] = None) -> Optional[Pose2]:
        raise NotImplementedError


class SourceAdapter(Plugin):
    """KINDS に、作れる情報源の種類を宣言する。"""
    KINDS: tuple = ()
    PARAMS: Mapping[str, ParamSpec] = {}

    def __init__(
        self, source: str, kind: SourceKind, params: Mapping[str, Any], store: SampleStore,
        context: AdapterContext,
    ) -> None:
        super().__init__(source, params)
        self.source = source
        self.kind = kind
        self.store = store
        self.context = context

    def subscriptions(self) -> List[Subscription]:
        raise NotImplementedError
