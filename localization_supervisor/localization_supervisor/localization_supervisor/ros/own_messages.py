"""監督自身が送ったメッセージの目印 (ROS に依存しない)。

監督は、推定器を初期化し直すメッセージ (/initialpose など) を送るとともに、同じトピックを
「外からの再初期化」として購読する。自分が送ったメッセージを外からのものと取り違えないよう、
送るときにトピックとヘッダの時刻を覚え、受け取ったときに照らし合わせる。
"""
from collections import OrderedDict
from typing import Tuple

Key = Tuple[str, int, int]


class OwnMessages:
    def __init__(self, capacity: int = 64) -> None:
        self.capacity = capacity
        self._keys: 'OrderedDict[Key, None]' = OrderedDict()

    def add(self, topic: str, sec: int, nanosec: int) -> None:
        self._keys[(topic, sec, nanosec)] = None
        while len(self._keys) > self.capacity:
            self._keys.popitem(last=False)

    def consume(self, topic: str, sec: int, nanosec: int) -> bool:
        """自分が送ったメッセージなら、目印を消して True を返す。"""
        key = (topic, sec, nanosec)
        if key in self._keys:
            del self._keys[key]
            return True
        return False
