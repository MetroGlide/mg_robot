"""標準の照合。設定の type に書く登録名と、クラスの対応。"""
from .correlative import CorrelativeMatcher

STANDARD_MATCHERS = {
    'correlative': CorrelativeMatcher,
}
