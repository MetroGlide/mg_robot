"""オフライン評価: rosbag を入力のアダプタで読み、Snapshot を作って、実時間を待たずに Supervisor を回す。

記録済みの推定器の出力を使うので、再初期化した後の挙動は見られない (それは再生評価で見る)。
Snapshot の段階で故障を注入して (core/faults.py)、正解の付いた事例を作り、判定のインスタンスごとの
検知率・誤検知率・検知までの時間を出す (core/evaluation.py)。

  ros2 run localization_supervisor offline_runner.py --config <設定> --bag <bag>[:トピック,...] \
      [--bag ...] [--map <reference の情報源>=<map.yaml>] [--faults <yaml>] [--profile <名前>] \
      [--start S] [--duration D] [--out <ディレクトリ>]

時刻は、最初の --bag の最初のメッセージからの経過秒 (故障の時刻もこれに合わせる)。
周期の時刻は bag の記録時刻 (再生評価の記録はシミュレーション時刻) で、標本の時刻はメッセージの header。
"""
import argparse
import heapq
import json
import math
import os
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Iterable, Iterator, List, Optional, Sequence, Tuple

import rosbag2_py
import tf2_ros
import yaml
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage

from ..core.config import SupervisorConfig, load_config
from ..core.evaluation import Report, TickRecord, evaluate, format_report
from ..core.faults import Fault, FaultInjectingStore, FaultInjector, parse_faults, validate_sources
from ..core.geometry import Pose2
from ..core.interfaces import SourceKind
from ..core.snapshot import SampleStore, Snapshot
from ..core.supervisor import StepResult, Supervisor, make_store
from .adapters import AdapterContext, build_adapters
from .adapters.standard import yaw_from_quaternion
from .map_io import load_map_yaml

TF_TOPICS = ('/tf', '/tf_static')


class BufferContext(AdapterContext):
    """bag の /tf・/tf_static を入れた TF のバッファ。"""

    def __init__(self) -> None:
        self.buffer = tf2_ros.Buffer()

    def add(self, msg: TFMessage, static: bool) -> None:
        for transform in msg.transforms:
            if static:
                self.buffer.set_transform_static(transform, 'offline')
            else:
                self.buffer.set_transform(transform, 'offline')

    def lookup(self, parent: str, child: str, stamp: Optional[float] = None) -> Optional[Pose2]:
        when = Time() if stamp is None else Time(nanoseconds=int(stamp * 1e9))
        try:
            t = self.buffer.lookup_transform(parent, child, when).transform
        except tf2_ros.TransformException:
            return None
        return (t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation))


@dataclass
class BagInput:
    path: str
    topics: Optional[List[str]] = None   # None なら、必要なトピックのすべて


@dataclass
class OfflineOptions:
    period_sec: float = 1.0
    start_sec: float = 0.0
    duration_sec: Optional[float] = None
    maps: Dict[str, str] = field(default_factory=dict)
    # 標本に注入する故障 (評価の正解にもなる)
    faults: List[Fault] = field(default_factory=list)
    # 正解としてだけ使う故障 (記録したときにすでに起きていたもの。注入しない)
    labels: List[Fault] = field(default_factory=list)


@dataclass
class OfflineResult:
    t0: float
    records: List[TickRecord]
    steps: List[Dict[str, Any]]
    events: List[Dict[str, Any]]
    report: Report


def _storage_id(path: str) -> str:
    meta = os.path.join(path, 'metadata.yaml')
    if os.path.isfile(meta):
        with open(meta, encoding='utf-8') as f:
            info = (yaml.safe_load(f) or {}).get('rosbag2_bagfile_information', {})
        if info.get('storage_identifier'):
            return info['storage_identifier']
    return 'mcap' if path.endswith('.mcap') or any(n.endswith('.mcap') for n in os.listdir(path)) else 'sqlite3'


def _open(path: str, topics: Sequence[str], seek_ns: Optional[int] = None) -> Tuple[Any, Dict[str, type]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=_storage_id(path)),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    available = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [t for t in topics if t in available]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    if seek_ns is not None:
        reader.seek(seek_ns)
    return reader, {t: get_message(available[t]) for t in wanted}


def _messages(path: str, topics: Sequence[str], seek_ns: Optional[int] = None) -> Iterator[Tuple[int, str, Any]]:
    reader, types = _open(path, topics, seek_ns)
    if not types:
        # 絞り込むトピックが空だと、rosbag2 はすべてのトピックを返すので、読まない
        return
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        yield t_ns, topic, deserialize_message(data, types[topic])


def _first_time_ns(path: str) -> int:
    reader, _ = _open(path, [])
    return reader.get_metadata().starting_time.nanoseconds


def read_static_tf(bags: Sequence[BagInput], context: BufferContext) -> None:
    """/tf_static は bag の最初にしか無いので、本体の読み込みとは別に、先頭から読む。"""
    for bag in bags:
        if bag.topics is not None and '/tf_static' not in bag.topics:
            continue
        for _, _, msg in _messages(bag.path, ['/tf_static']):
            context.add(msg, static=True)


class OfflineSession:
    """メッセージの列を、入力のアダプタで標本にして、周期ごとに Supervisor を回す (bag の読み込みとは独立)。"""

    def __init__(self, config: SupervisorConfig, options: OfflineOptions, context: AdapterContext, t0: float) -> None:
        self.config = config
        self.options = options
        self.t0 = t0
        if options.faults:
            validate_sources(options.faults, config.kinds())
            self.store: SampleStore = FaultInjectingStore(
                FaultInjector(options.faults, t0), config.kinds(), tuple(config.motion_priority),
                config.horizon_sec, config.max_extrapolation_sec, config.alignment)
        else:
            self.store = make_store(config)
        adapters = build_adapters(config, self.store, context)
        for name, path in options.maps.items():
            if config.sources.get(name) is None or config.sources[name].kind != SourceKind.REFERENCE:
                raise ValueError(f'--map {name}: not a reference source')
            self.store.set_map(name, load_map_yaml(path))
        self.routes: Dict[str, List[Callable[[Any], None]]] = {}
        for name, adapter in adapters.items():
            for sub in adapter.subscriptions():
                if name in options.maps and sub.qos == 'latched':
                    continue
                self.routes.setdefault(sub.topic, []).append(sub.handler)

    def run(
        self, messages: Iterable[Tuple[int, str, Any]],
        on_step: Optional[Callable[[Snapshot, StepResult, float], None]] = None,
    ) -> OfflineResult:
        """messages は時刻の順の (記録時刻 [ns], トピック, メッセージ)。t0 + start_sec から周期を回す。"""
        return _run(self, messages, on_step)


def run_offline(
    config: SupervisorConfig, bags: Sequence[BagInput], options: OfflineOptions,
    on_step: Optional[Callable[[Snapshot, StepResult, float], None]] = None,
) -> OfflineResult:
    """bags の最初の bag の始まりを時刻 0 として、start_sec から duration_sec の間を評価する。"""
    t0 = _first_time_ns(bags[0].path) * 1e-9
    context = BufferContext()
    session = OfflineSession(config, options, context, t0)
    read_static_tf(bags, context)
    # 開始より前の標本も履歴として入れる (時刻をそろえるため)
    seek_ns = int((t0 + options.start_sec - config.horizon_sec) * 1e9)
    streams = []
    for bag in bags:
        wanted = list(session.routes) + ['/tf']
        topics = [t for t in (bag.topics or wanted) if t in wanted]
        streams.append(_messages(bag.path, topics, seek_ns))

    def with_tf() -> Iterator[Tuple[int, str, Any]]:
        for item in heapq.merge(*streams, key=lambda m: m[0]):
            if item[1] == '/tf':
                context.add(item[2], static=False)
            yield item

    return session.run(with_tf(), on_step)


def _run(
    session: OfflineSession, messages: Iterable[Tuple[int, str, Any]],
    on_step: Optional[Callable[[Snapshot, StepResult, float], None]],
) -> OfflineResult:
    options, store, t0 = session.options, session.store, session.t0
    start = t0 + options.start_sec
    end = math.inf if options.duration_sec is None else start + options.duration_sec
    supervisor = Supervisor(session.config)
    records: List[TickRecord] = []
    steps: List[Dict[str, Any]] = []
    events: List[Dict[str, Any]] = []
    next_tick = start

    def tick(stamp: float) -> None:
        snapshot = store.snapshot(stamp)
        result = supervisor.step(snapshot)
        t_rel = stamp - t0
        records.append(TickRecord(
            t_rel, result.state, result.fusion.level, result.fusion.blame,
            {n: e.status for n, e in result.evidences.items()},
            {n: e.score for n, e in result.evidences.items()}))
        steps.append(step_record(t_rel, result))
        events.extend({'t': round(t_rel, 3), 'name': e.name, 'level': e.level, **e.fields} for e in result.events)
        if on_step is not None:
            on_step(snapshot, result, t_rel)

    for t_ns, topic, msg in messages:
        t = t_ns * 1e-9
        while t >= next_tick and next_tick <= end:
            tick(next_tick)
            next_tick += options.period_sec
        if t > end:
            break
        for handler in session.routes.get(topic, []):
            handler(msg)
    labels = sorted(options.faults + options.labels, key=lambda f: f.start)
    return OfflineResult(t0, records, steps, events, evaluate(records, labels))


def step_record(t_rel: float, result: StepResult) -> Dict[str, Any]:
    """周期の結果を JSON にできる形にする。"""
    return {
        't': round(t_rel, 3), 'state': result.state.name, 'level': round(result.fusion.level, 4),
        'blame': result.fusion.blame, 'contaminated': result.fusion.contaminated,
        'trusted': result.trusted, 'in_grace': result.in_grace,
        'checks': {name: {'status': e.status.value, 'score': round(e.score, 4),
                          'accumulated': round(result.fusion.accumulated.get(name, 0.0), 4),
                          **{k: round(v, 4) for k, v in e.values.items()}}
                   for name, e in result.evidences.items()},
    }


def write_outputs(result: OfflineResult, out_dir: str, title: str = '') -> None:
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'timeline.jsonl'), 'w', encoding='utf-8') as f:
        for step in result.steps:
            f.write(json.dumps(step, ensure_ascii=False) + '\n')
    with open(os.path.join(out_dir, 'events.jsonl'), 'w', encoding='utf-8') as f:
        for event in result.events:
            f.write(json.dumps(event, ensure_ascii=False) + '\n')
    with open(os.path.join(out_dir, 'report.md'), 'w', encoding='utf-8') as f:
        f.write(format_report(result.report, title))


def load_fault_file(path: Optional[str]) -> List[Fault]:
    if not path:
        return []
    with open(path, encoding='utf-8') as f:
        return parse_faults(yaml.safe_load(f) or {})


def parse_bag(text: str) -> BagInput:
    path, _, topics = text.partition(':')
    return BagInput(path, [t for t in topics.split(',') if t] or None)


def main(argv: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(description='自己位置の監督を rosbag でオフライン評価する')
    parser.add_argument('--config', required=True, help='監督の設定ファイル')
    parser.add_argument('--profile', default='', help='重ねるプロファイル')
    parser.add_argument('--bag', action='append', required=True, type=parse_bag,
                        help='bag のパス。:トピック,... で読むトピックを限る (複数指定可。最初の bag が時刻の基準)')
    parser.add_argument('--map', action='append', default=[], help='<reference の情報源>=<map.yaml>')
    parser.add_argument('--faults', help='Snapshot の段階で注入する故障の定義 (YAML)')
    parser.add_argument('--labels', help='正解としてだけ使う故障の定義 (YAML。記録したときに起きていたもの)')
    parser.add_argument('--period', type=float, default=1.0, help='周期 [s]')
    parser.add_argument('--start', type=float, default=0.0, help='評価の開始 [s]')
    parser.add_argument('--duration', type=float, help='評価の長さ [s]')
    parser.add_argument('--out', help='出力先 (timeline.jsonl、events.jsonl、report.md)')
    args = parser.parse_args(argv)

    config = load_config(args.config, args.profile)
    maps = dict(item.split('=', 1) for item in args.map)
    result = run_offline(config, args.bag, OfflineOptions(
        period_sec=args.period, start_sec=args.start, duration_sec=args.duration, maps=maps,
        faults=load_fault_file(args.faults), labels=load_fault_file(args.labels)))
    text = format_report(result.report, os.path.basename(args.config))
    if args.out:
        write_outputs(result, args.out, os.path.basename(args.config))
    print(text)
