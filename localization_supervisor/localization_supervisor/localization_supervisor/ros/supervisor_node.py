"""自己位置推定の監督ノード。

設定ファイルの情報源・判定・総合・状態遷移の組み合わせで、中核 (core/supervisor.py) の Supervisor を
周期 (period_sec) ごとに回し、状態のメッセージ・診断・イベントを出す。判定と状態遷移は中核にあり、
このノードは入力を標本にして渡し、結果を出すだけにする。

  パラメータ
    config_file : 設定ファイル (必須。書き方は doc/parameters.md)
    profile     : 重ねるプロファイル (空は基本設定)。走行中に変えられる (サービス ~/set_profile でも)
    use_gate    : 設定の gate 節の門を使う (起動時だけ)。門の周期で回し、門の照合を別のスレッドで回す
  入力
    設定の sources.<名前>.input のアダプタが購読するトピックと TF
    external_reinit.topics: 今のプロファイルのトピックに届いたら、人による上書きとして、状態を NORMAL に戻して
                            猶予を置く (どのプロファイルのトピックも購読しておく。門があるときは購読しない)
    gate.manual_pose_topic: 手動の初期姿勢 (門があるとき。門がその周りを照合してから置き直す)
  出力
    output.status_topic (LocalizationStatus)、診断 output.diagnostic_name、イベント (events.sink)
"""
import threading
import time
from typing import Optional, Tuple

import rclpy
import tf2_ros
from diagnostic_updater import Updater
from geometry_msgs.msg import PoseWithCovarianceStamped
from localization_supervisor_msgs.msg import LocalizationStatus
from localization_supervisor_msgs.srv import SetProfile
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data

from ..core.config import ConfigError, SupervisorConfig, load_config
from ..core.map_match import MapMatcher, MatchDecision
from ..core.motion_gate import MatchRequest, run_match
from ..core.status import StatusView, status_view
from ..core.supervisor import Supervisor, make_store
from .adapters import build_adapters
from .adapters.standard import pose_from_msg
from .actuators import build_actuators
from .actuators.variance_scale import delivered_scales
from .diagnostics import LocalizationTask
from .event_sinks import build_event_sink
from .own_messages import OwnMessages
from .resetters import build_resetters
from .status import to_message
from .tf_context import TfBufferContext

_QOS = {
    'default': 10,
    'sensor': qos_profile_sensor_data,
    'latched': QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
}


class TfContext(TfBufferContext):
    def __init__(self, node: Node) -> None:
        super().__init__()
        self.listener = tf2_ros.TransformListener(self.buffer, node)


def check_node_supports(config: SupervisorConfig) -> None:
    """基本設定とすべてのプロファイルが、このノードで動かせるか。"""
    for name in [''] + list(config.profiles):
        check_profile_supported(config, name)


def check_profile_supported(config: SupervisorConfig, name: str) -> None:
    """プロファイル (カンマで区切った重ね合わせも) が、このノードで動かせるか。

    - 入力のアダプタ・再初期化の送り先・対処は起動時に基本設定で作るので、プロファイルで変えることはできない
    - recovery_enabled には、再初期化の送り先 (recovery.resetters) が要る
    """
    profiled = config.with_profile(name)
    where = f'profiles.{name}' if name else 'config'
    if profiled.sources != config.sources:
        raise ConfigError(f'{where}: cannot change sources (adapters are built at startup)')
    if profiled.resetters != config.resetters or profiled.actuators != config.actuators:
        raise ConfigError(f'{where}: cannot change recovery.resetters or recovery.actuators '
                          '(they are built at startup)')
    if profiled.machine.recovery_enabled and not profiled.resetters:
        raise ConfigError(f'{where}: state_machine.recovery_enabled requires recovery.resetters '
                          '(where to send the reinitialized pose)')


class LocalizationSupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__('localization_supervisor_node')
        config_file = self.declare_parameter('config_file', '').value
        profile = self.declare_parameter('profile', '').value
        use_gate = self.declare_parameter('use_gate', False).value
        if not config_file:
            raise ConfigError('parameter config_file is required')
        config = load_config(config_file)
        if use_gate or 'gate' in config.raw:
            config = config.with_gate(use_gate)
        check_node_supports(config)
        self._config = config
        self._supervisor = Supervisor(config)
        if profile:
            check_profile_supported(config, profile)
            self._supervisor.set_profile(profile)
        self.add_on_set_parameters_callback(self._on_parameters)

        self._store = make_store(config)
        # Node の _context (rclpy の Context) と名前を分ける
        self._tf = TfContext(self)
        for adapter in build_adapters(config.sources, self._store, self._tf).values():
            for sub in adapter.subscriptions():
                self.create_subscription(sub.msg_type, sub.topic, sub.handler, _QOS[sub.qos])
        self._own = OwnMessages()
        resetters = build_resetters(config.resetters, self, self._own)
        self._resetters = [(item.get('role', 'estimator'), resetter)
                           for item, resetter in zip(config.resetters, resetters)]
        self._actuators = build_actuators(config.actuators, self)
        # 購読は起動時に作るので、どのプロファイルのトピックも購読し、使うかは今のプロファイルで決める
        topics = dict.fromkeys(config.external_reinit_topics)
        for name in config.profiles:
            topics.update(dict.fromkeys(config.with_profile(name).external_reinit_topics))
        for topic in topics if config.gate is None else ():
            self.create_subscription(
                PoseWithCovarianceStamped, topic, lambda msg, t=topic: self._on_external_reinit(t, msg), 10)

        # 門の照合 (重いので周期の処理と別のスレッドで回し、結果を次の周期に渡す) と、手動の初期姿勢
        self._lock = threading.Lock()
        self._match: Optional[Tuple[MatchRequest, MatchDecision]] = None
        self._manual = None
        self._match_ms = 0.0
        self._matcher = MapMatcher(config.gate.map_match) if config.gate is not None else None
        if config.gate is not None and config.manual_pose_topic:
            self.create_subscription(PoseWithCovarianceStamped, config.manual_pose_topic, self._on_manual, 10)

        output = config.output
        self._status_pub = self.create_publisher(LocalizationStatus, output.status_topic, 10)
        self.create_service(SetProfile, '~/set_profile', self._on_set_profile)
        self._events = build_event_sink(config.event_sink, self.get_logger())
        self._view: Optional[StatusView] = None
        self._updater = Updater(self)
        self._updater.setHardwareID(output.hardware_id)
        self._updater.add(LocalizationTask(output.diagnostic_name, lambda: self._view))
        self.create_timer(config.gate_period_sec if config.gate is not None else config.period_sec, self._on_timer)
        self._events.info(
            'node.started', name=self.get_name(), config=config_file, profile=profile,
            detectors=','.join(self._supervisor.detectors), gate=config.gate is not None)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_external_reinit(self, topic: str, msg) -> None:
        if self._own.consume(topic, msg.header.stamp.sec, msg.header.stamp.nanosec):
            return
        now = self._now()
        if now > 0.0:
            self._supervisor.external_reinit(now, topic)

    def _on_parameters(self, params) -> SetParametersResult:
        try:
            for param in params:
                if param.name == 'profile':
                    check_profile_supported(self._config, param.value)
                    self._supervisor.set_profile(param.value)
                elif param.name == 'config_file':
                    raise ValueError('config_file cannot be changed while running')
        except (ValueError, ConfigError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def _on_set_profile(self, request, response):
        result = self.set_parameters([Parameter('profile', Parameter.Type.STRING, request.profile)])[0]
        response.success = result.successful
        response.message = result.reason or f'profile: {request.profile or "(base)"}'
        return response

    def _on_manual(self, msg: PoseWithCovarianceStamped) -> None:
        with self._lock:
            self._manual = pose_from_msg(msg.pose.pose)

    def _run_match(self, request: MatchRequest) -> None:
        started = time.monotonic()
        decision = run_match(self._matcher, request)
        with self._lock:
            self._match_ms = (time.monotonic() - started) * 1000.0
            self._match = (request, decision)

    def _on_timer(self) -> None:
        now = self._now()
        if now == 0.0:
            return
        with self._lock:
            match, self._match = self._match, None
            manual, self._manual = self._manual, None
            match_ms = self._match_ms
        if manual is not None:
            self._supervisor.manual_pose(manual)
        result = self._supervisor.step(self._store.snapshot(now), match)
        map_match_event = f'{self._config.event_prefix}.map_match'
        for event in result.events:
            fields = dict(event.fields, duration_ms=round(match_ms, 1)) if event.name == map_match_event \
                else event.fields
            self._events.emit(event.name, event.level, fields)
        for directive in result.directives:
            if directive.kind == 'reinit':
                for role, resetter in self._resetters:
                    if directive.targets == 'all' or role == directive.targets:
                        resetter.reset(directive.candidate)
        if result.gate is not None and result.gate.request is not None:
            threading.Thread(target=self._run_match, args=(result.gate.request,), daemon=True).start()
        for actuator in self._actuators:
            actuator.apply(result)
        for name, scale in delivered_scales(self._actuators, result.applied_scales).items():
            self._store.note_scale(self._supervisor.config.residuals[name].a, now, scale)
        self._view = status_view(result, self._supervisor.config, self._supervisor.profile)
        self._status_pub.publish(to_message(self._view, self.get_clock().now().to_msg()))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationSupervisorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
