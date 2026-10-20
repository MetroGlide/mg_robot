"""自己位置推定の監督ノード。

設定ファイルの情報源・判定・総合・状態遷移の組み合わせで、中核 (core/supervisor.py) の Supervisor を
周期 (period_sec) ごとに回し、状態のメッセージ・診断・イベントを出す。判定と状態遷移は中核にあり、
このノードは入力を標本にして渡し、結果を出すだけにする。

  パラメータ
    config_file : 設定ファイル (必須。書き方は doc/parameters.md)
    profile     : 重ねるプロファイル (空は基の設定)。走行中に変えられる (サービス ~/set_profile でも)
  入力
    設定の sources.<名前>.input のアダプタが購読するトピックと TF
    external_reinit.topics: 届いたら、人による上書きとして、状態を NORMAL に戻して猶予を置く
  出力
    output.status_topic (LocalizationStatus)、診断 output.diagnostic_name、イベント (events.sink)
"""
from typing import Optional

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
from ..core.status import StatusView, status_view
from ..core.supervisor import Supervisor, make_store
from .adapters import build_adapters
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
    """基の設定とすべてのプロファイルが、このノードで動かせるか。

    - 入力のアダプタ・再初期化の送り先・対処は起動時に基の設定で作るので、プロファイルで変えることはできない
    - recovery_enabled には、再初期化の送り先 (recovery.resetters) が要る
    """
    for name in [''] + list(config.profiles):
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
        if not config_file:
            raise ConfigError('parameter config_file is required')
        config = load_config(config_file)
        check_node_supports(config)
        self._supervisor = Supervisor(config)
        if profile:
            self._supervisor.set_profile(profile)
        self.add_on_set_parameters_callback(self._on_parameters)

        self._store = make_store(config)
        # Node の _context (rclpy の Context) と名前を分ける
        self._tf = TfContext(self)
        for adapter in build_adapters(config, self._store, self._tf).values():
            for sub in adapter.subscriptions():
                self.create_subscription(sub.msg_type, sub.topic, sub.handler, _QOS[sub.qos])
        self._own = OwnMessages()
        self._resetters = build_resetters(config.resetters, self, self._own)
        self._actuators = build_actuators(config.actuators, self)
        for topic in config.external_reinit_topics:
            self.create_subscription(
                PoseWithCovarianceStamped, topic, lambda msg, t=topic: self._on_external_reinit(t, msg), 10)

        output = config.output
        self._status_pub = self.create_publisher(LocalizationStatus, output.status_topic, 10)
        self.create_service(SetProfile, '~/set_profile', self._on_set_profile)
        self._events = build_event_sink(config.event_sink, self.get_logger())
        self._view: Optional[StatusView] = None
        self._updater = Updater(self)
        self._updater.setHardwareID(output.hardware_id)
        self._updater.add(LocalizationTask(output.diagnostic_name, lambda: self._view))
        self.create_timer(config.period_sec, self._on_timer)
        self._events.info(
            'node.started', name=self.get_name(), config=config_file, profile=profile,
            detectors=','.join(self._supervisor.detectors))

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

    def _on_timer(self) -> None:
        now = self._now()
        if now == 0.0:
            return
        result = self._supervisor.step(self._store.snapshot(now))
        for event in result.events:
            self._events.emit(event.name, event.level, event.fields)
        for directive in result.directives:
            if directive.kind == 'reinit':
                for resetter in self._resetters:
                    resetter.reset(directive.candidate)
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
