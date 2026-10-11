#!/usr/bin/env python3
"""PC の CPU 使用率とメモリ使用率を配信し、しきい値で診断のレベルを決める。

- `cpu_usage`・`memory_usage` (`Float32`): 配信する値 (Web UI の表示用)
- 診断 `pc_resource_publisher_node: cpu` と `pc_resource_publisher_node: memory`: それぞれのレベル。
  瞬間的な値で揺れないよう、直近 `average_samples` 回の平均で判定する。しきい値はパラメータ
"""
import psutil

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, DiagnosticTask, Updater
from rclpy.node import Node

from std_msgs.msg import Float32

from mg_utils.monitoring.thresholds import MovingAverage, level_for_high

HARDWARE_ID = 'mg-robot'

_LEVELS = {
    'ok': DiagnosticStatus.OK,
    'warn': DiagnosticStatus.WARN,
    'error': DiagnosticStatus.ERROR,
}


class UsageTask(DiagnosticTask):
    """使用率 [%] の診断。直近の平均が warn 以上で WARN、error 以上で ERROR。"""

    def __init__(self, name: str, average: MovingAverage, warn: float, error: float) -> None:
        super().__init__(name)
        self._average = average
        self._warn = warn
        self._error = error

    def run(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        value = self._average.value
        level = level_for_high(value, self._warn, self._error)
        stat.summary(_LEVELS[level], f'{value:.0f} %')
        stat.add('percent', f'{value:.1f}')
        stat.add('warn_percent', f'{self._warn:.0f}')
        stat.add('error_percent', f'{self._error:.0f}')
        return stat


class PCResourcePublisherNode(Node):

    def __init__(self):
        super().__init__('pc_resource_publisher_node')
        self.get_logger().info('pc_resource_publisher_node started')

        self._publish_rate = self.declare_parameter(
            'publish_rate',
            1.0).value  # [Hz]
        self._cpu_interval = self.declare_parameter(
            'cpu_interval',
            0.5).value  # [sec]
        # 診断のしきい値 [%]。warn 以上で WARN、error 以上で ERROR
        cpu_warn = self.declare_parameter('cpu_warn_percent', 85.0).value
        cpu_error = self.declare_parameter('cpu_error_percent', 95.0).value
        mem_warn = self.declare_parameter('memory_warn_percent', 90.0).value
        mem_error = self.declare_parameter('memory_error_percent', 97.0).value
        average_samples = self.declare_parameter('average_samples', 5).value

        if self._cpu_interval > 1/self._publish_rate:
            self.get_logger().warn(
                'cpu_interval is larger than 1/publish_rate. '
                'cpu_interval is set to 1/publish_rate.')
            self._cpu_interval = 1/self._publish_rate

        self.cpu_average = MovingAverage(average_samples)
        self.mem_average = MovingAverage(average_samples)
        self._updater = Updater(self)
        self._updater.setHardwareID(HARDWARE_ID)
        self._updater.add(UsageTask('cpu', self.cpu_average, cpu_warn, cpu_error))
        self._updater.add(UsageTask('memory', self.mem_average, mem_warn, mem_error))

        self.cpu_usage_pub = self.create_publisher(
            Float32,
            'cpu_usage',
            1)
        self.mem_usage_pub = self.create_publisher(
            Float32,
            'memory_usage',
            1)

        self.timer = self.create_timer(
            1/self._publish_rate,
            self.timer_callback)

    def timer_callback(self):
        cpu_usage = psutil.cpu_percent(interval=self._cpu_interval)
        self.get_logger().debug(f'cpu_usage: {cpu_usage:.2f} %')
        self.cpu_usage_pub.publish(Float32(data=cpu_usage))
        self.cpu_average.add(cpu_usage)

        mem_usage = psutil.virtual_memory().percent
        self.get_logger().debug(f'mem_usage: {mem_usage:.2f} %')
        self.mem_usage_pub.publish(Float32(data=mem_usage))
        self.mem_average.add(mem_usage)


def main(args=None):
    rclpy.init(args=args)

    pc_resource_publisher_node = PCResourcePublisherNode()

    rclpy.spin(pc_resource_publisher_node)

    pc_resource_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
