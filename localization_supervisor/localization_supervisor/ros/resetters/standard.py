"""標準の再初期化の送り先。"""
import math
from typing import Any, Mapping

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty

from ...core.interfaces import Candidate, ParamSpec
from ..own_messages import OwnMessages
from .base import PoseResetter


def pose_message(candidate: Candidate, frame_id: str, stamp, covariance_scale: float) -> PoseWithCovarianceStamped:
    """候補を PoseWithCovarianceStamped にする。共分散の 3x3 (x, y, yaw) は 6x6 の対応する場所に入れる。"""
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    x, y, yaw = candidate.pose
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    covariance = [0.0] * 36
    indices = (0, 1, 5)
    for i, row in enumerate(indices):
        for j, col in enumerate(indices):
            covariance[row * 6 + col] = float(candidate.cov[i][j]) * covariance_scale
    msg.pose.covariance = covariance
    return msg


class TopicResetter(PoseResetter):
    """PoseWithCovarianceStamped をトピックに送る。"""
    PARAMS = {
        'topic': ParamSpec(str, None),
        'frame_id': ParamSpec(str, 'map'),
        'covariance_scale': ParamSpec(float, 1.0),
    }

    def __init__(self, name: str, params: Mapping[str, Any], node: Any, own: OwnMessages) -> None:
        super().__init__(name, params, node, own)
        self._pub = node.create_publisher(PoseWithCovarianceStamped, self.params['topic'], 10)

    def reset(self, candidate: Candidate) -> None:
        stamp = self.node.get_clock().now().to_msg()
        self.own.add(self.params['topic'], stamp.sec, stamp.nanosec)
        self._pub.publish(pose_message(candidate, self.params['frame_id'], stamp, self.params['covariance_scale']))


class Amcl(TopicResetter):
    """AMCL の /initialpose に送る。nomotion_delay_sec 後に request_nomotion_update を呼ぶ
    (ロボットが止まっていても、初期化し直した姿勢で粒子を更新して姿勢を出し直させる)。"""
    PARAMS = {
        **{k: v for k, v in TopicResetter.PARAMS.items() if k != 'topic'},
        'topic': ParamSpec(str, '/initialpose'),
        'nomotion_service': ParamSpec(str, '/request_nomotion_update', '空なら呼ばない'),
        'nomotion_delay_sec': ParamSpec(float, 0.5),
    }

    def __init__(self, name: str, params: Mapping[str, Any], node: Any, own: OwnMessages) -> None:
        super().__init__(name, params, node, own)
        service = self.params['nomotion_service']
        self._client = node.create_client(Empty, service) if service else None
        self._timer = None

    def reset(self, candidate: Candidate) -> None:
        super().reset(candidate)
        if self._client is None:
            return
        if self._timer is not None:
            self._timer.cancel()
        self._timer = self.node.create_timer(self.params['nomotion_delay_sec'], self._request_nomotion_update)

    def _request_nomotion_update(self) -> None:
        self._timer.cancel()
        if self._client.service_is_ready():
            self._client.call_async(Empty.Request())


class RobotLocalization(TopicResetter):
    """robot_localization の /set_pose に送る。"""
    PARAMS = {
        **{k: v for k, v in TopicResetter.PARAMS.items() if k != 'topic'},
        'topic': ParamSpec(str, '/set_pose'),
    }
