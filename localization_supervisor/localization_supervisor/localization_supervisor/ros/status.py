"""StatusView を localization_supervisor_msgs/LocalizationStatus に写す。"""
from diagnostic_msgs.msg import KeyValue
from localization_supervisor_msgs.msg import LocalizationCheck, LocalizationStatus

from ..core.status import CHECK_STATUS, StatusView


def to_message(view: StatusView, stamp) -> LocalizationStatus:
    """stamp は builtin_interfaces/Time。"""
    msg = LocalizationStatus()
    msg.header.stamp = stamp
    msg.state = int(view.state)
    msg.blame = view.blame
    msg.contaminated = list(view.contaminated)
    msg.isolated_sources = list(view.isolated_sources)
    msg.hold_requested = view.hold_requested
    msg.fusion_level = float(view.fusion_level)
    msg.trusted = view.trusted
    msg.in_grace = view.in_grace
    msg.footprint_in_lethal = view.footprint_in_lethal
    msg.lethal_depth_m = float(view.lethal_depth_m)
    msg.situation = view.situation
    msg.degree = int(view.degree)
    msg.possible_error_m = float(view.possible_error_m)
    msg.clearance_m = float(view.clearance_m)
    msg.speed_limit = float(view.speed_limit)
    msg.sensor_set = view.sensor_set
    msg.verify_id = int(view.verify_id)
    msg.verify_result = view.verify_result
    msg.profile = view.profile
    msg.last_event = view.last_event
    for check in view.checks:
        item = LocalizationCheck()
        item.name = check.name
        item.detector = check.detector
        item.sources = list(check.sources)
        item.status = CHECK_STATUS[check.status]
        item.score = float(check.score)
        item.accumulated = float(check.accumulated)
        item.values = [KeyValue(key=key, value=f'{value:.4g}') for key, value in check.values.items()]
        msg.checks.append(item)
    return msg
