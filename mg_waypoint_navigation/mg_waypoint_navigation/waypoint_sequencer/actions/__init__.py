from __future__ import annotations

import rclpy.node

from mg_waypoint_navigation.waypoint import ActionConfig
from mg_waypoint_navigation.waypoint_sequencer.actions.base import (
    BaseAction,
    EndpointCache,
)
from mg_waypoint_navigation.waypoint_sequencer.actions.builtins import (
    AmclResetAction,
    LoadMapAction,
    WaitAction,
    WaitTriggerAction,
    SetNavigationModeAction,
)
from mg_waypoint_navigation.waypoint_sequencer.actions.generic import (
    GenericPublishAction,
    GenericServiceAction,
)

_ACTION_REGISTRY = {
    "service": GenericServiceAction,
    "publish": GenericPublishAction,
    "load_map": LoadMapAction,
    "amcl_reset": AmclResetAction,
    "wait": WaitAction,
    "wait_trigger": WaitTriggerAction,
    "set_navigation_mode": SetNavigationModeAction,
}


def build_action(
    config: ActionConfig,
    node: rclpy.node.Node,
    endpoints: EndpointCache,
) -> BaseAction:
    cls = _ACTION_REGISTRY.get(config.type)
    if cls is None:
        raise ValueError(f"Unknown action type: {config.type!r}")
    return cls(config, node, endpoints)


__all__ = [
    "BaseAction",
    "EndpointCache",
    "build_action",
    "GenericServiceAction",
    "GenericPublishAction",
    "LoadMapAction",
    "AmclResetAction",
    "WaitAction",
    "WaitTriggerAction",
    "SetNavigationModeAction",
]
