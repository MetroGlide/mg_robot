import { NODE_NS, nodeNs } from './namespaces'

export const SERVICES = {
  WAYPOINT_START: nodeNs(NODE_NS.WAYPOINT_SEQUENCER, '/start'),
  WAYPOINT_STOP: nodeNs(NODE_NS.WAYPOINT_SEQUENCER, '/stop'),
  WAYPOINT_RELOAD: nodeNs(NODE_NS.WAYPOINT_SEQUENCER, '/reload_waypoints'),
  WAYPOINT_NAVIGATE_TO_POSE: nodeNs(NODE_NS.WAYPOINT_SEQUENCER, '/navigate_to_pose'),
  WAYPOINT_LOAD_MAP: nodeNs(NODE_NS.WAYPOINT_SEQUENCER, '/load_map'),
  // ウェイポイントの amcl_on / amcl_off と同じ要求元として、AMCL のゲートを切り替える
  AMCL_GATE_WAYPOINT: '/amcl_gate_arbiter/waypoint/change_publish_state',
  GNSS_CHANGE_PUBLISH_STATE: '/slam_gnss_nav_bridge/change_publish_state',
  AMCL_REINITIALIZE_GLOBAL: '/reinitialize_global_localization',
  GNSS_AMCL_REINIT: '/gnss_amcl_initializer_node/request_reinit',
  LIFECYCLE_NAV_IS_ACTIVE: '/lifecycle_manager_navigation/is_active',
  LIFECYCLE_LOC_IS_ACTIVE: '/lifecycle_manager_localization/is_active',
  ROSBAG_PAUSE: '/rosbag2_player/pause',
  ROSBAG_RESUME: '/rosbag2_player/resume',
  ROSBAG_SET_RATE: '/rosbag2_player/set_rate',
  SLAM_GNSS2D_GET_POSE_GRAPH: '/slam_gnss_2d/get_pose_graph',
} as const
