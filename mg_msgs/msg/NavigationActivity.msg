# Waypoint 走行の「いまの活動」の要約。表示用であり、制御の入力にはしない。
# 判定は waypoint_sequencer_node が 1 か所で行う。UI は表示だけを担当する。
builtin_interfaces/Time stamp

# 活動の種類 (下の定数のどれか)
string code
# 固定の短い説明。変わる数値は含めず、下の数値フィールドで渡す
string text
# 補足 (要求元、アクションの種類、collision_monitor のポリゴン名など)
string detail
# 同じ code が続いている秒数
float32 elapsed_sec
# 対象のウェイポイントの番号 (0 始まり)。該当しなければ -1
int32 index
int32 total
# code ごとの残り。走行中は残り距離 [m]、出発待ちと待機は残り秒数。該当しなければ -1
float32 remaining

# 状態ランプの意味 (code ごとに mg_waypoint_navigation/config/activity_lamps.yaml で決める)。
# 色・点滅の見た目は使う側 (Web UI・MMI) が決める。健全性は /diagnostics_agg が表す
# 人がどうすればよいか (下の LAMP_RESPONSE_* のどれか)
string lamp_response
# 動いているか (下の LAMP_MOTION_* のどれか)
string lamp_motion

string LAMP_RESPONSE_NORMAL="normal"
string LAMP_RESPONSE_WATCH="watch"
string LAMP_RESPONSE_ACT="act"
string LAMP_RESPONSE_OPERATOR="operator"
string LAMP_RESPONSE_START="start"
string LAMP_RESPONSE_NONE="none"
string LAMP_MOTION_MOVING="moving"
string LAMP_MOTION_WAITING="waiting"
string LAMP_MOTION_HANDLING="handling"

string IDLE="idle"
string DEPARTURE_WAIT="departure_wait"
string NAVIGATING="navigating"
string MANUAL_GOAL="manual_goal"
string EXTERNAL_GOAL="external_goal"
string OBSTACLE_SLOWDOWN="obstacle_slowdown"
string OBSTACLE_STOP="obstacle_stop"
string SENSOR_TIMEOUT_STOP="sensor_timeout_stop"
string RECOVERY="recovery"
string DETOUR_REJECTED="detour_rejected"
string NO_PROGRESS="no_progress"
string ACTION="action"
string TRIGGER_WAIT="trigger_wait"
string READY_TO_START="ready_to_start"
string PAUSED="paused"
string LOCALIZATION_HOLD="localization_hold"
string WATCHDOG_PAUSE="watchdog_pause"
string LOCALIZATION_RECOVERING_DRIVE="localization_recovering_drive"
string LOCALIZATION_SLOWDOWN="localization_slowdown"
string NO_LIDAR_DRIVE="no_lidar_drive"
string LOCALIZATION_UNKNOWN="localization_unknown"
string GOAL_REACHED="goal_reached"
string ERROR="error"
