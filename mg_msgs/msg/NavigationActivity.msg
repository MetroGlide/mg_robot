# Waypoint 走行の「いまの活動」の要約。表示用であり、制御の入力にはしない。
# 判定は waypoint_sequencer_node が 1 か所で行う。UI は表示だけを担当する。
builtin_interfaces/Time stamp

# 活動の種類 (下の定数のどれか)
string code
# 固定の短い説明。変わる数値は含めず、下の数値フィールドで渡す
string text
# 補足 (要求元、アクションの種類、collision_monitor のポリゴン名など)
string detail
# 表示の強調。健全性ではない (健全性は /diagnostics_agg が表す)
uint8 emphasis
# 同じ code が続いている秒数
float32 elapsed_sec
# 対象のウェイポイントの番号 (0 始まり)。該当しなければ -1
int32 index
int32 total
# code ごとの残り。走行中は残り距離 [m]、出発待ちと待機は残り秒数。該当しなければ -1
float32 remaining

uint8 NORMAL=0
uint8 NOTICE=1
uint8 ALERT=2

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
string PAUSED="paused"
string GOAL_REACHED="goal_reached"
string ERROR="error"
