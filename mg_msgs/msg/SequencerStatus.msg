builtin_interfaces/Time stamp
string state
int32 current_index
int32 total_waypoints
uint32 countdown_ms_remaining
bool is_paused
string[] pause_requesters
float32 distance_remaining

# ON_ARRIVING 中に実行しているアクションの type。実行していなければ空
string current_action
# wait_trigger で IDLE に止まっている (未開始の IDLE と区別する)
bool waiting_trigger
