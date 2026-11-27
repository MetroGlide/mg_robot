int32 index
geometry_msgs/PoseStamped pose
# 到達判定には使っていない (停止点の到達半径は Nav2 の goal_checker の共通設定。mg_waypoint_navigation/doc/waypoint_format.md)
float32 reach_tolerance
bool is_through_point
# 通過点で、Nav2 の完了を待たずに次へ進む半径 [m] (通過点のときだけ意味がある)
float32 through_tolerance
string[] on_reached_action_types
