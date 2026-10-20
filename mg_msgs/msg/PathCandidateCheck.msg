# グローバルプランナが引いた経路 (候補) を、CommitPath が採用するかどうかの判定結果。表示用であり、制御の入力にはしない。
# 判定は CommitPath が 1 か所で行う。候補があったときだけ、判定のたびに配信する。
builtin_interfaces/Time stamp

# 候補が直線距離に対して長すぎるために棄却したとき true
bool rejected
# 候補の経路の長さ [m]
float32 path_length
# 候補の始点からゴールまでの直線距離 [m]
float32 straight_distance
# 採用できる経路の長さの上限 [m]。上限の判定を無効にしているときは -1
float32 limit
