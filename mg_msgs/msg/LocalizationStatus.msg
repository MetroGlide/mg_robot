# 自己位置推定の監督ノード (localization_supervisor) が配信する状態

std_msgs/Header header

# 状態
uint8 NORMAL=0       # 正常
uint8 SUSPECT=1      # 異常の疑い (確認中)
uint8 ISOLATED=2     # AMCL を EKF から切り離した
uint8 RECOVERING=3   # EKF の姿勢で AMCL を初期化し直し、収束を確認中
uint8 DEGRADED=4     # 復旧できていない (AMCL を使えない)
uint8 state

# AMCL の出力を EKF に入れているか
bool amcl_attached

# 直近の判定の値。使えなかったものは -1
float32 amcl_gnss_d2       # AMCL と GNSS の位置の差のマハラノビス距離の二乗 (2 自由度)
float32 amcl_jump_m        # 連続する AMCL の推定が示す map→odom の位置の飛び [m]
float32 amcl_ekf_diff_m    # AMCL と EKF の位置の差 [m]
float32 scan_match_ratio   # EKF の姿勢でスキャンを地図に重ねたときに一致した点の割合 (0-1)
float32 scan_match_gain    # EKF の姿勢の周りでずらしたほうが、一致した点の割合がどれだけ増えるか (0 なら今の姿勢が最良)

# 直近に起きたこと (例: "isolated: amcl_gnss", "reinit from ekf", "recovered")
string last_event
