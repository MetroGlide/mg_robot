# 自己位置推定の監督 (localization_supervisor_node) が一定の周期で配信する状態

std_msgs/Header header

# 状態
uint8 NORMAL=0       # 正常
uint8 SUSPECT=1      # 異常の疑い (確認中)
uint8 ISOLATED=2     # 疑わしい情報源を切り離した
uint8 RECOVERING=3   # 推定器を初期化し直し、収束を確認中
uint8 DEGRADED=4     # 異常を確認した (通知だけのとき)、または復旧できていない
uint8 state

# 疑わしい独立の情報源の名前。決めきれなければ "unknown"、異常がなければ空
string blame
# 出力が汚れていると見なす、融合の結果の情報源 (例: EKF)
string[] contaminated
# 切り離している情報源
string[] isolated_sources
# 走行を止めてほしいか (切り離してから、戻すまで)
bool hold_requested

# 総合の強さ (総合の方式ごとの単位)
float32 fusion_level
# 自己位置が信頼できる周期か (アンカーを残す条件を満たす)
bool trusted
# 起動直後・復旧直後・地図の切り替えの後などの、総合しない猶予の間か
bool in_grace

# 安全の信号: 指定した姿勢の footprint が、計画用地図の致死セルにかかっているか
bool footprint_in_lethal
# 致死セルへの食い込みの深さ [m] (かかっていなければ 0、計算できなければ -1)
float32 lethal_depth_m

# 使っているプロファイル (空は基本設定)
string profile
# 直近に起きたこと (例: "suspect: amcl_jump"、"map_changed")
string last_event

# 有効な判定のインスタンスごとの結果
LocalizationCheck[] checks
