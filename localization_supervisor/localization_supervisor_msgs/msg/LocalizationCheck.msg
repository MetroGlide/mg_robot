# 判定のインスタンスの 1 周期分の結果 (LocalizationStatus.checks)

# インスタンスの名前 (例: amcl_gnss)
string name
# 判定の種類 (設定の type。例: pose_consistency)
string detector
# 比べた情報源の名前
string[] sources

uint8 NORMAL=0
uint8 ANOMALY=1
uint8 UNKNOWN=2     # 判定できない (入力が古い、届いていないなど)
uint8 status

# 正規化した強さ (1 が異常のしきい値)。UNKNOWN のときは 0
float32 score
# 総合での、このインスタンスの累積
float32 accumulated

# 記録用の値 (判定ごとに決まる)
diagnostic_msgs/KeyValue[] values
