# ホイールオドメトリの補正 (wheel_odom_corrector_node)

ホイールオドメトリ (`wheel_odometry_node` が出す値) の、スケールとバイアスを補正して、EKF が使う速度の共分散を設定する。
ナビゲーションの自己位置推定 (`ekf_global_node`) の入力を、補正済みの `/odom` にするためのノード。

```
wheel_odometry_node ─ /odom/raw ─► wheel_odom_corrector_node ─ /odom ─► odometry_tf_broadcaster (odom→base_footprint)
                                                                    └─► ekf_global_node (速度), Nav2
```

## 補正のモデル

差動二輪、車体座標系。

| パラメータ | 意味 | 補正 |
| :--- | :--- | :--- |
| `k_v` | 並進のスケール (車輪半径) | `dx' = k_v · dx` |
| `k_w` | 旋回のスケール (実効トレッド幅) | `dyaw' = k_w · dyaw + c · dx'` |
| `yaw_bias_per_meter` (`c`) | 走行距離あたりに曲がる量 [rad/m] (左右の車輪半径差) | 同上 |
| `time_offset` | 生のオドメトリの遅れ [s] | スタンプを過去へずらす |
| `twist_source` | 速度の出どころ (`raw` / `pose_diff`) | `raw`: ドライバの速度にスケール補正をかける。`pose_diff`: 補正した姿勢の差分から求める |
| `twist_window` | `pose_diff` で、何メッセージ前の姿勢との差から求めるか | 例: 20 Hz で 2 なら 0.1 秒 |
| `reset_jump_m` / `max_speed_mps` | 1 メッセージの移動量が `reset_jump_m + max_speed_mps · (前のメッセージからの時間)` を超えたら、生の姿勢が飛んだとみなす | 飛んだ増分を捨てる。補正した姿勢は連続のまま |
| `enabled` | `false` で補正せずに生の値を通す (共分散は設定する) | |
| `covariance_x` / `covariance_y` / `covariance_yaw` | 姿勢の共分散 (EKF は姿勢を使わない) | |
| `covariance_vx` / `covariance_vy` / `covariance_vyaw` | 速度の共分散 (EKF が使う) | |

- パラメータは `params/wheel_odom_corrector.yaml`。値は `tools/scripts/calib_wheel_odom.py` で、走行ログから推定する ([tools の README](../../tools/README.md))。
- 補正は姿勢の増分に対して行う。
- 補正の計算は、ROS に依存しない `scripts/wheel_odom_correction.py` にある。`make test pkg=mg_drivers` で単体テストする。

## 速度が 0 になる不具合

走行ログ (`record_slam_20260913_051635` の 370〜730 s) に、ドライバの速度が 0 のまま、姿勢だけ更新される期間があった。
EKF は速度を観測として使うため、速度を信頼する設定 (共分散を小さくする) では、この期間に破綻する。
`twist_source: pose_diff` は、姿勢の差分から速度を求めるので影響を受けない (速度が正常な走行では、前進速度の相関 0.997、旋回は 0.91)。

## 起動引数

`mg_bringup` の `bringup_navigation.launch.py` から `mg_drivers` の launch まで、同じ名前で伝わる。

| 引数 | 内容 |
| :--- | :--- |
| `use_odom_corrector` | `true` でドライバの出力先を `/odom/raw` に変え、補正ノードを起動する |
| `odom_corrector_params_file` | `params/` 配下のファイル名、または絶対パス |

## `/odom` の利用者と影響

補正を有効にすると、`/odom` は次のものが受け取る。

- EKF (`ekf_global_node`)
- `odometry_tf_broadcaster` (`odom→base_footprint`)
- Nav2 (`bt_navigator` の `odom_topic`、controller)
- Web UI の ODOM 表示
- rosbag の記録

`pose_diff` の速度は、ドライバの速度より約 0.05 s 遅れる。`velocity_smoother` は OPEN_LOOP なので影響しないが、controller の追従は実機で確認する。
問題があれば、`twist_source: raw` (ドライバの速度にスケール補正だけをかける) に戻せる。

## 運用上の注意

- **単一障害点**: 補正ノードが落ちると、`/odom` と `odom→base_footprint` の TF が止まる。launch で `respawn` (1 s 後) を付けてある。生の姿勢が飛んだときは、その増分を捨て、補正した姿勢を生の姿勢に合わせ直さない。補正した姿勢と生の姿勢は走るほど離れる (スケールと向きの偏りの補正の分) ので、合わせ直すと `odom→base_footprint` がその分だけ跳び、AMCL の粒子が散る (再生の評価で、記録が数秒途切れた後に約 100 m 跳んだ例がある)。途切れの後の普通の動きを飛びと見なさないよう、閾値は途切れた時間に応じて広げる。補正ノード自体が再起動したときは、最初のメッセージの生の姿勢から始めるので、`odom` フレームは跳ぶ。
- **補正値は路面と荷重で変わる**: yaml の値は、特定の日・同じ路面での推定。環境が変わったら、`calib_wheel_odom.py` で再推定する。
- **元に戻す方法**: `use_odom_corrector:=false` で、ドライバが `/odom` を直接出す従来の構成になる。SLAM (`slam_gnss_2d`、slam_toolbox) は補正を使わず、従来どおり生の `/odom` を使う。
- **負荷**: オドメトリのメッセージを 1 つ変換する Python ノードで、ごくわずか。
