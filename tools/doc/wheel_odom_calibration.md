# ホイールオドメトリの校正

走行ログとSLAM出力から、ホイールオドメトリの誤差を推定する `calib_wheel_odom.py`。補正のモデルは [mg_drivers](../../mg_drivers/doc/wheel_odom_corrector.md)。一覧は [tools の README](../README.md)。

## `calib_wheel_odom.py` (ホイールオドメトリの校正)

走行ログの `/odom` (**補正前の生の姿勢**) と、slam_gnss_2d の SLAM 出力 (`pose_graph.json`) を比べて、
ホイールオドメトリの誤差を推定します。全体を一度に積算すると誤差が累積して崩れるため、真値の経路で
一定の距離ごとに区切った窓について、窓の始点から終点への相対移動を比べ、Huber 損失で外れ値の窓の影響を抑えます。

```bash
docker run --rm -v $PWD:/app -v ~/ros2_data:/root/ros2_data mg_develop:latest bash -c \
  "source /opt/ros/humble/setup.bash; python3 /app/tools/scripts/calib_wheel_odom.py <bag> --slam-dir <SLAM 出力> -o wheel_odom_corrector.yaml"
```

| 推定するもの | 意味 |
| :--- | :--- |
| `k_v` | 並進のスケール (車輪半径) |
| `k_w` | 旋回のスケール (実効トレッド幅) |
| `yaw_bias_per_meter` | 走行距離あたりに曲がる量 [rad/m] (左右の車輪半径差) |
| `time_offset` | オドメトリの遅れ [s]。損失が最小になる遅れを探索する |
| `covariance_vx` / `covariance_vyaw` | EKF に入れる速度の共分散の目安。補正後の窓の平均速度の誤差 σ を `--inflate` 倍して二乗 |
| AMCL の `alpha1`〜`alpha4` | AMCL の動きのモデル (nav2 の差動モデル) のノイズの下限の目安。真値の隣り合うノードの間ごとに、補正後のオドメトリの誤差を、モデルの形 (回転・並進の二乗に比例する分散) に当てはめ、`--alpha-margin` 倍の標準偏差にした値 |

AMCL の `alpha` の目安は、そのまま設定する値ではなく、下限として扱います。
- 真値の SLAM はオドメトリを使って作るので、オドメトリの誤差が小さく見えます (特に、スキャンの特徴が少ない場所)。
- 区間ごとに独立なノイズとしての推定で、走行や路面で変わる偏り、滑りは含みません。
- 設定する値は、この下限より大きく、再生評価 (評価セットとオドメトリの滑りの注入) で確かめて決めます。

- 出力の YAML は `mg_drivers/params/wheel_odom_corrector.yaml` の形式で、そのまま補正ノードのパラメータになります。
- 推定値は前半・後半でも別々に出るので、安定しているか確認してください。路面や荷重が違う走行では、走行ごとに推定して差を見ます。
- 補正ノードを通した bag (補正済みの `/odom`) には使えません。
