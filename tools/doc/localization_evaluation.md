# 自己位置推定の評価

自己位置推定 (ホイールオドメトリ + AMCL + GNSS の EKF 融合) を、rosbag から評価する `eval_localization.py` と、
パラメータの変種を rosbag の再生で比べる `run_localization_variant.sh`。実機は使わない。
概要は [tools の README](../README.md)、自己位置推定の構成は [mg_navigation](../../mg_navigation/doc/localization.md)。

## `eval_localization.py` (自己位置推定の評価)

オドメトリ + AMCL + GNSS を EKF で融合した自己位置推定の出力を rosbag から評価します。
実機のナビ走行の bag と、再生評価の出力 bag のどちらにも使えます。

```bash
# 真値なし (滑らかさ・EKF と AMCL の挙動)
make bag-eval-localization BAG=/root/ros2_data/rosbag/<bag> TO_TOOLS=1
# 真値あり (評価 bag の SLAM 出力)
make bag-eval-localization BAG=<bag> GT_DIR=<評価 bag の SLAM 出力>
# 別走行の評価: 地図を作った走行の SLAM 出力も渡し、真値をその座標系へ移す
make bag-eval-localization BAG=<bag> GT_DIR=<評価 bag の SLAM 出力> MAP_GT_DIR=<地図の SLAM 出力>
```

推定姿勢は `/tf` の `map→odom` と `odom→base_footprint` を合成して求めます (Nav2 が使う姿勢と同じ)。

| 指標 | 内容 |
| :--- | :--- |
| 滑らかさ | `map→odom` の更新でロボットの姿勢が飛ぶ大きさ (AMCL / GNSS の補正が Nav2 に与える影響) |
| 精度 | 真値 (`pose_graph.json`) との位置・yaw の誤差。全体と時間窓ごと |
| NEES | EKF の共分散が誤差に見合うか (自由度 3 の一貫した推定なら平均 3、95% が 7.8 以下) |
| EKF と odom | EKF の速度とホイールオドメトリ速度の差・遅れ、EKF の σ |
| AMCL | σ、遅延 (bag の記録時刻と header が同じ時計のときのみ)、届いた時点の推定姿勢との差 |
| 故障注入 (`FAULTS=`) | 故障ごとの最大誤差・復旧までの時間・検知までの時間、故障のない区間での誤検知の回数 |

- `--start` / `--end`: 評価する区間 (先頭からの経過秒)
- `--ok-threshold` / `--hold-sec`: 復旧とみなす位置誤差と、その誤差以下でいる時間
- `--map-frame` / `--odom-frame` / `--base-frame` / `--*-topic`: フレーム名とトピック名の変更

## `run_localization_variant.sh` (自己位置推定の再生評価)

自己位置推定のスタック (`map_server` / AMCL / GNSS ブリッジ / EKF) だけを起動し、rosbag のセンサデータ
(`/odom` `/scan_top_lidar` `/navpvt` `/gps/fix`) を再生して、推定結果を `eval_localization.py` で評価します。
パラメータを変えた「変種」を同じ条件で試して比べるためのものです。実機は使いません。

```bash
# 現状のパラメータでベースラインを測る (既定は短縮版データセット、3 回)
tools/scripts/run_localization_variant.sh baseline
# パラメータを書き換えた変種 (プレフィックス: ekf. / amcl. / bridge.)
tools/scripts/run_localization_variant.sh lag_on ekf.smooth_lagged_data=true ekf.history_length=1.0
tools/scripts/run_localization_variant.sh light_amcl amcl.max_particles=2000 amcl.max_beams=240 --cpus 4
# 本番のデータセット (10 分)
tools/scripts/run_localization_variant.sh baseline --dataset map043837_eval051635
# 変種同士を比べる
docker run --rm -v $PWD:/app -v ~/ros2_data:/root/ros2_data mg_develop:latest \
  python3 /app/tools/scripts/compare_localization.py <出力ディレクトリ1> <出力ディレクトリ2>
```

| オプション | 内容 |
| :--- | :--- |
| `--dataset <名前\|パス>` | `tools/datasets/localization/` の定義。既定 `map043837_eval051635_short` |
| `--runs N` | 繰り返す回数 (既定 3)。AMCL は乱数を使うため、分布で比べる |
| `--init gt\|gnss` | 初期姿勢。`gt` は真値を与える (既定)、`gnss` は GNSS による初期化に任せて初期化も評価する |
| `--initializer true\|false` | GNSS から AMCL の初期姿勢を与えるノード。既定は `--init gt` で `false`、`gnss` で `true` |
| `--monitor none\|watchdog\|supervisor` | 自己位置の監視ノード。既定は `--init gt` で `none`、`gnss` で `watchdog` |
| `--faults <yaml>` | センサ入力に故障を注入して、復旧を評価する (`tools/datasets/localization/faults/`) |
| `--start S` / `--duration D` | 評価する区間 (データセットの値を上書き) |
| `--rate R` | 再生速度 (既定 1.0)。処理が間に合わなくなるため実時間が基本 |
| `--cpus N` | コンテナが使える CPU 数。開発 PC は実機より速いため、実機相当に絞って AMCL の遅延を見る |
| `--ekf-file` `--nav2-file` `--bridge-file` `--odom-file` `--supervisor-file` | パラメータファイルを、リポジトリの既定ではなく指定したファイルから複製する (複数行のリストなど、1 行の書き換えでは変えられない変更に使う) |

書き換えるパラメータは `プレフィックス.キー=値` で指定します (キーはドット区切り)。

| プレフィックス | ファイル | ノード |
| :--- | :--- | :--- |
| `ekf.` | `mg_drivers/params/ekf_global.yaml` | `ekf_global_node` |
| `amcl.` | `mg_navigation/params/nav2_params.yaml` | `amcl` |
| `bridge.` | `slam_gnss_2d/slam_gnss_2d/params/nav_bridge.yaml` | `slam_gnss_nav_bridge` |
| `odom.` | `mg_drivers/params/wheel_odom_corrector.yaml` | `wheel_odom_corrector_node` |
| `sup.` | `mg_navigation/params/localization_supervisor.yaml` | `localization_supervisor_node` |

- 出力: `<評価 bag>/eval_loc/<データセット>/<名前>/{params/, run_N/, summary.md}`。`run_N/eval_localization.md` に試行ごとの評価、`summary.md` に試行をまとめた表が入ります。
- 起動前に、関連パッケージ (`slam_gnss_2d` `mg_msgs` `mg_bringup` `mg_navigation` `mg_drivers`) をコンテナ内で増分ビルドします。ソースの変更はイメージの再ビルドなしで反映されます。
- 書き換えられるのは既存のキーの値です。複数行にまたがるリスト (`odom0_config` など) も、`ekf.odom0_config="[true, true, ...]"` のように新しい値を 1 行で渡すとまとめて置き換えられます。キーが元のファイルに無い場合はエラーになります。
- 実機と同じく、ホイールオドメトリは補正ノードを通して `/odom/raw` → `/odom` になります。
- ROS の通信は `ROS_LOCALHOST_ONLY=1` とランダムな `ROS_DOMAIN_ID` で隔離しており、実機や他のコンテナと混ざりません。複数の変種を並列に実行できます (並列は 4 本まで。6 本並列では、スタックの起動が 180 s を超えて試行が無効になることがあります)。
- 初期姿勢が AMCL に届かなかった試行は無効として 1 回だけやり直します。
- 同じ変種名で再実行すると、その変種の `run_N` は上書きされます。

### 評価の進め方

1. **ベースラインを測る**: 変更前の状態で `run_localization_variant.sh baseline` を実行する。試行間のばらつきが分かる。
2. **1 つずつ変える**: 変種を 1 つのパラメータ (または 1 組の関連するパラメータ) だけ変えて実行する。複数を同時に変えると効果を分離できない。
3. **複数の試行で比べる**: AMCL は乱数を使い、条件によっては試行の約半数が破綻する (`movingstart`)。平均や中央値だけでなく、**破綻した試行の割合**
   (位置誤差の p95 が 3 m を超えた試行) を見る。効果の小さい差は、試行数 (`--runs`) を増やさないと判断できない。
4. **比較表を作る**: `compare_localization.py <変種のディレクトリ...>` (先頭が基準)。
5. **実機で確認する**: 採用する変更は、実機の走行でも同じコースを走り、記録した bag を `make bag-eval-localization` で評価して、再生の結果と食い違わないか確かめる。

注意すること:
- 再生を始める時点は、ロボットが**止まっている**ところを選ぶ (実運用は止まった状態で初期化する)。
- 並列に実行できるが、負荷が高いと AMCL の遅延が変わる。AMCL の遅延や CPU に関わる評価は、`--cpus` を指定して 1 つずつ実行する。
- 評価データの記録に不具合がないか確認する。051635 では、ホイールオドメトリの速度が 0 のまま姿勢だけ更新される期間があった (上記)。

### センサ故障の注入

`--faults` で、再生中のセンサ入力に故障を注入して、検知と復旧を評価できます (`fault_injector.py`)。
故障の定義は YAML で、時刻は再生開始からの経過秒です。

| 種類 | 内容 |
| :--- | :--- |
| `kidnap` | その時刻の推定姿勢をずらした姿勢を `/initialpose` に送り、AMCL だけをずらす |
| `gnss_bias` | `/navpvt` の位置にバイアスを乗せる (`carr_soln` で解の種類も変えられる) |
| `gnss_drop` | `/navpvt` を止める |
| `odom_scale` | 生のオドメトリの並進・旋回を拡大する (ホイールのスリップ。生じたずれはその後も残る) |
| `scan_drop` | LiDAR のスキャンの一部の角度を欠測にする |

評価レポートには、故障ごとの最大誤差・復旧までの時間・監督ノードの検知までの時間と、
**故障のない区間で状態が NORMAL 以外になった回数 (誤検知)** が出ます。

### データセット

`tools/datasets/localization/*.yaml` に、地図 (`localization_yaml` / `gnss_transform` / `slam_dir`)、評価する走行 (`bag` / `slam_dir`)、区間 (`start_offset` / `duration`) を定義します。

| データセット | 内容 |
| :--- | :--- |
| `same_run_043837` | 地図を作った走行をそのまま評価。手早いチェック用。地図がその走行に合わせ込まれているため楽観的な結果になる |
| `map043837_eval051635` | 別走行 (地図は 043837、評価は 051635 の 760〜1400 s)。本番用 |
| `map043837_eval051635_short` | 上の 760〜960 s だけ。動作確認と粗い比較用 |
| `map043837_eval051635_movingstart` | 051635 の 800〜980 s。**動いている最中から**始める厳しい条件で、AMCL が試行の約半数で進行方向にずれて固まる。復旧の仕組みの効果を、破綻した試行の割合で比べる用 |
| `map043837_eval051635_twist_glitch` | 051635 の 540〜720 s。ホイールオドメトリの速度が 0 のまま姿勢だけ更新される不具合区間 (下記) |

別走行では、評価する走行の SLAM 出力 (`pose_graph.json`) を、UTM 経由で地図の座標系へ移して真値にします。次の点に注意してください。
- 真値と地図の位置合わせの精度は、2 つの SLAM を作ったときの GNSS の精度で決まります。RTK Fix が少ない bag (051635 は Fixed 0%) では約 1 m ずれるため、絶対値ではなく**変種同士の相対比較**に使ってください。評価レポートには、走行全体を SE(2) 整合した後の誤差 (地図に対する整合性) を併記します。
- 評価する走行が、地図の範囲外を通る区間では AMCL が使えず破綻します。区間は地図がカバーする範囲に絞ってください (051635 は地図 043837 に対して 0〜138 s / 462〜1586 s / 1636〜1856 s が範囲内。範囲に入った直後の 462〜540 s も特徴が乏しく誤差が大きい)。
- **ホイールオドメトリの速度 (twist) が 0 になる不具合**: 051635 の 370〜730 s では、ドライバの速度が 0 のまま姿勢だけが更新されています (043837 は全区間で正常)。EKF は速度を観測として使うので、この区間では速度の共分散を小さくするほど破綻します。補正ノードの `twist_source: pose_diff` (姿勢の差分から速度を求める) で影響を受けなくなります。`odom.twist_source=pose_diff` で試せます。区間を選ぶときは `mg_drivers` の速度と姿勢の差分が一致しているか確認してください。
- 再生を始める位置は、評価する走行でロボットが**止まっている**時点を選んでください。動いている最中から始めると、初期化直後の収束前に走り出すことになり、実運用より厳しい条件で AMCL が破綻しやすくなります (051635 は 722〜810 s が停止中)。
- `--init gt` では、初期姿勢を与えた直後の 10 秒 (`EVAL_SKIP`) を評価から除きます。
