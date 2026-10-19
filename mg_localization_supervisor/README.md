# mg_localization_supervisor

自己位置推定の検証と復旧のフレームワーク [localization_supervisor](../localization_supervisor/README.md) を、このロボットの構成 (AMCL + GNSS + オドメトリを EKF で融合) で使うための設定・launch・部品。
自己位置推定の全体の構成は [mg_navigation/doc/localization.md](../mg_navigation/doc/localization.md)。

## 構成要素

| 種類 | 名前 | 内容 |
| :--- | :--- | :--- |
| launch | `launch/supervisor.launch.py` | ゲートの調停 (`amcl_gate_arbiter`)・GNSS による初期化・監督ノードを起動する。`mg_navigation` の `localization_launch.py` から include される |
| ノード | `gnss_amcl_initializer_node.py` | GNSS の位置から、AMCL (`/initialpose`) と EKF (`/set_pose`) の初期姿勢を出す ([下記](#gnss_amcl_initializer_node)) |
| 設定 | `config/supervision.yaml` | 監督の設定 (情報源・判定のインスタンス・総合・状態遷移・アンカー・安全の信号・出力)。監督ノードとオフライン評価が読む ([下記](#監督の構成-supervisionyaml)) |
| 設定 | `config/profiles/*.yaml` | `supervision.yaml` に重ねるプロファイル |
| 部品 | `mg_localization_supervisor/event_sinks/event_log.py` | 監督のイベントを、このプロジェクトのイベントログ (`mg_utils.event_log`) で出す (`events.sink`) |
| 部品 | `mg_localization_supervisor/actuators/` | 自動復旧の対処: シーケンサの一時停止の要求 (`pause_request.py`)、再初期化の後のコストマップの消去 (`clear_costmap.py`) |
| 設定 | `config/gate_arbiter.yaml` | AMCL のゲートの調停 (`amcl_gate_arbiter`) |
| 設定 | `config/initializer.yaml` | GNSS による初期化 (`gnss_amcl_initializer_node`) |

GNSS による初期化と監督ノードは、他のノードが立ち上がるのを待って遅れて起動する (初期化ノードを先に、監督ノードを後に起動する)。

### supervisor.launch.py の引数

| 引数 | 内容 |
| :--- | :--- |
| `use_sim_time` | シミュレーション・rosbag 再生の時刻を使う |
| `use_gnss_amcl_initializer` | GNSS による初期化ノードを起動するか |
| `use_supervisor` | 監督ノードを起動するか (`localization_launch.py` は `localization_monitor:=supervisor` か `localization_supervisor:=true` のときに true を渡す) |
| `supervisor_config` | 監督の設定ファイル (既定は `config/supervision.yaml`) |
| `localization_profile` | 監督の設定に重ねるプロファイル (空は基の設定) |

## AMCL のゲートの調停 (amcl_gate_arbiter)

フレームワークの `gate_arbiter_node.py` を、AMCL の出力を EKF に入れるゲート (`amcl_publish_controller_node`) の調停に使う。規則は [localization_supervisor/doc/gate_arbiter.md](../localization_supervisor/doc/gate_arbiter.md)。

| サービス (SetBool。data=true で入れる) | 要求元 |
| :--- | :--- |
| `/amcl_gate_arbiter/waypoint/change_publish_state` | ウェイポイントの `amcl_on` / `amcl_off` |
| `/amcl_gate_arbiter/supervisor/change_publish_state` | 監督の切り離し・復帰 (自動復旧の対処 `gate` が呼ぶ。`recovery_enabled` のとき、AMCL が疑わしいあいだ閉じる) |

- ウェイポイントが切った区間では、監督が復旧しても AMCL は戻らない。監督が切っている間は、`amcl_on` でも戻らない。
- `amcl_off` の後に `amcl_on` を送るのは、ウェイポイントの作り手の責任 (送り忘れると AMCL は戻らない)。
- 監視ノードの種類 (`none` / `watchdog` / `supervisor`) によらず、常に起動する。負荷はごく小さい。
- 状態 `/amcl_gate_arbiter/state` は、Web UI の「Actions」が表示に使う。Web UI の「Actions」の AMCL の入/切は、`waypoint` の要求元として呼ぶ。次のウェイポイントのアクションで上書きされる。
- 診断 `amcl_gate_arbiter: gate` は、集約で `/Robot/Localization/Gate`。監督ノードが止めているときのイベント `gate.changed` は WARN ([doc/monitoring.md](../doc/monitoring.md))。

## 監督ノード (localization_supervisor_node)

パラメータ・入出力・イベントは [localization_supervisor/doc/supervisor_node.md](../localization_supervisor/doc/supervisor_node.md)。このロボットでの使い方:

- `localization_supervisor:=true` で、既定の監視 (`mg_navigation` の `amcl_watchdog_node`。共分散が大きければ GNSS で初期化し直す) のまま、監督ノードを並行して動かす。状態 `/localization/status` は監督ノードが出す (watchdog は出さない)。実機で通知の質を確かめるときに使う。watchdog と二重に再初期化しないよう、プロファイルは `notify_only` になる。
- `localization_monitor:=supervisor` (既定) では、watchdog の代わりに監督ノードだけを動かし、基の設定の自動復旧を使う。watchdog と並行して動かすときは、プロファイル `notify_only` になる (二重に再初期化しない)。
- 設定は `supervisor_config`、プロファイルは `localization_profile` で渡す。走行中は `ros2 service call /localization_supervisor_node/set_profile localization_supervisor_msgs/srv/SetProfile "{profile: gnss_fallback}"` で切り替えられる。
- GNSS による初期化 (`gnss_amcl_initializer_node`)・watchdog・人による `/initialpose`・`/set_pose` は、外からの再初期化として扱う (状態を戻して猶予を置く)。
- 診断 `localization_supervisor_node: localization` は、集約で `/Robot/Localization/Monitor`。watchdog と並行して動かすときは、watchdog の診断も同じ集約に入る (悪いほうのレベルになる)。Web UI の「自己位置」のタイルに、状態・疑わしい情報源・異常の判定・障害物の中かが出る。
- シーケンサを通らないゴール (手動のゴール) は、自動復旧の一時停止 (`pause_request`) でも止まらない。

## 監督の構成 (supervision.yaml)

フレームワークの中核 ([design.md](../localization_supervisor/doc/design.md)) に与える、このロボットの構成。書き方は [parameters.md](../localization_supervisor/doc/parameters.md)、各値の理由はファイルのコメント。

| 情報源 | 種類 | 入力 |
| :--- | :--- | :--- |
| `amcl` | pose | `/amcl_pose_origin` (ゲートを通る前の AMCL の推定) |
| `gnss` | pose | `/odom/gps` (map 座標の GNSS) |
| `ekf` | pose、融合の結果 (`amcl`・`gnss`・`odom` から) | `/ekf_global_odom` |
| `odom` | motion | `/odom` (補正したホイールオドメトリ) |
| `map_scan` | reference | `/map` と上部 LiDAR のスキャン |
| `planning_map` | reference (地図だけ) | `/planning_map` (グローバルコストマップの静的な層と同じ計画用地図。安全の信号に使う) |

判定のインスタンス (総合は `cusum`):

| インスタンス | 内容 |
| :--- | :--- |
| `amcl_gnss`、`amcl_jump`、`scan_ekf`、`amcl_ekf` | 以前の監督ノードの判定 (GNSS との食い違い、飛び、EKF の姿勢での地図との一致、AMCL と EKF の差) |
| `ekf_gnss` | EKF が引きずられたか |
| `scan_amcl` | AMCL の姿勢での地図との一致 (AMCL と独立でないので重みを下げる) |
| `amcl_covariance` | watchdog と同じ共分散の判定 |
| `submap_ekf`、`submap_amcl` | 直近の走行のスキャンをまとめた局所地図と、地図との照合。GNSS が使えず、AMCL が誤った場所に収束して EKF も引きずられたときに効く |
| `freespace_ekf`、`drift_ekf` | 定義だけ (無効)。この環境のオフライン評価で、正しい姿勢と区別できなかった (理由はファイルのコメント) |

安全の信号は、EKF の姿勢の footprint と計画用地図で求める。footprint は `mg_navigation/params/nav2_params.yaml` のコストマップと同じ (テストで確かめる)。

| プロファイル | 内容 |
| :--- | :--- |
| `notify_only` | 検知と通知だけにする (自動復旧しない)。判定は基の設定のまま。watchdog と並行して動かすときの既定 |
| `gnss_fallback` | AMCL と GNSS が乖離したら GNSS で引き戻すだけの動作 (以前の watchdog と GNSS による初期化と同じ)。AMCL と GNSS の食い違いだけを、続けて異常だった回数で判断し、候補は精度の良い GNSS が続いたときの位置だけ。評価も切り離しも一時停止もしない |
| `consecutive` | 以前の監督ノードと同じ判定・しきい値・回数。累積 (`cusum`) と比べるために使う |

### 自動復旧の構成

`supervision.yaml` の `recovery` (理由と値はファイルのコメント)。

| 項目 | 内容 |
| :--- | :--- |
| 候補を作るもの | `gnss` (精度の良い測位が続いたら、その位置)、`dead_reckoning` (疑いの前のアンカーにオドメトリを足した位置)、`submap` (その周りでスキャンを地図に照合した位置) |
| 候補の評価 | スキャンの一致率を主に、GNSS・推測航法との近さを小さい重みで足す。計画用地図の致死セルに footprint がかかる候補は捨てる |
| 選び方 | 最良が最低の評価に届かない、または離れた次点との差が小さい(似た場所の取り違え)ときは、再初期化せず `DEGRADED` で知らせる |
| 再初期化の送り先 | AMCL (`/initialpose`) と EKF (`/set_pose`)。監督が送ったものは、外からの再初期化として扱わない |
| 対処 | AMCL のゲートを閉じる (`gate`。`/amcl_gate_arbiter/supervisor/...`)、シーケンサを一時停止する (`pause_request`)、再初期化の後にグローバルコストマップを消す (`clear_costmap`)、GNSS の分散の倍率を配信する (`variance_scale`) |

### GNSS の信頼度

地図と GNSS のずれは場所ごとに違う (地図は SLAM から作るため)。`supervision.yaml` の `residuals` が、信頼できる周期の GNSS と EKF の位置の差から、GNSS の自己申告の分散より実際の誤差が大きいかを調べ、分散にかける倍率を出す ([設計](../localization_supervisor/doc/design.md#残差と分散の倍率))。
対処 `variance_scale` が `/gnss_variance_scale` に配信し、`slam_gnss_nav_bridge` が `/odom/gps` の分散にかける ([nav_bridge.md](../slam_gnss_2d/doc/nav_bridge.md#外からの分散の倍率))。GNSS を切り離しているあいだは倍率が上限になる。
倍率が大きい状態が続くと、イベント `localization.residual_changed` で知らせる (`gnss_transform.yaml` を見直すきっかけ)。止めたいときは、`slam_gnss_nav_bridge` の `variance_scale_topic` を空にする (プロファイル `gnss_fallback` は倍率を 1 に固定する)。

### GNSS だけがずれたとき

AMCL と GNSS の食い違いだけでは、どちらが悪いかは決まらない。`submap_amcl` (AMCL の姿勢で、直近の走行のスキャンを地図に照合する判定) が正常と判定できているあいだは、AMCL は正しいと見なして (`vouch`)、GNSS を疑う。`scan_amcl` は AMCL と同じスキャンと地図で推定しているので独立ではなく、`vouch` しない。
GNSS が悪いと分かったとき (`state_machine.isolate_only`) は、止めず、AMCL と EKF の再初期化もせずに、GNSS の分散を上限にして使わないようにするだけにする。地図の歪みなどで、GNSS が地図の座標と場所によって合わないことがあっても、正しい AMCL を切り離したり、走行を止めたりしない。GNSS の異常がなくなれば戻る。
`submap_amcl` が判定できていないとき (走り出して 5 m を走る前、止まっているなど) は、AMCL も疑う側に戻り、通常の対処 (ゲートを閉じ、止め、再初期化) になる。ただし、再初期化の前に GNSS のずれと分かれば、その時点で止めるのをやめる。

- 自動復旧が使える状況: GNSS の測位がある(運ばれたときも、その位置へ戻せる)、または、ずれが推測航法とスキャンの照合で直せる範囲(オドメトリのすべり・AMCL の少しずつのずれ)。GNSS がなく運ばれたときは、復旧できず `DEGRADED` で知らせる。
- 対処は起動時にすべて作る。プロファイルで変えられるのは、働かせない対処 (`recovery.disabled_actuators`) だけ。`gnss_fallback` は、以前の初期化ノードに無かった対処 (`pause_request`・`clear_costmap`・`variance_scale`) を無効にする。
- 戻し方: プロファイル `notify_only`(通知だけ)、`state_machine.request_hold: false`(止めない)、`state_machine.isolate_only: []`(GNSS のずれでも、通常の対処をする)、`gnss_fallback`(以前の GNSS で引き戻すだけの動作)、`localization_monitor:=watchdog`(従来の監視)。
- 部品は `mg_localization_supervisor/actuators/` (`pause_request.py`、`clear_costmap.py`)。
- 将来の拡張 (構想): [GNSS の場所ごとのずれを信頼度に反映する](../doc/ideas/gnss_map_distortion.md)、[AMCL を止めている区間を監督が知る](../doc/ideas/localization_gate_awareness.md)。

### オフライン評価

再生評価 (`tools/scripts/run_localization_variant.sh`) の記録を使って、判定と総合を実時間を待たずに評価する ([tools の README](../tools/README.md) の `eval_localization_offline.py`)。
しきい値・重み・`allowance` を変えたら、正常な記録 (誤検知) と故障注入 (検知までの時間・`blame`) で、`consecutive` のプロファイル (以前の監督ノード) と同じか、それより良いことを確かめる。
新しい判定は、まず重み 0 (記録だけ) のプロファイルで、正しい姿勢での値の分布と故障のときの値を比べてから有効にする。

```bash
# 例: 採用している設定の再生の記録に kidnap を注入し、既定の設定と consecutive を比べる
tools/scripts/eval_localization_offline.py <評価 bag>/eval_loc/<データセット>/<変種> \
    --faults tools/datasets/localization/faults/kidnap.yaml --profile '' --profile consecutive
```

## gnss_amcl_initializer_node

`/odom/gps` (map 座標の GNSS の位置) の精度が、続けて条件を満たしたとき、`/initialpose` (AMCL) と、`publish_set_pose` が `true` なら `/set_pose` (EKF) に、初期の姿勢を出す。
EKF の初期共分散が大きく、AMCL の初期値 (原点) に引かれるので、EKF にも送る。サービス `~/request_reinit` (`std_srvs/Trigger`) で、内部の状態をリセットして、初期化をやり直す (`amcl_watchdog_node` と Web UI が呼ぶ)。

主なパラメータ。

| パラメータ | 内容 |
| :--- | :--- |
| `required_consecutive_good` | 条件を満たす測位が続く回数 |
| `max_position_std_m` | 使う測位の水平の標準偏差の上限 [m] |
| `max_vertical_std_m` | 同、鉛直 (`ignore_z_std` が `true` のときは見ない) |
| `use_fixed_heading` | 向きが得られないとき、固定の向き (`fixed_heading` [rad]) を使う |
| `override_pose_covariance` / `pose_covariance` | 初期姿勢の共分散を固定の値にする |
| `max_consecutive_bad` | 条件を満たさない測位がこの回数続いたら、初期化をあきらめる (`request_reinit` でやり直せる) |
| `odom_age_timeout_sec` / `ignore_odom_age` | 測位の古さの確認 |

実際の値は `config/initializer.yaml` が正 (書いていないものはノードの既定値)。

## テスト

```bash
make test pkg=mg_localization_supervisor
```

- `test_config.py`: 設定ファイルが読み込め、起動するノードの名前・ゲートの要求元が合っている
- `test_event_format.py`: フレームワークが出すイベントが、このプロジェクトのイベントの規約 ([doc/monitoring.md](../doc/monitoring.md)) と同じ書式で、`mg_utils.event_log.parse_event` で読める
- `test_supervision_config.py`: `supervision.yaml` とすべてのプロファイルがフレームワークの検証を通り、監督ノードで動かせる、入力のトピックが AMCL の設定と合う、footprint と計画用地図がコストマップと合う、イベントがプロジェクトのイベントログで出る
- `test_recovery_config.py`: 自動復旧の構成(候補・評価・送り先・対処)、`pause_request` の変換、`gnss_fallback` が以前の初期化ノードと同じ条件・共分散で、すぐ GNSS の位置で初期化し直し、以前に無かった対処を無効にする、`variance_scale` のトピックが GNSS のブリッジの設定と合う、GNSS のずれは `submap_amcl` の `vouch` と `isolate_only` で切り離すだけにする

## 依存

[package.xml](./package.xml) を参照。
