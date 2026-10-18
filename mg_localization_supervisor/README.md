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
| `/amcl_gate_arbiter/supervisor/change_publish_state` | 監督の切り離し・復帰 (自動復旧の対処で使う。今の監督ノードは通知だけなので呼ばない) |

- ウェイポイントが切った区間では、監督が復旧しても AMCL は戻らない。監督が切っている間は、`amcl_on` でも戻らない。
- `amcl_off` の後に `amcl_on` を送るのは、ウェイポイントの作り手の責任 (送り忘れると AMCL は戻らない)。
- 監視ノードの種類 (`none` / `watchdog` / `supervisor`) によらず、常に起動する。負荷はごく小さい。
- 状態 `/amcl_gate_arbiter/state` は、Web UI の「Actions」が表示に使う。Web UI の「Actions」の AMCL の入/切は、`waypoint` の要求元として呼ぶ。次のウェイポイントのアクションで上書きされる。
- 診断 `amcl_gate_arbiter: gate` は、集約で `/Robot/Localization/Gate`。監督ノードが止めているときのイベント `gate.changed` は WARN ([doc/monitoring.md](../doc/monitoring.md))。

## 監督ノード (localization_supervisor_node)

パラメータ・入出力・イベントは [localization_supervisor/doc/supervisor_node.md](../localization_supervisor/doc/supervisor_node.md)。今は検知と通知だけを行う。このロボットでの使い方:

- `localization_supervisor:=true` で、既定の監視 (`mg_navigation` の `amcl_watchdog_node`。共分散が大きければ GNSS で初期化し直す) のまま、監督ノードを並行して動かす。状態 `/localization/status` は監督ノードが出す (watchdog は出さない)。実機で通知の質を確かめるときに使う。
- `localization_monitor:=supervisor` にすると、watchdog の代わりに監督ノードだけを動かす (自動の再初期化はしない)。
- 設定は `supervisor_config`、プロファイルは `localization_profile` で渡す。走行中は `ros2 service call /localization_supervisor_node/set_profile localization_supervisor_msgs/srv/SetProfile "{profile: gnss_fallback}"` で切り替えられる。
- GNSS による初期化 (`gnss_amcl_initializer_node`)・watchdog・人による `/initialpose`・`/set_pose` は、外からの再初期化として扱う (状態を戻して猶予を置く)。
- 診断 `localization_supervisor_node: localization` は、集約で `/Robot/Localization/Monitor`。watchdog と並行して動かすときは、watchdog の診断も同じ集約に入る (悪いほうのレベルになる)。Web UI の「自己位置」のタイルに、状態・疑わしい情報源・異常の判定・障害物の中かが出る。
- シーケンサを通らないゴール (手動のゴール) は、将来の自動復旧の一時停止でも止まらない。

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
| `gnss_fallback` | AMCL と GNSS の食い違いだけを、続けて異常だった回数で判断する (AMCL と GNSS が乖離したら GNSS で引き戻すだけの動作に戻す。引き戻しは自動復旧で足す) |
| `consecutive` | 以前の監督ノードと同じ判定・しきい値・回数。累積 (`cusum`) と比べるために使う |

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

## 依存

[package.xml](./package.xml) を参照。
