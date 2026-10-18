# mg_localization_supervisor

自己位置推定の検証と復旧のフレームワーク [localization_supervisor](../localization_supervisor/README.md) を、このロボットの構成 (AMCL + GNSS + オドメトリを EKF で融合) で使うための設定・launch・部品。
自己位置推定の全体の構成は [mg_navigation/doc/localization.md](../mg_navigation/doc/localization.md)。

## 構成要素

| 種類 | 名前 | 内容 |
| :--- | :--- | :--- |
| launch | `launch/supervisor.launch.py` | ゲートの調停 (`amcl_gate_arbiter`)・GNSS による初期化・監督ノードを起動する。`mg_navigation` の `localization_launch.py` から include される |
| ノード | `gnss_amcl_initializer_node.py` | GNSS の位置から、AMCL (`/initialpose`) と EKF (`/set_pose`) の初期姿勢を出す ([下記](#gnss_amcl_initializer_node)) |
| 設定 | `config/supervisor.yaml` | 監督ノード (`localization_supervisor_node`)。各値の説明はファイルのコメント |
| 設定 | `config/gate_arbiter.yaml` | AMCL のゲートの調停 (`amcl_gate_arbiter`) |
| 設定 | `config/initializer.yaml` | GNSS による初期化 (`gnss_amcl_initializer_node`) |

GNSS による初期化と監督ノードは、他のノードが立ち上がるのを待って遅れて起動する (初期化ノードを先に、監督ノードを後に起動する)。

### supervisor.launch.py の引数

| 引数 | 内容 |
| :--- | :--- |
| `use_sim_time` | シミュレーション・rosbag 再生の時刻を使う |
| `use_gnss_amcl_initializer` | GNSS による初期化ノードを起動するか |
| `use_supervisor` | 監督ノードを起動するか (`localization_launch.py` は `localization_monitor:=supervisor` のときに true を渡す) |
| `supervisor_params_file` | 監督ノードのパラメータファイル |

## AMCL のゲートの調停 (amcl_gate_arbiter)

フレームワークの `gate_arbiter_node.py` を、AMCL の出力を EKF に入れるゲート (`amcl_publish_controller_node`) の調停に使う。規則は [localization_supervisor/doc/gate_arbiter.md](../localization_supervisor/doc/gate_arbiter.md)。

| サービス (SetBool。data=true で入れる) | 要求元 |
| :--- | :--- |
| `/amcl_gate_arbiter/waypoint/change_publish_state` | ウェイポイントの `amcl_on` / `amcl_off` |
| `/amcl_gate_arbiter/supervisor/change_publish_state` | 監督ノードの切り離し・復帰 |

- ウェイポイントが切った区間では、監督ノードが復旧しても AMCL は戻らない。監督ノードが切っている間は、`amcl_on` でも戻らない。
- `amcl_off` の後に `amcl_on` を送るのは、ウェイポイントの作り手の責任 (送り忘れると AMCL は戻らない)。
- 監視ノードの種類 (`none` / `watchdog` / `supervisor`) によらず、常に起動する。負荷はごく小さい。
- 状態 `/amcl_gate_arbiter/state` は、Web UI の「Actions」が表示に使う。Web UI の「Actions」の AMCL の入/切は、`waypoint` の要求元として呼ぶ。次のウェイポイントのアクションで上書きされる。
- 診断 `amcl_gate_arbiter: gate` は、集約で `/Robot/Localization/Gate`。監督ノードが止めているときのイベント `gate.changed` は WARN ([doc/monitoring.md](../doc/monitoring.md))。

## 監督ノード (localization_supervisor_node)

検知・状態遷移・復旧は [localization_supervisor/doc/supervisor_node.md](../localization_supervisor/doc/supervisor_node.md)。このロボットでの使い方:

- `localization_monitor:=supervisor` で起動する (既定の監視は `mg_navigation` の `amcl_watchdog_node`)。
- 地図との照合には、上部 LiDAR のスキャンを使う (`config/supervisor.yaml` の `scan_topic`)。
- 診断 `localization_supervisor_node: localization` は、集約で `/Robot/Localization/Monitor`。

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

- `test_config.py`: 設定ファイルが読み込め、起動するノードの名前・ゲートの要求元・監督ノードのゲートのサービス名が互いに合っている
- `test_event_format.py`: フレームワークが出すイベントが、このプロジェクトのイベントの規約 ([doc/monitoring.md](../doc/monitoring.md)) と同じ書式で、`mg_utils.event_log.parse_event` で読める

## 依存

[package.xml](./package.xml) を参照。
