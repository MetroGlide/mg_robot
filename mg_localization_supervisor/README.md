# mg_localization_supervisor

自己位置推定の検証と復旧のフレームワーク [localization_supervisor](../localization_supervisor/README.md) を、このロボットの構成 (AMCL + GNSS + オドメトリを EKF で融合) で使うための設定・launch・部品。
自己位置推定の全体の構成は [mg_navigation/doc/localization.md](../mg_navigation/doc/localization.md)、目指す最終形 (門と初期化の監督への統合、センサの組、例外のファイル。一部未実装) は [localization_target.md](../mg_navigation/doc/localization_target.md)。blame・vouch・切り離しなどの用語は [用語集](../localization_supervisor/doc/glossary.md)。

## 構成要素

| 種類 | 名前 | 内容 |
| :--- | :--- | :--- |
| launch | `launch/supervisor.launch.py` | ゲートの調停 (`amcl_gate_arbiter`)・GNSS による初期化・監督ノード・手動の初期姿勢の中継を起動する。`mg_navigation` の `localization_launch.py` から include される |
| ノード | `gnss_amcl_initializer_node.py` | GNSS の位置から、AMCL (`/initialpose`) と EKF (`/set_pose`) の初期姿勢を出す ([下記](#gnss_amcl_initializer_node)) |
| 設定 | `config/supervision.yaml` | 監督の設定 (情報源・判定のインスタンス・総合・状態遷移・アンカー・安全の信号・出力)。監督ノードとオフライン評価が読む ([下記](#監督の構成-supervisionyaml)) |
| 設定 | `config/profiles/*.yaml` | `supervision.yaml` に重ねるプロファイル |
| 部品 | `mg_localization_supervisor/event_sinks/event_log.py` | 監督のイベントを、このプロジェクトのイベントログ (`mg_utils.event_log`) で出す (`events.sink`) |
| 部品 | `mg_localization_supervisor/actuators/` | 自動復旧の対処: シーケンサの一時停止の要求 (`pause_request.py`)、観測を増やす首振り (`spin_observe.py`)、再初期化の後のコストマップの消去 (`clear_costmap.py`) |
| 設定 | `config/gate_arbiter.yaml` | AMCL のゲートの調停 (`amcl_gate_arbiter`) |
| 設定 | `config/initializer.yaml` | GNSS による初期化 (`gnss_amcl_initializer_node`) |

GNSS による初期化と監督ノードは、他のノードが立ち上がるのを待って遅れて起動する (初期化ノードを先に、監督ノードを後に起動する)。

### supervisor.launch.py の引数

| 引数 | 内容 |
| :--- | :--- |
| `use_sim_time` | シミュレーション・rosbag 再生の時刻を使う |
| `use_gnss_amcl_initializer` | GNSS による初期化ノードを起動するか |
| `use_supervisor` | 監督ノードを起動するか (`localization_launch.py` は `localization_monitor:=supervisor` か `localization_supervisor:=true` のときに true を渡す) |
| `use_supervisor_gate` | 監督ノードの中で門を使うか (`true` / `false` / `observe`。`use_supervisor` と組。`localization_launch.py` は `localization_gate` を渡す)。`true` なら、GNSS による初期化ノードは起動しない (起動の直後の姿勢も門が決める)。`true` でなければ、手動の初期姿勢をそのまま `/initialpose` へ送る中継を起動する。`observe` は判定と照合だけ (ノードの `gate_observe_only`) |
| `supervisor_config` | 監督の設定ファイル (既定は `config/supervision.yaml`) |
| `localization_profile` | 監督の設定に重ねるプロファイル (空は基本設定) |

## AMCL のゲートの調停 (amcl_gate_arbiter)

フレームワークの `gate_arbiter_node.py` を、AMCL の出力を EKF に入れるゲート (`amcl_publish_controller_node`) の調停に使う。規則は [localization_supervisor/doc/gate_arbiter.md](../localization_supervisor/doc/gate_arbiter.md)。

| サービス (SetBool。data=true で入れる) | 要求元 |
| :--- | :--- |
| `/amcl_gate_arbiter/waypoint/change_publish_state` | ウェイポイントの `amcl_on` / `amcl_off` |
| `/amcl_gate_arbiter/supervisor/change_publish_state` | 監督の切り離し・復帰 (自動復旧の対処 `gate` が呼ぶ。`recovery_enabled` のとき、AMCL が疑わしいあいだ閉じる)。監督ノードの中の門があるときは、門の開閉 (閉じる理由の部品が異常を出してから、照合で決まるまで閉じる) |

- ウェイポイントが切った区間では、監督が復旧しても AMCL は戻らない。監督が切っている間は、`amcl_on` でも戻らない。
- `amcl_off` の後に `amcl_on` を送るのは、ウェイポイントの作り手の責任 (送り忘れると AMCL は戻らない)。
- 監視ノードの種類 (`none` / `watchdog` / `supervisor`) によらず、常に起動する。負荷はごく小さい。
- 状態 `/amcl_gate_arbiter/state` は、Web UI の「Actions」が表示に使う。Web UI の「Actions」の AMCL の入/切は、`waypoint` の要求元として呼ぶ。次のウェイポイントのアクションで上書きされる。
- 診断 `amcl_gate_arbiter: gate` は、集約で `/Robot/Localization/Gate`。監督ノードが止めているときのイベント `gate.changed` は WARN ([doc/monitoring.md](../doc/monitoring.md))。

## 監督ノード (localization_supervisor_node)

パラメータ・入出力・イベントは [localization_supervisor/doc/supervisor_node.md](../localization_supervisor/doc/supervisor_node.md)。このロボットでの使い方:

- `localization_supervisor:=true` で、既定の監視 (`mg_navigation` の `amcl_watchdog_node`。共分散が大きければ GNSS で初期化し直す) のまま、監督ノードを並行して動かす。状態 `/localization/status` は監督ノードが出す (watchdog は出さない)。実機で通知の質を確かめるときに使う。watchdog と二重に再初期化しないよう、プロファイルは `notify_only` になる。
- `localization_monitor:=supervisor` では、watchdog の代わりに監督ノードだけを動かし、基本設定の自動復旧を使う。watchdog と並行して動かすときは、プロファイル `notify_only` になる (二重に再初期化しない)。
- 設定は `supervisor_config`、プロファイルは `localization_profile` (既定は構成の値) で渡す。走行中は `ros2 service call /localization_supervisor_node/set_profile localization_supervisor_msgs/srv/SetProfile "{profile: gnss_fallback}"` で切り替えられる。
- GNSS による初期化 (`gnss_amcl_initializer_node`)・watchdog・人による `/initialpose`・`/set_pose` は、外からの再初期化として扱う (状態を戻して猶予を置く)。
- `localization_gate:=true` (既定は構成の値) で、監督ノードの中で門 (`supervision.yaml` の `gate` 節) を使う。置き直し・AMCL の門・一時停止・起動の直後の姿勢 (GNSS の周りの全周の照合) を門が決め、監督の復旧も門の照合で決める (分担は [design.md](../localization_supervisor/doc/design.md#門を入れたとき))。初期化ノードと手動の初期姿勢の中継は起動しない。外からの `/initialpose` は、外からの再初期化ではなく、AMCL の飛びとして門が判定する。AMCL の門の要求元は監督 (`supervisor`) だけになる。
- `localization_gate:=observe` では、門は判定と照合だけをし、門を開けたまま、置き直しも止める要求も出さない。監督も通知だけになる (プロファイルが復旧を有効にしても)。初期化ノードと中継は起動する。構成 `observe` (watchdog と並走) で使う。
- 診断 `localization_supervisor_node: localization` は、集約で `/Robot/Localization/Monitor`。watchdog と並行して動かすときは、watchdog の診断も同じ集約に入る (悪いほうのレベルになる)。Web UI の「自己位置」のタイルに、状態・疑わしい情報源・異常の判定・障害物の中かが出る。
- シーケンサを通らないゴール (手動のゴール) は、自動復旧の一時停止 (`pause_request`) でも止まらない。

### 手動の初期姿勢

| 送り手 | 送り先 | 監督ノードの中の門があるとき | ないとき |
| :--- | :--- | :--- | :--- |
| Web UI | `/localization/manual_initial_pose` | 門が周りを照合してから置き直し、確かめた姿勢にする | 中継 (`manual_pose_relay`) がそのまま `/initialpose` へ送る (今の動作と同じ) |
| RViz (2D Pose Estimate) | `/initialpose` | AMCL の振る舞いとして門の判定を受ける (地図に合えば採られる) | 今の動作と同じ |

- 中継は、門がないとき、初期化ノードの有無 (`use_gnss_amcl_initializer`) に依らず起動する。
- RViz の送り先は変えていない。門で運用するときは、Web UI から与える。

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
| `amcl_split` | AMCL の粒子 (情報源 `amcl_particles`) が、離れた姿勢の仮説に割れているか。重み 0 で記録だけする |

安全の信号は、EKF の姿勢の footprint と計画用地図で求める。footprint は `mg_navigation/params/nav2_params.yaml` のコストマップと同じ (テストで確かめる)。

| プロファイル | 内容 |
| :--- | :--- |
| `notify_only` | 検知と通知だけにする (自動復旧しない)。判定は基本設定のまま。watchdog と並行して動かすときの既定 |
| `hypotheses` | 復旧の候補に、AMCL の粒子の仮説 (`hypotheses`) を足す。角で AMCL が誤った向きに収束した後も、正しい向きの粒子が少数残っていれば、その姿勢を候補にできる |
| `gnss_fallback` | AMCL と GNSS が乖離したら GNSS で引き戻すだけの動作 (以前の watchdog と GNSS による初期化と同じ)。AMCL と GNSS の食い違いだけを、続けて異常だった回数で判断し、候補は精度の良い GNSS が続いたときの位置だけ。評価も切り離しも一時停止もしない |
| `robust_recovery` | 止まったままでも確かめて戻り、決められなければ首を振って観測を増やす復旧 (下の「止まっているときの復旧」)。比べる構成 `supervised` で使う |
| `fault_injection` | シナリオ試験で、`/initialpose` から自己位置推定をずらす故障を注入するとき。`/initialpose` を外からの再初期化として扱わない (`/set_pose` は扱う)。シナリオでは `localization_test_profile` で、比べるプロファイルの後に重ねる。実機では使わない |

### 自動復旧の構成

`supervision.yaml` の `recovery` (理由と値はファイルのコメント)。

| 項目 | 内容 |
| :--- | :--- |
| 候補を作るもの | `gnss` (精度の良い測位が続いたら、その位置)、`dead_reckoning` (疑いの前のアンカーにオドメトリを足した位置)、`submap` (その周りでスキャンを地図に照合した位置)。`hypotheses` (AMCL の粒子の仮説) は、プロファイル `hypotheses` で足す |
| 候補の評価 | スキャンの一致率を主に、GNSS・推測航法との近さを小さい重みで足す。計画用地図の致死セルに footprint がかかる候補は捨てる |
| 選び方 | 最良の評価が最低の評価に届かない、または次点との差が小さい (似た場所の取り違えの疑い) ときは、再初期化せず `DEGRADED` で知らせる |
| 再初期化の送り先 | AMCL (`/initialpose`) と EKF (`/set_pose`)。監督が送ったものは、外からの再初期化として扱わない |
| 対処 | AMCL のゲートを閉じる (`gate`。`/amcl_gate_arbiter/supervisor/...`)、シーケンサを一時停止する (`pause_request`)、観測を増やす指示でその場で首を振る (`spin_observe`)、再初期化の後にグローバルコストマップを消す (`clear_costmap`)、GNSS の分散の倍率を配信する (`variance_scale`) |

### 止まっているときの復旧 (robust_recovery)

止めている間は、AMCL は `update_min_d`・`update_min_a` だけ動くまで更新されず、GNSS のブリッジも `min_publish_distance` だけ動くまで出さない。基本設定の収束の確認 (`amcl_ekf`) は「判定できない」のまま時間切れになり、確かめないまま戻るか、候補を選べないまま止まり続けることがあった ([異常パターンの台帳](../mg_navigation/doc/localization_failure_ledger.md))。プロファイル `robust_recovery` は、止まっていても確かめられる形にする ([設計](../localization_supervisor/doc/design.md#確かめて戻る-止まっているとき))。

| 状態 | 走行 | 次へ |
| :--- | :--- | :--- |
| `NORMAL`・`SUSPECT` | 走る | |
| `ISOLATED` (GNSS だけ) | 走る (GNSS の分散を上限に) | GNSS の異常がなくなれば `NORMAL` |
| `ISOLATED` (AMCL・不明) | 止める | 待ってから `RECOVERING` |
| `RECOVERING` | 止める | 照合が合えば `NORMAL`。候補を選べない・照合が合わない・判定できない時間が続けば首振り |
| `RECOVERING`・首振り | 止めたまま、その場で回る (行って逆向きに戻る) | 動いた後に候補を作り直し、照合し直す |
| `DEGRADED` | 止める | 一定の間隔で `RECOVERING` (首振りから) に戻り続ける。確かめないまま `NORMAL` には戻らない |

- 収束の確認 `recover_scan`: EKF の姿勢 (復旧中は AMCL を切り離しているので、撒いた位置 + オドメトリ。止まっていても出る) で、今のスキャンを地図に照合する。再初期化で正常なときの基準を捨てるので、一致率と利得 (ずらしたほうが合う量) を絶対値で決める。値はシミュレータの地図での例で、実機の地図では正しい姿勢での分布を確かめてから決める。
- 収束の確認 `recover_scan_amcl`: AMCL の姿勢で同じ照合をする。撒き直しの候補を選べないまま EKF が固まっても (AMCL は切り離し中、GNSS は止まっていると出ない)、AMCL が自分で正しい姿勢に戻っていれば確かめて戻す。2 つの確認のどちらかが合えば戻る (`convergence_mode: any`)。AMCL は止まると更新しないので、最後の姿勢をオドメトリで今に移して使う。残る危険: AMCL が地図の似た場所に収束していると、照合だけでは見分けられない。
- 首振り `spin_observe`: Nav2 の behavior_server の Spin を呼ぶ。止めている間はシーケンサがゴールを取り消しているので、BT の Spin と競合しない。回転だけで並進しない。
- 残る危険: Spin の衝突の確認は behavior_server の局所コストマップの予測 (`simulate_ahead_time`) だけで、collision_monitor を通らない。
- 一度撒いた後は、動いて観測が増えるまで撒き直さない。長く止まっても推測航法の候補を作れるよう、アンカーを残す時間を延ばす。
- 処理の負荷: 正常なときは変わらない。復旧の間だけ、照合 1 回 (スキャン 1 枚) と Spin が増える。
- 戻し方: プロファイルを外す (基本設定)。首振りだけ止めるときは `recovery.observe.request_after_sec: 0`。

### GNSS の信頼度

地図と GNSS のずれは場所ごとに違う (地図は SLAM から作るため)。`supervision.yaml` の `residuals` が、信頼できる周期の GNSS と EKF の位置の差から、GNSS の自己申告の分散より実際の誤差が大きいかを調べ、分散にかける倍率を出す ([設計](../localization_supervisor/doc/design.md#残差と分散の倍率))。
対処 `variance_scale` が `/gnss_variance_scale` に配信し、`slam_gnss_nav_bridge` が `/odom/gps` の分散にかける ([nav_bridge.md](../slam_gnss_2d/doc/nav_bridge.md#外からの分散の倍率))。GNSS を切り離しているあいだは倍率が上限になる。
倍率が大きい状態が続くと、イベント `localization.residual_changed` で知らせる (`gnss_transform.yaml` を見直すきっかけ)。止めたいときは、プロファイルで `residuals.gnss_vs_ekf.max_scale` を 1 にする (切り離しのときも倍率は 1 になる。`gnss_fallback` はこうしている)。監督は、対処 `variance_scale` が出した倍率だけを割り戻して GNSS の誤差を見積もる。`slam_gnss_nav_bridge` の `variance_scale_topic` だけを空にすると、監督は倍率がかかったものとして割り戻し続け、GNSS の誤差を小さく見積もるので、ブリッジの購読を止めるときは、監督の対処 `variance_scale` も外す (設定から消す)。

### GNSS だけがずれたとき

AMCL と GNSS の食い違いだけでは、どちらが悪いかは決まらない。`submap_amcl` (AMCL の姿勢で、直近の走行のスキャンを地図に照合する判定) が正常と判定できているあいだは、AMCL は正しいと見なして (`vouch`)、GNSS を疑う。`scan_amcl` は AMCL と同じスキャンと地図で推定しているので独立ではなく、`vouch` しない。
GNSS が悪いと分かったとき (`state_machine.isolate_only`) は、止めず、AMCL と EKF の再初期化もせずに、GNSS の分散を上限にして使わないようにするだけにする。地図の歪みなどで、場所によっては GNSS が地図の座標と合わないことがあっても、正しい AMCL を切り離したり、走行を止めたりしない。GNSS の異常がなくなれば戻る。
`submap_amcl` が判定できていないとき (走り出して 5 m を走る前、止まっているなど) は、AMCL も疑う側に戻り、通常の対処 (ゲートを閉じ、止め、再初期化) になる。ただし、再初期化の前に GNSS のずれと分かれば、その時点で止めるのをやめる。

- 自動復旧が使える状況: GNSS の測位がある(運ばれたときも、その位置へ戻せる)、または、ずれが推測航法とスキャンの照合で直せる範囲(オドメトリのすべり・AMCL の少しずつのずれ)。GNSS がなく運ばれたときは、復旧できず `DEGRADED` で知らせる。
- 対処は起動時にすべて作り、プロファイルでは変えられない。`gnss_fallback` は、`request_hold` を偽、倍率の上限を 1 にして、一時停止の要求も倍率も出さない。以前の初期化ノードと違うのは、初期化し直した後にグローバルコストマップを消すところ。
- 戻し方: プロファイル `notify_only`(通知だけ)、`state_machine.request_hold: false`(止めない)、`state_machine.isolate_only: []`(GNSS のずれでも、通常の対処をする)、`gnss_fallback`(以前の GNSS で引き戻すだけの動作)、`localization_monitor:=watchdog`(従来の監視)。
- 部品は `mg_localization_supervisor/actuators/` (`pause_request.py`、`spin_observe.py`、`clear_costmap.py`)。
- 将来の拡張 (構想): [GNSS の場所ごとのずれを信頼度に反映する](../doc/ideas/gnss_map_distortion.md)、[AMCL を止めている区間を監督が知る](../doc/ideas/localization_gate_awareness.md)。

### オフライン評価

再生評価 (`tools/scripts/run_localization_variant.sh`) の記録を使って、判定と総合を実時間を待たずに評価する ([tools の README](../tools/README.md) の `eval_localization_offline.py`)。
しきい値・重み・`allowance` を変えたら、正常な記録 (誤検知) と故障注入 (検知までの時間・`blame`) で、変える前の設定の結果と同じか、それより良いことを確かめる。変える前の結果は、`--out` で名前を分けて残しておく。
新しい判定は、まず重み 0 (記録だけ) のプロファイルで、正しい姿勢での値の分布と故障のときの値を比べてから有効にする。

```bash
# 例: 採用している設定の再生の記録に kidnap を注入し、既定の設定と gnss_fallback を比べる
tools/scripts/eval_localization_offline.py <評価 bag>/eval_loc/<データセット>/<変種> \
    --faults tools/datasets/localization/faults/kidnap.yaml --profile '' --profile gnss_fallback
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
| `verify_by_scan` | GNSS の姿勢の周りで今のスキャンを地図に照合して、初期姿勢を確かめる (下記)。`config/initializer.yaml` で書く |
| `verify_map_topic`・`verify_scan_topic`・`verify_base_frame`・`verify_odom_frame` | 照合に使う地図 (AMCL と同じ地図)・スキャン・base の frame・照合の間に進んだ分を足すオドメトリの frame |
| `verify_retry_distance_m` | 合うまで、GNSS の位置がこれだけ動くごとに照合し直す |
| `external_check_delay_sec`・`external_search_range_m`・`external_search_yaw_rad` | 外から初期姿勢を与えられてから照合するまでの時間と、その姿勢の周りを探す範囲 |
| `external_accept_distance_m` | 外から与えられた初期姿勢に従うのは、起動してからこれだけ動くまで (スタート地点で止まって与える想定) |
| `verify.*` | 照合の範囲と刻み、一致とみなす距離 (粗い格子の `coarse_tolerance_m` と細かい格子の `tolerance_m`)、絞り込む候補の数、合うとみなす一致率 (`min_ratio`)、似た一致の候補との差 (`ambiguity_margin`)、合ったとき・確かめられないときの標準偏差 (`localization_supervisor` の `core/initial_search.py` の `InitialSearchConfig`。監督ノードの中の門の全周の照合と同じ部品) |

実際の値は `config/initializer.yaml` が正 (書いていないものはノードの既定値)。

GNSS の位置は、地図と GNSS の位置合わせのずれ (場所で変わる) を含む。ずれた位置へそのまま初期化すると、AMCL が自分で戻るまで、ずれたまま走る ([異常パターンの台帳](../mg_navigation/doc/localization_failure_ledger.md) の 9)。起動の直後は GNSS の向きも定まらず、スタート地点で止まって推論する時間もない。`verify_by_scan` が `true` のとき、初期姿勢は走りながら (遠隔操作でも自律でも) 決める。

- GNSS の精度が条件を満たしたら、GNSS の位置の周りで、向きは全周を、今のスキャンを地図に照合する (別のスレッドで動き、照合の間もスキャンと TF を受ける)。
- はっきり合う姿勢が 1 つ見つかれば、その姿勢を小さい共分散で送り、初期化を終える。照合したスキャンの時刻から送るまでに進んだ分は、オドメトリで足す。
- 合わない・似た一致の姿勢が離れて並ぶ・点や地図が足りないときは、最初の 1 回だけ、GNSS の姿勢を広い共分散で送る (AMCL の粒子を広く撒き、AMCL に選ばせる)。その後も、GNSS の位置が `verify_retry_distance_m` 動くごとに照合し直し、合えばその姿勢で上書きして終える。測位が悪い区間ではあきらめずに待つ (`max_consecutive_bad` は使わない)。
- 外から `/initialpose` を与えられたら (スタート地点の近くで起動し直したときなど)、それに従い、GNSS による初期化をやめる。`external_check_delay_sec` の後、与えられた姿勢の周りを狭く照合し、はっきり合った姿勢が離れていれば、その姿勢へ直す。合わない・決めきれないときは、与えられた姿勢のままにする (GNSS で上書きしない)。GNSS の照合で決まった後でも従う。起動してから `external_accept_distance_m` 動いた後の `/initialpose` (走行中の監督の復旧など) には関わらない。
- 照合の工夫 (根拠は、実機の地図とシミュレーションの地図での照合の試し):
  - 実機の地図は線が太く、植え込みは、点の散らばる幅 1〜2 m の帯として写る。広い許容距離では少しずれた姿勢も合ってしまうので、一致とみなす距離を狭くする。
  - 狭い許容距離のままだと、粗い格子では本当の姿勢の山を取りこぼす (向きの刻みの分だけ、遠い点が大きくずれる)。粗い格子だけ許容距離を広げて候補を多めに拾い、細かい格子で狭い許容距離で絞り込む。
  - 点を地図の外へ追い出す姿勢は、残った少ない点だけで合ったように見えるので、地図の外の点は不一致に数える。
- 結果はイベントで出す。
  - `localization.initial_pose_verified`: GNSS の周りの照合。`reason` (`matched`・`low_ratio`・`ambiguous`・`too_few_points`・`no_map`)、何回目か、送ったか、GNSS と照合の姿勢、一致率、次点との差。
  - `localization.initial_pose_external`: 外から初期姿勢を受けた。
  - `localization.initial_pose_external_checked`: その姿勢の周りの照合。直したか、ずれの大きさ、GNSS との差。
- 処理の負荷: 照合は、初期姿勢が決まるまで、GNSS の位置が一定の距離を動くごとに 1 回 (開発 PC で 1 回 0.1 s 程度、例)。決まった後と、正常な走行中は 0。
- 戻し方: `config/initializer.yaml` の `verify_by_scan: false`。
- 監督ノードの中で門を使うとき (`localization_gate:=true`) は、このノードを起動せず、同じ照合を門が行う ([motion_gate.md](../localization_supervisor/doc/motion_gate.md#未初期化の処理-全周の照合があるとき))。
- 地図を切り替えたときの照合は、まだしない (初期化ノードは地図の切り替えを受け取っていない)。

## テスト

```bash
make test pkg=mg_localization_supervisor
```

- `test_config.py`: 設定ファイルが読み込め、起動するノードの名前・ゲートの要求元が合っている
- `test_event_format.py`: フレームワークが出すイベントが、このプロジェクトのイベントの規約 ([doc/monitoring.md](../doc/monitoring.md)) と同じ書式で、`mg_utils.event_log.parse_event` で読める
- `test_supervision_config.py`: `supervision.yaml` とすべてのプロファイルがフレームワークの検証を通り、監督ノードで動かせる、入力のトピックが AMCL の設定と合う、footprint と計画用地図がコストマップと合う、イベントがプロジェクトのイベントログで出る
- `test_initializer_attempts.py`: 初期化の照合の、決まるまでの段取り (照合し直す間隔・外からの初期姿勢・照合の間に進んだ分)
- `test_recovery_config.py`: 自動復旧の構成(候補・評価・送り先・対処)、`pause_request` の変換、`gnss_fallback` が以前の初期化ノードと同じ条件・共分散で、すぐ GNSS の位置で初期化し直し、以前に無かった対処を無効にする、`variance_scale` のトピックが GNSS のブリッジの設定と合う、GNSS のずれは `submap_amcl` の `vouch` と `isolate_only` で切り離すだけにする

## 依存

[package.xml](./package.xml) を参照。
