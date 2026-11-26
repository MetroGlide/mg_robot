# 自己位置推定の最終形 (MG、一部未実装)

このロボットの自己位置推定の、目指す構成。今の構成は [localization.md](./localization.md)、ロボットに依存しない枠組みの最終形は [localization_supervisor の architecture_target.md](../../localization_supervisor/doc/architecture_target.md)。
印 (実装済み / 一部 / 未実装 / 未確定 / 見送り) の意味は、枠組みの文書と同じ。数値はどれも「例」で、値は yaml と launch が正。

## ノードの構成

```
センサ ─► AMCL ─► [門] ─► EKF ─► map→odom ─► Nav2
                   ▲                ▲
                   │                └─ GNSS (slam_gnss_nav_bridge。分散の倍率を受ける)
監督のノード (localization_supervisor_node) ─ 門の開閉・置き直し・一時停止・減速・状態 ◄─► 走行側の BT・シーケンサ
```

| ノード | 最終形での役割 | 状況 |
| :--- | :--- | :--- |
| `localization_supervisor_node` | 門・照合・判定・復旧・未初期化の処理をまとめて持つ。置き直し (`/initialpose`・`/set_pose`) を自動で出すのは、このノードだけ | 実装済み (`localization_gate:=true` のとき。既定は門なし) |
| `motion_gate_node` | 監督に統合した後に消す | 実装済み (消した) |
| `gnss_amcl_initializer_node` | GNSS の周りを向きの全周で照合する部分は、監督の未初期化の処理の候補の部品にする。ノードとしては消す | 一部 (照合は枠組みの部品 `core/initial_search.py` にし、監督の門が使う。門があるときはノードを起動しない。構成 `legacy` では使う) |
| `amcl_watchdog_node` | 構成 `legacy` だけで使う | 実装済み |
| `amcl_gate_arbiter` | AMCL の門の要求をまとめる。要求元は監督だけになる | 実装済み |
| `manual_pose_relay` | 監督がいない構成で、Web UI の手動の初期姿勢を `/initialpose` へ素通しする | 実装済み |
| `ekf_global_node` | 走らせる推定器 (既定)。入力に LiDAR オドメトリ・IMU・カメラの動きを設定で足せる | 一部 (入力は今と同じ) |

## 機能 × センサ

| 機能 | LiDAR | カメラ (深度) | GNSS | ホイール | IMU |
| :--- | :--- | :--- | :--- | :--- | :--- |
| 地図との照合 (絶対) | AMCL、スキャンの照合 | 深度から作ったスキャンの照合 (見送り) | — | — | — |
| 地図座標の位置 | — | — | GNSS の位置 | — | — |
| 動き | LiDAR オドメトリ (見送り) | カメラの動き (見送り) | GNSS の差分 | ホイール | 角速度 (見送り) |
| 確かめ (「確かめた」を出す) | スキャンの照合 | 深度から作ったスキャンの照合 (見送り) | RTK Fix で、動きと整合し続けるとき (見送り) | — | — |
| 復旧・初期化の観測 | スキャン | 深度 (見送り) | 候補 | 動き | 動き |
| コストマップの障害物 | スキャン | 深度の点群 | — | — | — |
| 衝突監視 (`collision_monitor`) | スキャン | 深度の障害物の点 (見送り。LiDAR なしではコストマップの深度の障害物で走る) | — | — | — |

- カメラ・IMU・LiDAR オドメトリは、本番の前には足さない。LiDAR を失った後は精度を追わず走りを続ける判断で、カメラと IMU は実データで確かめられないため。理由と再開の条件は構想 ([localization_without_lidar.md](../../doc/ideas/localization_without_lidar.md)、[localization_graph_aggregator.md](../../doc/ideas/localization_graph_aggregator.md))。

## 置き直しの持ち主と手動の経路

- 自動の置き直しを出すのは監督だけにする。監督ノードの中の門 (`localization_gate:=true`) では、監督だけが出す (実装済み)。門なしの構成 (初期化ノード・watchdog) も戻し方として残っていて、`localization_monitor` と `localization_gate` で選ぶ。
- 手動の経路は残す ([localization.md の手動の初期姿勢](./localization.md#手動の初期姿勢))。Web UI は専用のトピック、RViz は `/initialpose`。人が与える姿勢は運用と調べ用で、復旧を人に頼る経路ではない。

## 起動の流れ (一部)

| 起動 | 流れ |
| :--- | :--- |
| 離れた所で起動 | 走りながら、GNSS の周りを向きの全周で照合し、決まるまで減速する |
| スタート地点で初期姿勢を与える | 与えた姿勢を候補に、その周りで確かめる |

- どちらも、監督の未初期化の処理 1 か所で決める。初期化ノードの全周の照合は、門の起動の直後の確かめと同じ規則で比べる候補の 1 つにする (実装済み。[motion_gate.md](../../localization_supervisor/doc/motion_gate.md#未初期化の処理-全周の照合があるとき))。
- 一致率が下限をわずかに超えるだけの決め方は通さない。今は「推定器と別の仮説に置き直すのは、全周の照合と同じ仮説に決まったときだけ」にしている (仮。[根拠](#根拠))。
- 決まるまで減速するのは未実装 (今は走りながら照合する)。
- 地図の切り替え (地図の版が変わったとき) の後も、同じ処理で確かめる (実装済み。`map_loader` の `loaded_maps` を受ける形ではなく、地図のトピックの版で見る)。

## GNSS の扱い

- 監督の GNSS の判定と、疑った GNSS の分散を広げる対処 (`variance_scale`。`slam_gnss_nav_bridge` が倍率を受ける) は、門を統合した後も残す (実装済み)。
- 門を閉じている間、EKF はホイールと GNSS だけで進む。GNSS の門 (GNSS の動いた量とオドメトリの比較、申告の精度が急に良くなったときの見習い期間) は未実装。
- 門は、監督が GNSS を疑っている間 (切り離し、または残差の倍率が大きい間)、GNSS を使う閉じる理由の部品と GNSS の候補を使わない (実装済み)。
- AMCL のプリセット `robust` (EKF が外れた AMCL を棄却する) は監督と組む。監督ノードの中の門と組める (実装済み。構成 `supervised`)。

## 走行側の BT との受け渡し (一部)

- `IsLocalizationHealthy` ([bt_plugins.md](./bt_plugins.md#islocalizationhealthy)) は、状態が `NORMAL` かではなく、直近に確かめたかを見る。
- 監督が止めている間は、BT の後退・首振りを止める。
- 計画の失敗 (始点が占有にある) とめり込み (ロボットの形が計画用地図の通れないセルにかかる) のとき、BT から監督に確かめの要求を出す ([枠組みの確かめの要求](../../localization_supervisor/doc/architecture_target.md#確かめの要求-実装済み))。めり込みからの脱出の前は実装済み (`VerifyLocalization`、[bt_plugins.md](./bt_plugins.md#verifylocalization))。
- 変えるのは `mg_navigation` の BT のプラグインと、`mg_waypoint_navigation` の XML。

## センサの組 (一部)

| 組 | 入る条件 | 走り方 |
| :--- | :--- | :--- |
| 全部 | 既定 | 通常 |
| LiDAR なし | `driver_watchdog_node` が LiDAR を起動し直しても戻らない状態が続いたとき。それまでは今どおり一時停止して起動し直しを試す ([driver_watchdog.md](../../mg_drivers/doc/driver_watchdog.md)) | 組の速度の上限で走り続ける。地図との照合はしない。精度は追わず、ずれは人が見て止める |
| カメラなし | RealSense が出ない | LiDAR で今と同じ |

- LiDAR なしの組で許す動き (前進だけ・後退の禁止など) の差し替えと、`collision_monitor` の入力を深度の障害物の点にすることは見送り (走りを続けることを優先する判断)。
- LiDAR が戻ったら、照合で確かめてから全部の組に戻る。
- 実装済み: LiDAR なしの組への引き継ぎ (監督の `sensor_sets` の `no_lidar`、対処 `sensor_set` が collision_monitor の scan の途絶による停止とセンサの途絶の見張りの LiDAR の一時停止を外す)、組の速度の上限。障害物は深度 (ローカルコストマップの 3D の障害物検出の層) で見る (2026-11-26 ユーザー判断)。前進だけ・後退の禁止などの許す動きの組ごとの差し替えはまだ。

## 例外のファイル (見送り)

LiDAR なし・GNSS 弱のような、合格の条件を満たせない状況を例外として持つ仕組みは作らない。今はセンサの組の設定と、シナリオの既知の問題 (`known_issue`) で足りる。構想は [localization_exceptions.md](../../doc/ideas/localization_exceptions.md)。

## 構成と戻すルート (一部)

| 構成 | 走らせる推定器 | 門・判定・復旧 | 使い方 |
| :--- | :--- | :--- | :--- |
| `legacy` | EKF (今の設定) | v2.1 の watchdog だけ | 戻すルート (前回の実機走行の構成) |
| `observe` | EKF (今の設定) | 判定と記録だけ (門を閉じない・止めない・減速しない) | 実機で誤った判定の率を測る (実装済み) |
| `supervised` | EKF + 門 | 全部 | 最新の構成。launch の既定 (実装済み) |

- 比べる構成 (シミュレータ・再生) は `legacy` と `supervised` だけにする。窓つきの推定器は見送り (作るときは `supervised` の中のフラグとして足す)。
- 実機では `legacy` → `observe` → `supervised` の順に有効にする。
- どの部品も構成の名前 1 つから launch の引数を決める (`localization_config`、実装済み)。部品ごとに戻し方を散らさない。
- 等価性の試験: タグ v2.1 のコードとこのブランチの `legacy` で同じシナリオ (smoke と anomaly) を回し、完走・所要時間・真値との差が雑音の範囲で同じことを確かめる (未実装)。
- 範囲の注意: このブランチで v2.1 から変わった、自己位置推定と関係のない部分 (ホイールオドメトリの補正、シーケンサの変更など) は `legacy` でも戻らない。完全に戻すには、タグ v2.1 のコードを使う。

## 根拠

- 異常パターンのシナリオ (`anomaly`・`anomaly_holdout`・`anomaly_sensor` のタグ、各 3 回) で、門のノードだけの構成 (`LOCALIZATION_MONITOR=motion_gate`、プリセットなし、初期化ノードの照合あり) と構成 `supervised` の前身 (統合前、門なし) を比べた (2026-11-25。結果は `~/ros2_data/scenario_results/20261125_000544`・`20261125_000544b` と `20261118_222738`)。合格の目安 ([localization_failure_ledger.md](./localization_failure_ledger.md#異常パターンのシナリオの合格の目安)) を外したのは、門のノードの構成だけで、2 種類だった。
  - GNSS が弱る回: AMCL は正しいのに、EKF がずれた GNSS に付いていった。門のノードは GNSS を疑わない。→ 監督の GNSS の判定と分散の対処を残す。
  - 離れた所からの起動: 門のノードの起動の直後の確かめが、低い一致率で GNSS 寄りの誤った姿勢を決め、初期化ノードの正しい姿勢 (全周の照合) を上書きした。→ 未初期化の処理を 1 か所にし、全周の照合を候補にする。
- 門の照合の規則と戻し方の根拠は、記録の再生 ([tools の localization_evaluation.md](../../tools/doc/localization_evaluation.md)) と、シナリオ `gate_jump_driving`・`gate_drift_driving` ([scenarios の README](../../mg_scenario_test/scenarios/README.md))。
