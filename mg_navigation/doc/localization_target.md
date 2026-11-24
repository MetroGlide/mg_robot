# 自己位置推定の最終形 (MG、一部未実装)

このロボットの自己位置推定の、目指す構成。今の構成は [localization.md](./localization.md)、ロボットに依存しない枠組みの最終形は [localization_supervisor の architecture_target.md](../../localization_supervisor/doc/architecture_target.md)。
印 (実装済み / 一部 / 未実装 / 未確定) の意味は、枠組みの文書と同じ。数値はどれも「例」で、値は yaml と launch が正。

## ノードの構成

```
センサ ─► AMCL ─► [門] ─► EKF ─► map→odom ─► Nav2
                   ▲                ▲
                   │                └─ GNSS (slam_gnss_nav_bridge。分散の倍率を受ける)
監督のノード (localization_supervisor_node) ─ 門の開閉・置き直し・一時停止・減速・状態 ◄─► 走行側の BT・シーケンサ
```

| ノード | 最終形での役割 | 状況 |
| :--- | :--- | :--- |
| `localization_supervisor_node` | 門・照合・判定・復旧・未初期化の処理をまとめて持つ。置き直し (`/initialpose`・`/set_pose`) を自動で出すのは、このノードだけ | 一部 (門は `motion_gate_node` に別にある) |
| `motion_gate_node` | 監督に統合した後に消す | 実装済み (統合の前) |
| `gnss_amcl_initializer_node` | GNSS の周りを向きの全周で照合する部分は、監督の未初期化の処理の候補の部品にする。ノードとしては消す | 一部 (今は別に置き直しを出す) |
| `amcl_watchdog_node` | 構成 `current` だけで使う | 実装済み |
| `amcl_gate_arbiter` | AMCL の門の要求をまとめる。要求元は監督だけになる | 実装済み |
| `manual_pose_relay` | 監督がいない構成で、Web UI の手動の初期姿勢を `/initialpose` へ素通しする | 実装済み |
| `ekf_global_node` | 走らせる推定器 (既定)。入力に LiDAR オドメトリ・IMU・カメラの動きを設定で足せる | 一部 (入力は今と同じ) |

## 機能 × センサ

| 機能 | LiDAR | カメラ (深度) | GNSS | ホイール | IMU |
| :--- | :--- | :--- | :--- | :--- | :--- |
| 地図との照合 (絶対) | AMCL、スキャンの照合 | 深度から作ったスキャンの照合 (未確定) | — | — | — |
| 地図座標の位置 | — | — | GNSS の位置 | — | — |
| 動き | LiDAR オドメトリ (未実装) | カメラの動き (未確定) | GNSS の差分 | ホイール | 角速度 (未実装) |
| 確かめ (「確かめた」を出す) | スキャンの照合 | 深度から作ったスキャンの照合 (未確定) | RTK Fix で、動きと整合し続けるとき (未実装) | — | — |
| 復旧・初期化の観測 | スキャン | 深度 (未確定) | 候補 | 動き | 動き |
| コストマップの障害物 | スキャン | 深度の点群 | — | — | — |
| 衝突監視 (`collision_monitor`) | スキャン | 深度の障害物の点 (未確定) | — | — | — |

- RealSense (D435i) の深度は、走行中に周波数を変えられない。カメラを使う方式は、カメラを含む実走行の記録 (record_all) で比べてから選ぶ。
- IMU は RealSense のもの。記録の一覧 (`params/record_topic_list.txt`) とシミュレータに足してから評価する (未実装)。

## 置き直しの持ち主と手動の経路

- 自動の置き直しを出すのは監督だけにする。今は監督・門のノード・初期化ノード・watchdog のどれもが出せるので、`localization_monitor` で 1 つを選んでいる。それでも、門のノードと初期化ノードは同時に動く (一部)。
- 手動の経路は残す ([localization.md の手動の初期姿勢](./localization.md#手動の初期姿勢))。Web UI は専用のトピック、RViz は `/initialpose`。人が与える姿勢は運用と調べ用で、復旧を人に頼る経路ではない。

## 起動の流れ (一部)

| 起動 | 流れ |
| :--- | :--- |
| 離れた所で起動 | 走りながら、GNSS の周りを向きの全周で照合し、決まるまで減速する |
| スタート地点で初期姿勢を与える | 与えた姿勢を候補に、その周りで確かめる |

- どちらも、監督の未初期化の処理 1 か所で決める。初期化ノードの全周の照合は、門の起動の直後の確かめと同じ規則で比べる候補の 1 つにする (未実装)。
- 一致率が下限をわずかに超えるだけの決め方は通さない (条件は未確定。[根拠](#根拠))。
- 地図の切り替え (`map_loader` の `loaded_maps`) の後も、同じ処理で確かめる (未実装)。

## GNSS の扱い

- 監督の GNSS の判定と、疑った GNSS の分散を広げる対処 (`variance_scale`。`slam_gnss_nav_bridge` が倍率を受ける) は、門を統合した後も残す (実装済み)。
- 門を閉じている間、EKF はホイールと GNSS だけで進む。GNSS の門 (GNSS の動いた量とオドメトリの比較、申告の精度が急に良くなったときの見習い期間) は未実装。
- AMCL のプリセット `robust` (EKF が外れた AMCL を棄却する) は、今は監督とだけ組める。統合の後は、門のある構成で使えるようにする (未実装)。

## 走行側の BT との受け渡し (未実装)

- `IsLocalizationHealthy` ([bt_plugins.md](./bt_plugins.md#islocalizationhealthy)) は、状態が `NORMAL` かではなく、直近に確かめたかを見る。
- 監督が止めている間は、BT の後退・首振りを止める。
- 計画の失敗 (始点が占有にある) とめり込み (ロボットの形が計画用地図の通れないセルにかかる) のとき、BT から監督に確かめの要求を出す ([枠組みの確かめの要求](../../localization_supervisor/doc/architecture_target.md#確かめの要求-未実装))。
- 変えるのは `mg_navigation` の BT のプラグインと、`mg_waypoint_navigation` の XML。

## センサの組 (未実装)

| 組 | 入る条件 | 走り方 |
| :--- | :--- | :--- |
| 全部 | 既定 | 通常 |
| LiDAR なし | `driver_watchdog_node` が LiDAR を起動し直しても戻らない状態が続いたとき。それまでは今どおり一時停止して起動し直しを試す ([driver_watchdog.md](../../mg_drivers/doc/driver_watchdog.md)) | GNSS が RTK Fix の区間は確かめながら低速。弱い区間は縮退走行 |
| カメラなし | RealSense が出ない | LiDAR で今と同じ |

- LiDAR なしの組の安全: 前進だけで低速・遅い旋回。後退は禁止。首振りは周りに当たらない所だけ。移るときに、コストマップに残ったスキャンの障害物を消す。`collision_monitor` の入力を深度の障害物の点にするかは、深度の遅れと誤った停止の率を測ってから決める (未確定)。
- LiDAR が戻ったら、照合で確かめてから全部の組に戻る。

## 例外のファイル (未実装)

例外の一覧は `mg_localization_supervisor/config/` の例外のファイル (仮の名前 `exceptions.yaml`) を正とする。仕組みは [枠組みの例外](../../localization_supervisor/doc/architecture_target.md#例外-未実装)。最初の例外:

| 項目 | 中身 |
| :--- | :--- |
| 名前 | LiDAR なし・GNSS 弱 |
| 入る条件 | LiDAR なしの組で、GNSS が弱く、縮退走行の距離の上限を使い切った |
| 緩める不変条件 | 1 (ずれたまま走る距離) と 5 (確かめずに戻らない)。止まったままにしない条件は守る |
| 代わりの対処 | 最低の速度で前進だけ。推測で走り、照合か GNSS で決まったらすぐ確かめる。LiDAR の起動し直しは続ける。推測の不確かさが走路の幅を超えたら、前進をやめ、その場の旋回と照合をくり返す |
| 出る条件 | GNSS が良くなる、照合で確かめられる、LiDAR が戻る |

## 構成と戻すルート (一部)

| 構成 | 走らせる推定器 | 門・判定・復旧 | 使い方 |
| :--- | :--- | :--- | :--- |
| `current` | EKF (今の設定) | v2.1 の watchdog だけ | 戻すルート (前回の実機走行の構成) |
| `observe` | EKF (今の設定) | 判定と記録だけ (門を閉じない・止めない・減速しない) | 実機で誤った判定の率を測る (未実装) |
| `new` | EKF + 門 | 全部 | 目標の構成 |
| `new_window` | 窓つきの推定器 | 全部 (EKF は予備として並べる) | 走らせる推定器の比較 (未確定) |

- 実機では `current` → `observe` → `new` の順に有効にする。
- 今の `make/test.mk` の `LOCALIZATION_CONFIG` は `current`・`supervisor`・`new` で、`new` は監督 + プリセット `robust` + プロファイル `robust_recovery`。`supervisor` を消し、`observe` と `new_window` を足し、`new` を上の表の中身にする (未実装)。
- どの部品も構成の名前 1 つから launch の引数を決める。部品ごとに戻し方を散らさない。
- 等価性の試験: タグ v2.1 のコードとこのブランチの `current` で同じシナリオ (smoke と anomaly) を回し、完走・所要時間・真値との差が雑音の範囲で同じことを確かめる (未実装)。
- 範囲の注意: このブランチで v2.1 から変わった、自己位置推定と関係のない部分 (ホイールオドメトリの補正、シーケンサの変更など) は `current` でも戻らない。完全に戻すには、タグ v2.1 のコードを使う。

## 根拠

- 異常パターンのシナリオ (`anomaly`・`anomaly_holdout`・`anomaly_sensor` のタグ、各 3 回) で、門のノードだけの構成 (`LOCALIZATION_MONITOR=motion_gate`、プリセットなし、初期化ノードの照合あり) と構成 `new` を比べた (2026-11-25。結果は `~/ros2_data/scenario_results/20261125_000544`・`20261125_000544b` と `20261118_222738`)。合格の目安 ([localization_failure_ledger.md](./localization_failure_ledger.md#異常パターンのシナリオの合格の目安)) を外したのは、門のノードの構成だけで、2 種類だった。
  - GNSS が弱る回: AMCL は正しいのに、EKF がずれた GNSS に付いていった。門のノードは GNSS を疑わない。→ 監督の GNSS の判定と分散の対処を残す。
  - 離れた所からの起動: 門のノードの起動の直後の確かめが、低い一致率で GNSS 寄りの誤った姿勢を決め、初期化ノードの正しい姿勢 (全周の照合) を上書きした。→ 未初期化の処理を 1 か所にし、全周の照合を候補にする。
- 門の照合の規則と戻し方の根拠は、記録の再生 ([tools の localization_evaluation.md](../../tools/doc/localization_evaluation.md)) と、シナリオ `gate_jump_driving`・`gate_drift_driving` ([scenarios の README](../../mg_scenario_test/scenarios/README.md))。
