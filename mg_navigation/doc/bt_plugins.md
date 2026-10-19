# BT プラグイン

`bt_navigator` が読み込む BT ノード (`params/nav2_params.yaml` の `plugin_lib_names`)。一覧は [README](../README.md#bt-プラグイン)、使い方と BT 全体の動きは [mg_waypoint_navigation](../../mg_waypoint_navigation/README.md#behavior-tree)。BT の XML を書く人のため、ポートの既定値も載せる (ソースの `providedPorts()` が正)。

## IsPathClear

RPP (Humble 1.1.20 の `RegulatedPurePursuitController`) の衝突判定と同等以上に厳しく判定する。次のどこかで、フットプリントのコストが致死 (`LETHAL_OBSTACLE`) 以上ならブロックとする (Nav2 の `FootprintCollisionChecker` を使う。未知のセルの扱いも RPP と同じ)。

1. 現在の姿勢
2. 最小・最大の先読み距離のキャロットへ向かう動き。向きのずれが `rotate_to_heading_min_angle` を超えればその場の回転の掃引、そうでなければピュアパーシュートの円弧
3. 経路の、ロボットの最近傍点から `max_lookahead_dist + margin` までの範囲

- 設定値は持たない。RPP のパラメータ (`<controller_id>.lookahead_dist` など) を `controller_server` から、フットプリント・`footprint_padding`・`track_unknown_space`・`robot_base_frame` を `local_costmap` から取得する。tick の間隔が一定時間以上空いたら新しいリカバリーとみなし、取得し直す (パラメータの変更は次のリカバリーから効く)。取得できない間は FAILURE (再開しない)
- コストマップは `local_costmap/costmap_raw` を購読する。判定は、新しいコストマップか経路が届いたとき、または一定の周期ごと (姿勢の変化を反映する) に行い、tick では結果を返すだけ
- SUCCESS は、そのリカバリーの間に一度ブロックを観測し、その後、空いた状態が `clear_duration` 以上、かつコストマップ複数枚続いたとき。最初から空いている場合 (スリップなど障害物以外が原因の失敗) は SUCCESS にしない。ブロックはリカバリーに入る直前のコストマップでも記録するが、空いたことはリカバリーが始まってから届いたコストマップでだけ記録する (コストマップのクリア直後の空の地図で誤判定しない)。コストマップが一定時間届かなければ、空いたとはみなさない
- `local_costmap` の `publish_frequency` を下げると、「複数枚・一定時間以上」に時間がかかり、再開が遅れる
- RPP の `use_collision_detection` が false なら、常に FAILURE
- ログ (bt_navigator): リカバリーごとに最初の判定の結果 (`first check of this recovery: blocked=...`。コストマップの古さと経路の終点も出す)、塞がっていることを観測したとき (`path is blocked`)、空いて SUCCESS を返したとき (`path became clear, resuming`) に INFO を出す。経路がない、ロボットの姿勢や経路を変換できない、コストマップが古い、で判定できないときは WARN (間引いて出す)。リカバリーで早く再開しなかった理由は、最初の判定が `blocked=0` (障害物以外が原因と判断) か、判定できなかったかで見分ける
- Nav2 を更新するときは、RPP の判定 (`isCollisionImminent` など) の変更に合わせて見直す

| ポート | 既定値 | 内容 |
| :--- | :--- | :--- |
| `path` | なし | 判定する経路 |
| `margin` | `0.3` | 経路に沿って `max_lookahead_dist` の先まで見る距離 [m] |
| `clear_duration` | `1.0` | 空いた状態が続く必要のある時間 [s] |
| `controller_node` / `controller_id` | `controller_server` / `FollowPath` | RPP のパラメータの取得先 |
| `costmap_node` / `costmap_topic` | `local_costmap/local_costmap` / `local_costmap/costmap_raw` | コストマップのパラメータの取得先と購読するトピック |
| `transform_tolerance` | `0.2` | TF の許容時間 [s] |

## IsPathBlocked

経路の近くが今塞がっているかを判定する。判定は `IsPathClear` と同じ (RPP の衝突判定と同等以上)。パラメータの取得、コストマップの購読、評価は両者で共通のクラス (`PathClearanceMonitor`) が行う。

`mg_navigate_to_pose.xml` では 3 つの使い方をする。

- 走行中の定期的な再計画のゲート (`Inverter` で包む): 塞がっている間は再計画しない
- リカバリーで引き直した経路の確かめ (`Inverter` で包む): 塞がっていなければ走行へ戻る
- リカバリーの後退の条件 (そのまま): 今の経路が塞がっているときだけ後退する

判定の扱い:

- SUCCESS は、塞がっていると確定したときだけ。判定できないとき (パラメータの未取得、コストマップが古い、経路がない、`use_collision_detection` が false) は FAILURE。ゲートでは再計画を止めず、確かめでは走行へ戻して RPP に判定を任せ、後退の条件では後退しない
- リカバリーの段階の中のノードは、その段階で初めて tick される。最初の tick から判定できるよう、パラメータの応答を tick の中で短い上限まで待つ (サービスが見つからないときは待たない)
- `IsPathClear` と違い、tick の間隔でパラメータを取り直さない。パラメータは一度取得し、変更は拾わない
- ログ (bt_navigator): 塞がり始めと解消のときに INFO を出す (`holding periodic replanning` / `resuming periodic replanning`)
- ポートは `IsPathClear` から `clear_duration` を除いたもの (`path`, `margin`, `controller_node`, `controller_id`, `costmap_node`, `costmap_topic`, `transform_tolerance`)

## FindFreeStart

ロボットが、グローバルコストマップ (計画用地図) の障害物にめり込んでいるとき、その領域の外の最も近い位置を `free_start` に出す。
計画用地図にだけある進入禁止の領域に入ると、プランナが「始点が致死」で計画できず、リカバリーをくり返す。`ComputePathToPose` の `start` に `free_start` を渡せば、領域の外から計画でき、走行はローカルコストマップ (センサだけ) が見ながら領域から出る。

- 衝突の判定は RPP と同じ (フットプリントのコストが致死以上。`IsPathClear` と同じ `PathClearanceChecker`)。フットプリントとコストマップは `global_costmap` から取得する (設定値は持たない)
- 探し方: ロボットの姿勢が衝突しているとき、衝突している領域の中だけを通る最短の道のり (ダイクストラ法。8 近傍) で、衝突しない最初の位置を選ぶ。領域の外に出てから進入禁止の線をまたぐことはしない。出口は、`margin` 以内に衝突する位置がない場所にする (出た直後に衝突へ戻らないため)。向きはロボットの今の向きのまま
- SUCCESS は `free_start` を出したときだけ。ロボットがめり込んでいない (探す必要がない)、`max_distance` 以内に出られない、判定できない (フットプリントの未取得、コストマップが未受信・古い、TF が取れない) ときは FAILURE (`free_start` は変えない)
- 自己位置が疑わしいときに領域から出ようとしても、誤った場所へ出るだけなので、`IsLocalizationHealthy` と組み合わせて使う
- ログ (bt_navigator): 最初に見つけたとき INFO (`the robot is inside an obstacle of the global costmap`)、出られないとき WARN

| ポート | 既定値 | 内容 |
| :--- | :--- | :--- |
| `free_start` | なし | (出力) 領域の外の最も近い姿勢 |
| `max_distance` | `3.0` | 出口を探す距離の上限 [m] |
| `margin` | `0.1` | 出口に求める、衝突からの余裕 [m] |
| `controller_node` / `controller_id` | `controller_server` / `FollowPath` | `PathClearanceMonitor` の設定取得先 (RPP の設定は判定に使わない) |
| `costmap_node` / `costmap_topic` | `global_costmap/global_costmap` / `global_costmap/costmap_raw` | コストマップのパラメータの取得先と購読するトピック |
| `transform_tolerance` | `0.2` | TF の許容時間 [s] |

## IsLocalizationHealthy

自己位置の監督 (`localization_supervisor_node`) の状態 `/localization/status` (`localization_supervisor_msgs/LocalizationStatus`) が `NORMAL` なら SUCCESS。
自己位置が疑わしいとき (`SUSPECT`・`ISOLATED`・`RECOVERING`・`DEGRADED`) は、復旧に任せて、走行側の対処 (障害物の領域からの脱出など) を止める。

- 状態を受け取っていない、`max_age` 秒より古い (監督がいない、止まっている) ときは FAILURE。つまり、監視が `none` のときや、状態を配信するノードがないときは SUCCESS にならない
- 状態の変化 (信頼できる ↔ 信頼できない) のときに INFO を出す
- GNSS だけを切り離している間 (`ISOLATED`) も FAILURE になる。将来の拡張: [doc/ideas](../../doc/ideas/localization_pose_trust.md)

| ポート | 既定値 | 内容 |
| :--- | :--- | :--- |
| `topic` | `/localization/status` | 状態のトピック |
| `max_age` | `3.0` | 状態を信頼する古さの上限 [s] |

## CommitPath

| ポート | 既定値 | 内容 |
| :--- | :--- | :--- |
| `candidate` | なし | 計画の結果 (`ComputePathToPose` の出力。失敗すると空になる) |
| `goal` | なし | 今のゴール |
| `path` | なし | 追従する経路 (入出力) |
| `goal_tolerance` | `0.6` | 経路の終点とゴールの距離がこれ以内なら、そのゴールへの経路とみなす [m] |
