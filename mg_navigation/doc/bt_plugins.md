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

走行中に、経路の近くが今塞がっているかを判定する。判定は `IsPathClear` と同じ (RPP の衝突判定と同等以上)。パラメータの取得、コストマップの購読、評価は両者で共通のクラス (`PathClearanceMonitor`) が行う。

- SUCCESS は、塞がっていると確定したときだけ。判定できないとき (パラメータの未取得、コストマップが古い、経路がない、`use_collision_detection` が false) は FAILURE。XML では `Inverter` で包んで再計画のゲートにするので、判定できないときは再計画を止めない (安全側)
- `IsPathClear` と違い、定期的に tick されるので、tick の間隔でパラメータを取り直さない。パラメータは一度取得し、変更は拾わない
- ログ (bt_navigator): 塞がり始めと解消のときに INFO を出す (`holding periodic replanning` / `resuming periodic replanning`)
- ポートは `IsPathClear` から `clear_duration` を除いたもの (`path`, `margin`, `controller_node`, `controller_id`, `costmap_node`, `costmap_topic`, `transform_tolerance`)

## CommitPath

| ポート | 既定値 | 内容 |
| :--- | :--- | :--- |
| `candidate` | なし | 計画の結果 (`ComputePathToPose` の出力。失敗すると空になる) |
| `goal` | なし | 今のゴール |
| `path` | なし | 追従する経路 (入出力) |
| `goal_tolerance` | `0.6` | 経路の終点とゴールの距離がこれ以内なら、そのゴールへの経路とみなす [m] |
