# RPP (Regulated Pure Pursuit) の調整

`controller_server` の `FollowPath` (`nav2_regulated_pure_pursuit_controller`) のパラメータの意味と、調整の考え方。Nav2 全体の構成は [architecture.md](./architecture.md)。

- **実際の値は [nav2_params.yaml](../params/nav2_params.yaml) の `FollowPath` が正**。この文書には現在値を書かない。文中の数値は、考え方を説明するための**例**で、実際の設定とは限らない。
- 値同士の関係 (守る規則) は [3. 値の関係](#3-値の関係) にまとめ、`test/test_nav2_params_consistency.py` が検査する。

## 1. 動作の流れ

1. グローバル経路のうち、ロボットに最も近い点を探す (`max_robot_pose_search_dist` の範囲内)。
2. そこから先読み距離 L だけ進んだ点 (carrot) を決める。
3. carrot が車体座標で横にずれている量を y とすると、曲率は κ = 2y / L²、角速度は ω = v·κ になる。
4. 目標速度 `desired_linear_vel` に、曲率・コスト・ゴール接近による減速を掛ける。
5. 現在の指令速度で carrot まで前進したとき、フットプリントが致死コストに重なるかを確認する。重なれば例外を出す。
6. 出力は `cmd_vel_nav`。後段の `collision_monitor` と `velocity_smoother` を通って、`cmd_vel` になる。

## 2. パラメータ

### 先読み (軌跡の質を決める最重要グループ)

| パラメータ | 内容 |
| :--- | :--- |
| `use_velocity_scaled_lookahead_dist` | true なら L = 現在速度 × `lookahead_time` を、`min_lookahead_dist` 〜 `max_lookahead_dist` に収める。false なら `lookahead_dist` で固定 |
| `lookahead_time` | 速度から L を作るときの係数 [s]。「何秒先を見るか」 |
| `min_lookahead_dist` / `max_lookahead_dist` | L の下限と上限 [m] |
| `lookahead_dist` | 速度連動を使わないときの固定値。**速度連動を使う間は参照されないので、設定に置かない** |
| `use_interpolation` | 経路の点の間を補間して carrot を決める |
| `max_robot_pose_search_dist` | 最近点を探す範囲。経路が自分に近づく形 (ループ、折り返し) で、最近点が離れた区間へ飛ぶのを防ぐ |

**考え方**

- L が長いと、蛇行しにくく、位置の飛びやオドメトリの誤差にも強い。その代わりカーブで内側を切り、経路への収束が遅い。カーブでの切り込み量の目安は L² / (2R) (R は経路の曲率半径)。例: L = 1.3 m、R = 2 m なら約 0.4 m。
- L が短いと、経路に忠実になる。その代わり、制御の遅れ・ヨーのノイズ・低い制御周波数で蛇行する。
- ヨーを直す動きの速さは v / L に比例する。**速度が変わっても L / v (= 先読みの時間) を保てば、動きの性質が保たれる**。速度連動はそのための仕組みで、`lookahead_time` が「先読みの時間」に当たる。
- `min_lookahead_dist` は、低速や停止からの発進で L が短くなりすぎるのを防ぐ。`max_lookahead_dist` は、速度を上げたときに L が際限なく伸びるのを防ぐ。
- **巡航速度で `min` や `max` に張り付くと、速度連動が効かない。** 巡航速度 × `lookahead_time` が、`min` と `max` の間に入るようにする。例: 巡航 0.8 m/s、`lookahead_time` 1.6 秒なら L は 1.28 m で、`min` を 0.8 m、`max` を 1.8 m にすれば、巡航はどちらにも当たらない。
- 速度に使うのは、指令ではなく**現在の速度** (オドメトリ)。停止からの発進直後は `min` になる。
- 経路が滑らかでないと、先読みを調整しても蛇行が残る。経路の質は、プランナとコストマップの膨張が決める。

### 速度と減速

| パラメータ | 内容 |
| :--- | :--- |
| `desired_linear_vel` | 巡航速度 [m/s]。`velocity_smoother` の `max_velocity` (x) と同じ値にする |
| `use_regulated_linear_velocity_scaling` | 曲率が大きいときに減速する |
| `regulated_linear_scaling_min_radius` | 旋回半径がこれ未満のカーブで、速度を v × (R / この値) にする [m] |
| `regulated_linear_scaling_min_speed` | 上の減速と、コストによる減速の下限 [m/s]。目標速度がこれ未満でも、この値になる |
| `use_cost_regulated_linear_velocity_scaling` | 障害物の近く (コストが高い場所) で減速する。詳細は [コストによる減速](#コストによる減速) |
| `cost_scaling_dist` | 障害物までの距離がこれ未満で減速を始める [m] |
| `cost_scaling_gain` | 1 以下。小さいほど強く減速する |
| `inflation_cost_scaling_factor` | コストから距離を逆算するための値。コストマップの膨張と同じ値にする |
| `approach_velocity_scaling_dist` | ゴールまでの残りがこれ未満なら、残り距離に比例して減速する [m] |
| `min_approach_linear_velocity` | ゴール接近の減速の下限 [m/s] |

**考え方**

- 曲率による減速は、半径が `regulated_linear_scaling_min_radius` 未満のカーブでしか効かない。巡航が遅く、経路のカーブが緩やかなら、ほとんど発動しない。カーブの手前で十分に落としたいときは、この値を大きくする。
- ゴール接近の下限は、モータが起動できる最小速度より高くする。低すぎると、ゴール手前で止まりきって進めず、`progress_checker` の「進捗なし」に引っかかる。
- `approach_velocity_scaling_dist` は、ゴールで滑らかに止まるための距離。必要な制動距離は v² / (2 × 減速度)。例: 0.8 m/s、減速度 2.0 m/s² なら 0.16 m。これに余裕を足す。短くするとゴール付近の所要時間が縮み、長くすると止まり方が穏やかになる。
- 実際の上限と加減速は `velocity_smoother` が最後に決める ([architecture.md](./architecture.md#速度指令の経路))。RPP の出力は、そこで絞られる。

#### コストによる減速

ローカルコストマップの膨張コストから、ロボットの**原点**が障害物からどれだけ離れているかを逆算し、近いときに減速する。

```
距離 d = 内接半径 + ln(252 / コスト) / inflation_cost_scaling_factor
d < cost_scaling_dist のとき、速度 ×= cost_scaling_gain × d / cost_scaling_dist
```

- **`inflation_cost_scaling_factor` は、ローカルコストマップの `inflation_layer.cost_scaling_factor` と同じ値にする。** コストは膨張レイヤーがこの値で作るので、違う値で逆算すると距離がずれる。例: コストマップが 3.0 で RPP が 1.0 だと、内接半径を超える分の距離を実際の約 3 倍と見積もり、減速がほとんど発動しない。
- **`cost_scaling_dist` は、内接半径より大きくする。** 上の式の d は、膨張のコストが付く範囲では内接半径以上になる。`cost_scaling_dist` が内接半径以下だと、`d < cost_scaling_dist` が成り立たず、減速が一切効かない。内接半径は、原点からフットプリントの最も近い辺までの距離 (`footprint_padding` を含む)。
- `cost_scaling_dist` は、膨張半径 (`inflation_radius`) 以下にする。膨張の外ではコストが 0 で、距離を求められない。
- 判定に使うのは原点のセルのコスト。原点がフットプリントの後端寄りだと、前方の障害物には反応が遅れる。
- 狭い通路でも減速する。通路の幅が狭いほど原点が壁に近く、`cost_scaling_gain × d / cost_scaling_dist` が小さくなる。通路で遅すぎるときは、`cost_scaling_dist` を小さくする (内接半径より大きい範囲で)、または `cost_scaling_gain` を 1 に近づける。

### 旋回

| パラメータ | 内容 |
| :--- | :--- |
| `use_rotate_to_heading` | 経路とのヨー差が大きいとき、または最終ゴールで向きを合わせるとき、その場で旋回する |
| `rotate_to_heading_min_angle` | その場旋回を始めるヨー差 [rad] |
| `rotate_to_heading_angular_vel` | その場旋回の角速度 [rad/s] |
| `max_angular_accel` | その場旋回の角加速度 [rad/s²]。`velocity_smoother` と `behavior_server` の旋回の加速度と同じ値にする |
| `allow_reversing` | 経路が後退を含むときに後退する |

**考え方**

- ゴール付近で何度も向きを合わせ直すときは、`yaw_goal_tolerance` と、その場旋回の角速度・角加速度を見る。
- その場旋回の角速度は、`velocity_smoother` の旋回の `max_velocity` 以下にする。

### 衝突確認と TF

| パラメータ | 内容 |
| :--- | :--- |
| `use_collision_detection` | 現在の指令速度で carrot まで進んだときの、フットプリントの致死コストへの重なりを確認する |
| `max_allowed_time_to_collision_up_to_carrot` | 確認する時間の上限 [s]。carrot までの距離が短ければ、そこまで |
| `transform_tolerance` | TF の許容する遅れ [s] |

**考え方**

- フットプリントは `footprint_padding` を含む。誤検出で止まるときは、コストマップのノイズ、padding、この時間のどれかを疑う。
- **この時間は、`collision_monitor` の `PolygonApproach` の `time_before_collision` より長くする。** RPP が先に減速・停止し、`collision_monitor` は通常の走行では発動しない後ろ盾になる ([costmap_and_collision.md](./costmap_and_collision.md#衝突の防止-collision_monitor))。
- `transform_tolerance` が小さすぎると、PC の負荷や EKF の遅れで、TF の外挿のエラーが出る。

## 3. 値の関係

守る関係。YAML で表せるもの (等しい値) は、スカラーのアンカーで共有している。アンカーで表せないものは `test/test_nav2_params_consistency.py` が検査する。

| 関係 | 理由 |
| :--- | :--- |
| `desired_linear_vel` = `velocity_smoother` の `max_velocity` (x) | RPP の目標が smoother で頭打ちにならないように |
| `max_angular_accel` = `behavior_server` の `rotational_acc_lim` = `velocity_smoother` の旋回の加速度 | 旋回の動きを、どの経路でも揃える |
| `min_lookahead_dist` < `desired_linear_vel` × `lookahead_time` < `max_lookahead_dist` | 巡航で先読みが速度連動すること |
| `lookahead_dist` を置かない | 速度連動を使う間は参照されず、実態と食い違う値が残るため |
| `inflation_cost_scaling_factor` = ローカルコストマップの `inflation_layer.cost_scaling_factor` | コストから距離を正しく逆算するため。アンカーで共有 |
| 内接半径 < `cost_scaling_dist` ≤ `inflation_radius` | コストによる減速が効く範囲 |
| `max_allowed_time_to_collision_up_to_carrot` > `collision_monitor` の `PolygonApproach` の `time_before_collision` | RPP を先に効かせる |

## 4. 調整の進め方

1. **観察を用意する**: RViz で `/received_global_plan`、`/lookahead_point`、`/lookahead_arc`、実際の軌跡を重ねる。`cmd_vel_nav` の角速度を rqt_plot などで見ると、蛇行が分かる。
2. **1 つずつ変える**: 先読み → 減速 → 衝突確認の順に。同時に複数を変えると、原因が分からない。
3. **実行中に試す**: 主要なパラメータは `ros2 param set /controller_server FollowPath.<名前> <値>` で変えられる。反映されないものは、再起動して確認する。
4. **確定したら**: YAML に反映し、`make test pkg=mg_navigation` (値の関係の検査) と、`make scenario-test-all` (回帰) を実行する。実機では、シミュレータより遅れとノイズが大きいので、先読みは短くしすぎない。

## 5. 症状と対処

| 症状 | まず見るもの |
| :--- | :--- |
| 左右に蛇行する | `min_lookahead_dist` と `lookahead_time` を大きく。次に速度を下げる。自己位置の飛びとオドメトリの遅れを確認 |
| カーブで内側を切り、コーンや壁に近づく | `lookahead_time` と `min_lookahead_dist` を小さく。`regulated_linear_scaling_min_radius` を大きくして、カーブの手前で減速させる |
| 低速や発進直後に蛇行する | `min_lookahead_dist` を大きく |
| ゴールを行き過ぎる、止まれない | `approach_velocity_scaling_dist` を長く。`velocity_smoother` の減速度も確認 |
| ゴール手前で止まって進まない | `min_approach_linear_velocity` を、モータの起動速度以上に |
| 障害物の前で急に止まる | `max_allowed_time_to_collision_up_to_carrot` を短く。コストマップのノイズと `footprint_padding` を確認 |
| 障害物の近くでも減速しない | [コストによる減速](#コストによる減速) の 2 つの規則 (`inflation_cost_scaling_factor`、`cost_scaling_dist`) を確認 |
| 狭い通路で遅すぎる | `cost_scaling_dist` を小さく (内接半径より大きい範囲で)、`cost_scaling_gain` を 1 に近づける |
| ゴールの向き合わせで揺れる | `rotate_to_heading_angular_vel` を下げる。`yaw_goal_tolerance` を確認 |
| 発進が唐突、または遅い | RPP ではなく `velocity_smoother` の加速度 |

## 6. 他との関係

- **`velocity_smoother`**: RPP の指令は、smoother の上限と加減速で絞られる。RPP が先読みに使う現在速度は、smoother の加速に追従するので、発進直後は短い先読みになる。
- **コストマップとプランナ**: 経路の質と、膨張のコストの付き方が、蛇行・コストによる減速に効く。
- **`collision_monitor`**: RPP が主で、`collision_monitor` は後ろ盾。時間の大小関係は [3. 値の関係](#3-値の関係) のとおり。
