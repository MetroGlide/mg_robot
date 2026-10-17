# シナリオ

```
scenarios/
  regression/   動作保証用の回帰テスト (実装を変えたら実行する)
  examples/     書き方の見本 (入門用)
  data/         シナリオが参照する補助データ (waypoint ファイル)
```

`make scenario-test SCENARIO=<名前>` (regression/・examples/ 配下の名前、拡張子なし) で 1 本、
`make scenario-test-all` で回帰テストを一括実行します (詳細は [../README.md](../README.md))。
所要時間はヘッドレス・GPU 構成での目安で、スタックの起動時間を含みます。

## 回帰テスト (regression/)

**層 (tier)**: `smoke` タグ = 変更のたびに実行する短い確認 (`make scenario-test-all TIER=smoke`)。
それ以外 = full 層 (夜間・リリース前。`make scenario-test-all` は smoke も含めて全件を実行)。
`known_issue` タグのシナリオは、既知の問題で失敗するため一括実行から既定で除外される (`KNOWN=1` で含める)。

### smoke 層

| シナリオ | 保証すること |
|---|---|
| `nav_basic_goal` | 障害物なしの単一ゴールに時間内に到達し、停止位置の誤差が 0.5m 以内、速度が 1.2m/s 以下 |
| `nav_unreachable_goal` | 地図の外のゴールへの走行が失敗として扱われる (異常系) |
| `static_avoid_cones` | 通れない隙間 (間隔 0.8m) のコーン列を避けて回り込み、到達する |
| `static_avoid_replan` | 走行中に進路の正面へ壁が出現しても、まず `failure_tolerance` の間待ち、リカバリーの最初の引き直しで迂回して (首振り・後退をせずに) 到達する |
| `dynamic_stop_and_resume` | 目の前に人が飛び出しても、迂回せずに人が立ち去る (3 秒) まで止まって待ち、リカバリー (Spin / BackUp) に入らず再開して到達する (`failure_tolerance` の回帰) |
| `through_point_continuity` | 通過点 3 つを減速・停止せず (停止 3 秒以内) 走り抜け、停止点で止まる (通過点判定・ゴール上書きの回帰) |

### full 層

| シナリオ | 保証すること |
|---|---|
| `waypoint_all_actions` | waypoint の到達時アクション 7 種 (`publish` / `service` / `wait` / `load_map` / `wait_trigger` / `set_navigation_mode` / `amcl_reset`) が実行され、シーケンスが最後まで進む。publish の受信と地図の再配信を確認 |
| `sequencer_pause_resume` | 走行中の一時停止要求で 4 秒以上止まり、解除で再開して最後まで到達する |
| `sequencer_stop_restart` | 走行中に停止して次の index を設定すると、止まったあと (auto_start が) 再開して最後まで到達する |
| `dynamic_persistent_recovery` | 走行中のロボットを壁で囲い込んで居座らせると、待機時間を超えてリカバリーに入り、引き直しが通らないので後退と首振りまで進む。後退をくり返しても囲いの壁に当たらず、除去後に到達する |
| `dynamic_recovery_resume` | 壁で囲い込んでリカバリーに入れ、待機の段階で壁を除くと、`IsPathClear` が待機を中断して 3 秒以内に走り出し、到達する |
| `full_lap_sequencer` | シミュレーション用 waypoint 9 個 (通過点と wait_trigger の停止点) を 1 周する |
| `localization_gnss_denied` | 走行中に GNSS の配信を止めても (ブリッジの `change_publish_state`)、AMCL とホイールオドメトリだけでゴールへ到達し、停止位置の誤差が 0.5m 以内 |
| `narrow_corridor_blocked` | 通れない幅 (0.5m) の狭い回廊へ経路を引かず、壁の外側を迂回して到達する。壁に接触しない (`obstacle_clearance`) |
| `dynamic_avoid` | 横切る歩行者 (0.6m/s) に、ロボット自身の動きで接触せず (`obstacle_clearance`) 到達する |

壁を 1 枚だけ置くと、コストマップにはセンサで見えた範囲しか載らないので、見えている端を回る経路が引けて迂回し、リカバリーに入らないことがある。リカバリーを確かめるシナリオでは、ロボットを囲い込んで計画を必ず失敗させる。

走行中の定期的な再計画を止めるゲート (`mg_navigate_to_pose.xml` の `ReplanPath` の `IsPathBlocked`) が外れたことは、`static_avoid_replan` の停止時間で検出する。`dynamic_stop_and_resume` の人は小さく、ゲートがなくても出ている間に迂回路へ乗らないので、このシナリオでは検出できない (止まって待ち、リカバリーに入らずに再開することの回帰として使う)。

## 見本 (examples/)

| シナリオ | 内容 |
|---|---|
| `example_inline_goals` | インラインで定義した 2 つのゴール。静的障害物の前置きと、走行中の飛び出し |
| `example_waypoints_file` | poses 形式のファイル (`data/warehouse_poses.yaml`) のゴールを順に走行し、waypoint index ごとに障害物を出し入れする |
| `example_waypoints_sequencer` | waypoint_sequencer に走行を委ね、通過を status で判定する |

## 既知の問題

`known_issue` タグ付きのシナリオは、現在ありません。

このほか、シナリオにできなかった (または挙動が不安定で見送った) 項目:

- **コーンの隙間へ経路が引かれ、`static_avoid_cones` が約 3 割で失敗する**: 2026-10-17 の実験走行の調整 (`footprint_padding` を小さくし、前側の幅を狭めた) 以降、
  同じシナリオが成功と失敗に分かれる (9 回中 3 回が失敗)。成功する回は RPP の `detected collision ahead` が 0 回で、失敗する回は同じ警告が連続し、
  リカバリー (spin → wait → backup) を繰り返して時間切れになる。コーンは底面が約 0.5m 角で、間隔 0.8m の隙間は約 0.3m しかなく、ロボットは通れない。
  内接半径が半幅より小さくなったため、プランナが通れない隙間へ経路を引き、RPP が実フットプリントで衝突と判定して進めなくなると考えられる (原因の切り分けは未実施)。
  現地では、狭い通路を通るために `footprint_padding` を小さくしたので、ローカルコストマップ (RPP の衝突の確認) の余白は小さいまま、
  グローバルコストマップ (プランナ) だけ余白を広げる案を検討する。そのときは、アンカー `&footprint_padding` の共有と、
  `test_nav2_params_consistency.py` の共有の検査を分ける。判定は緩めず、タグ `known_issue` も付けない。

- **通れる幅の狭い回廊の通行 (`narrow_corridor_pass`、回帰テストにしていない)**: 幅 1.2m・長さ 4.4m の回廊 (出口以外に抜け道なし) の内側から
  スタートすると、Smac Lattice が `no valid path found` で計画に失敗する (2 回中 2 回)。差動二輪用のプリミティブ (最小旋回半径 0.5m) では、
  幅 1.2m の回廊の中で向きを合わせられないと考えられる。回廊が単に脇にあるだけの配置では、コストの低い迂回が選ばれて回廊を通らない。
  回廊の中の走行が必要な現場では、`global_planner:=navfn` に戻す、回廊を広げる、プリミティブを替える (最小旋回半径の小さいもの) の検討が必要。
  通れない幅の回廊 (`narrow_corridor_blocked`) は、迂回して到達できる。
- **走行の途中・終了後に遠くへ再び respawn**: AMCL の watchdog / GNSS 初期化ノードが `/initialpose` を上書きして位置が狂う。
  原点以外への respawn も、開始時に収束しないことがある (地図の特徴が少ない)。GNSS 関連は範囲外。
- **飛び出した人の前で止まる余裕が小さい (`dynamic_stop_and_resume` がまれに失敗する)**: 人 (`obstacle_clearance` では半径の円) と止まった
  ロボットとの距離は数 cm〜十数 cm しかなく、まれに 0 になって失敗する。シミュレータの真の姿勢で測っても同じなので、自己位置の誤差ではない。
  人を出してから止まるまでに進む距離 (teleport の反映の遅れ、制動距離、scan と制御の遅れ) が巡航速度で決まり、反映の遅れが回ごとにばらつく。
  巡航速度 (`nav2_params.yaml` のアンカー `&v_max`) を下げれば余裕が増える。実機で止まる余裕を確かめてから決める
  ([costmap_and_collision.md](../../mg_navigation/doc/costmap_and_collision.md))。出す距離を延ばすことは、判定を緩めることになるのでしない。

## 解決済みの問題 (参考)

- 横切る歩行者 (`dynamic_avoid`) との接触。`min_scan_range` は、`set_pose` で動く歩行者が停止中のロボットを通り抜けた場合も検出していた。
  `obstacle_clearance` (ロボット自身の動きで近づいた場合だけを判定) に替え、collision_monitor に `PolygonSlowdown`
  (両脇 0.8m・前方 1.4m・後方 0.3m の広い範囲で減速) を追加し、`PolygonApproach` をフットプリント基準にした。
  歩行者は、進路の手前で減速したロボットの側面・後部へ入ってくるため、前方の箱だけでは間に合わなかった。
  `PolygonStop` を前方 1.0m・左右 0.5m に広げると接触は減るが、壁際で止まり続けて `static_avoid_replan` が失敗した。

- 一時的な障害物で、出現から 1 秒足らずで FollowPath が中断されリカバリーに入っていた。
  原因は `controller_server.failure_tolerance: 0.5`。数秒待てる長さにした。
  **`-1` (無期限) は不可**: progress_checker の失敗まで無視され、詰まると永久に停止する。
- 人が数秒前に立って去っても、再開して進みきる前に progress_checker が失敗し、Spin に入っていた (`dynamic_stop_and_resume`)。
  `movement_time_allowance` が `failure_tolerance` と同じで、どちらも止まった時点から数えるため。
  `movement_time_allowance` を、再開して `required_movement_radius` 進む時間の分だけ長くした。
- コーン列 (間隔 0.8m) の隙間を、プランナが通れると誤認して詰まっていた。膨張レイヤーの内接半径がフットプリントの後端 (0.2m) で
  決まり半幅 (0.3m) より小さかったため。`nav2_params.yaml` の両コストマップに `footprint_padding: 0.1` を追加した。
- world に常設されていたコーン (`warehouse.sdf`) が、他のシナリオの走行を妨げていた。`static_avoid_cones` へ移した。
- waypoint の publish アクションが初回に取りこぼされていた (パブリッシャ作成直後の 1 回送信で購読側の発見が間に合わない)。
  パブリッシャを新規に作ったとき、購読側が見つかるまで最大 1 秒待つようにした。
- launch の終了後もシミュレータ (`ign gazebo -s`) が残り、スイートで実行を重ねるたびに干渉していた。
- 急に出した障害物が、ロボットに重なるか後ろに出て、RPP の前で待つ動きを確かめられていなかった (`dynamic_stop_and_resume`)。
  Fuel モデルの `spawn` は完了まで約 1 秒かかり、その間にロボットが進むため。障害物は待避位置に置いておき、`teleport` で出す。
  飛び出す距離は、巡航速度で RPP が止まりきれる距離にしている (近すぎると RPP の前に `collision_monitor` の範囲に入り、
  RPP の待ちを確かめられない)。速度を変えたら、出す距離を見直す。
- 速度を上げたあと、障害物に出会わないまま PASS していた (`dynamic_avoid`。歩行者が来る前に交差点を通り過ぎた)。
  障害物のシナリオには `obstacle_clearance` の `require_within` で「出会ったこと」を判定に入れた。
- attach で続けて実行すると、1 つおきに respawn の収束で失敗していた。respawn が AMCL だけに初期位置を送り、EKF が前の
  位置に残っていたため。プロファイルの `initialpose_topics` で EKF (`/set_pose`) にも送る。
