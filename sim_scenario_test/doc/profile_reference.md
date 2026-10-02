# ロボットプロファイル リファレンス

プロファイルは、ロボット・シミュレーション構成に固有の設定をシナリオから分離する YAML です。
シナリオの `profile:` (または CLI の `--profile`) に **名前** または **ファイルパス** を指定します。
名前で参照するには、プロファイルを置くパッケージが CMake で
`ament_index_register_resource("sim_scenario_test.profiles")` を呼び、`share/<pkg>/profiles/<名前>.yaml` に
インストールしている必要があります。

## 全項目

```yaml
name: mg01                        # 必須
plugins:                          # 起動時に import するモジュール (型の登録が目的)
  - mg_scenario_test.plugins

sim:
  backend: gazebo_fortress        # 登録済みのバックエンド名
  robot_entity: mg                # シミュレータ上のロボットのモデル名
  robot_spawn_z: 0.05             # respawn / teleport 時にロボットへ加える高さ [m] (地面へのめり込み防止)
  clock_stall_sec: 30.0           # sim 時計が壁時計でこの秒数進まなければ ERROR
  launch:                         # 起動するシミュレータの launch (run / run-all のみ)
    package: mg_simulation
    file: launch/bringup.launch.py    # パッケージの share ディレクトリからの相対パス
    args:                             # launch 引数。値は変数展開される
      world: "{world.sdf}"
      headless: "{headless}"

stack:                            # 起動するナビゲーションスタックの launch (同じ形式)
  package: mg_bringup
  file: launch/navigation/bringup_navigation.launch.py
  args:
    map_path: "{world.localization_map}"

frames:
  map: map                        # 地図フレーム
  base: base_footprint            # ロボットのベースフレーム
  map_in_world: { x: 0.0, y: 0.0, yaw: 0.0 }   # ワールド座標系での地図原点の姿勢

robot:
  # base フレームでのロボットの外形 (多角形の頂点 [x, y])。省略可。
  # obstacle_clearance monitor が、フットプリントと障害物の距離を測るのに使う
  footprint: [[0.4, 0.3], [0.4, -0.3], [-0.2, -0.3], [-0.2, 0.3]]

nav2:
  navigate_to_pose_action: navigate_to_pose
  initialpose_topic: /initialpose
  initialpose_cov_xy: 0.25
  initialpose_cov_yaw: 0.0685
  clear_costmap_services:         # コストマップのクリアに呼ぶサービス
    - /global_costmap/clear_entirely_global_costmap
    - /local_costmap/clear_entirely_local_costmap

readiness:                        # 走行開始前に待つもの
  timeout_sec: 120.0
  lifecycle_managers: [lifecycle_manager_localization, lifecycle_manager_navigation]
  services: []                    # 利用可能になるまで待つサービス名

worlds:                           # ワールドごとの変数 (シナリオの world: で選ぶ)
  warehouse:
    sim_world: warehouse          # Gazebo のワールド名 (省略時はキー名)
    sdf: pkg://mg_simulation/worlds/warehouse.sdf
    localization_map: pkg://mg_simulation/maps/warehouse/localization.yaml
    waypoints: pkg://mg_simulation/maps/warehouse/waypoints.yaml   # 任意のキーを追加できる
```

## 変数展開

`sim.launch.args` / `stack.args` の値と、シナリオ中のパス (`waypoints_file` など) で使えます。

| 記法 | 展開先 |
|---|---|
| `{world.<キー>}` | `worlds.<選択したワールド>.<キー>` (`name` と `sim_world` は自動で使える) |
| `{headless}` | `run` の GUI 指定から決まる `true` / `false` (launch 引数の展開時のみ) |
| `{profile.name}` | プロファイル名 |
| `pkg://<package>/<path>` | パッケージの share ディレクトリ配下の絶対パス |

未定義の変数は読み込み時にエラーになります。

## 補足

- `attach` モード (`--attach`) では `sim.launch` / `stack` は使われません。起動済みのものに接続します。
- 他のロボットに使う場合、標準の Nav2 のみなら `plugins` は空でよく、`stack` にそのロボットの
  ナビゲーション起動 launch を指定するだけで動きます。
