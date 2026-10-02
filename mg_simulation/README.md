# mg_simulation

Gazebo Fortress (Ignition) のシミュレータ。ワールド、ロボットの生成、ROS との橋渡し (ブリッジ) の設定、シミュレータ用の地図とウェイポイントを持つ。
実機のドライバの代わりに、LiDAR・GNSS・深度カメラ・車体の動きを、Gazebo が ROS のトピックとして配信する。

## 使い方

```bash
make build-sim                  # simulation イメージをビルド
make gazebo-simulation          # シミュレータを起動 (ヘッドレスにするには OPTS="headless:=true")
make navigation                 # (別端末) ナビゲーションスタック。.env の SIMULATION=true
```

GPU の指定は `.env` の `USE_GPU`、または `make gazebo-simulation USE_GPU=nvidia` ([doc/docker.md](../doc/docker.md))。
自動テストは [mg_scenario_test](../mg_scenario_test/README.md)。シミュレータとナビゲーションを一緒に起動して、シナリオを実行する。

## launch

`launch/bringup.launch.py` (compose サービス `gazebo-simulation`)。

| 引数 | 既定値 | 内容 |
| :--- | :--- | :--- |
| `world` | `worlds/warehouse.sdf` | ワールドの SDF (絶対パス) |
| `robot_name` | `mg` | Gazebo 上のロボット名 |
| `spawn_x` / `spawn_y` / `spawn_z` / `spawn_yaw` | `0.0` / `0.0` / `0.05` / `0.0` | ロボットを置く位置 |
| `headless` | `false` | `true` で GUI なし (`gz sim -s`) |
| `publish_gazebo_tf` | `$PUBLISH_GAZEBO_TF` (未設定なら `false`) | `true` で `robot_state_publisher` を起動して、Gazebo 側の TF を出す |

起動するもの: Gazebo (`ros_gz_sim` の `gz_sim.launch.py`)、ロボットの生成 (`ros_gz_sim create`)、`ros_gz_bridge` の `parameter_bridge`。
ロボットの URDF は、`mg_description` の xacro を `simulation:=true` で展開して作る ([mg_description](../mg_description/README.md))。

## ワールド

`worlds/` に 6 つある。

| ワールド | ファイル |
| :--- | :--- |
| warehouse (既定) | `warehouse.sdf` |
| construction | `construction.sdf` |
| office | `office.sdf` |
| orchard | `orchard.sdf` |
| pipeline | `pipeline.sdf` |
| solar_farm | `solar_farm.sdf` |

地図とウェイポイントがあるのは、warehouse だけ ([下記](#地図とウェイポイント))。

### モデル

`models/` (ワールドが使う 3D モデル。`IGN_GAZEBO_RESOURCE_PATH` に設定される) は、Git に含まれない (`.gitignore` の対象)。`.gitkeep` だけがある。
`scripts/download_models.sh` は、clearpath_simulator のモデル (約 280 MB) を取得するスクリプト。ただし、`models/` が `.gitkeep` のために常に存在するので、「ディレクトリが無ければ取得する」判定が成立せず、何もしない。モデルは各自の環境で `models/` に置く。

## ブリッジ (`config/bridge.yaml`)

Gazebo のトピックと ROS のトピックの対応。

| ROS のトピック | Gazebo のトピック | 向き |
| :--- | :--- | :--- |
| `/clock` | `/clock` | Gazebo → ROS |
| `/odom` | `/odom` | Gazebo → ROS |
| `/tf` | `/tf` | Gazebo → ROS |
| `/cmd_vel` | `/cmd_vel` | ROS → Gazebo |
| `/scan_front_lidar_origin` | `/front_lidar_scan` | Gazebo → ROS |
| `/scan_top_lidar` | `/top_lidar_scan` | Gazebo → ROS |
| `/gps/fix` | `/navsat` | Gazebo → ROS |
| `/rs_d435i/depth/image_raw`、`/rs_d435i/color/image_raw` | `/rgbd_camera/depth_image`、`/rgbd_camera/image` | Gazebo → ROS |
| `/rs_d435i/depth/camera_info`、`/rs_d435i/color/camera_info` | `/rgbd_camera/camera_info` | Gazebo → ROS |
| `/rs_d435i/depth/color/points` | `/rgbd_camera/points` | Gazebo → ROS |

- 前方 LiDAR は `/scan_front_lidar_origin` に出る。実機と同じく、`mg_drivers` の公開制御ノードが `/scan_front_lidar` に中継する。
- 車体の動き (差動駆動と車輪のスリップ) は、URDF の Gazebo プラグインで定義している。

## シミュレータ用のナビゲーション設定 (`config/nav_bridge_sim.yaml`)

GNSS ブリッジ (`slam_gnss_nav_bridge`) のシミュレータ用パラメータ。Gazebo の GNSS は `NavSatFix` (`/gps/fix`) だけを出し、搬送波位相の解の種類がないので、単独測位として扱う。
`hAcc` の下限 (`single_floor_m: 0.05`) と、アンテナ位置の補正 (レバーアーム) を決める。

## 地図とウェイポイント

`maps/warehouse/` に、warehouse ワールド用のデータがある。

| ファイル | 内容 |
| :--- | :--- |
| `localization.yaml`、`map.pgm` | 測位用の地図 |
| `planning.yaml` | 計画用の地図 |
| `waypoints.yaml` | ウェイポイント (9 点) |
| `gnss_transform.yaml` | GNSS を地図の座標に変換する (`tools/scripts/make_sim_gnss_transform.py` で生成) |

これらは、シナリオテストのプロファイル (`mg_scenario_test/profiles/mg01.yaml` の `warehouse`) が `pkg://mg_simulation/maps/warehouse/...` として参照する。

## 依存

`mg_description`、`robot_state_publisher`、`xacro`、`ros_gz_sim`、`ros_gz_bridge`、`ros_gz_image`。simulation イメージで、他のシミュレータ系パッケージと一緒にビルドされる ([doc/docker.md](../doc/docker.md))。テストはない。
