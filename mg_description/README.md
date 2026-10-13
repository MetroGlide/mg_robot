# mg_description

MG-01 の URDF (xacro) と、RViz2 の表示設定。`robot_state_publisher` を起動して、車体とセンサの TF を配信する。

## 構成

| パス | 内容 |
| :--- | :--- |
| `urdf/mg_description.urdf.xacro` | ロボット全体。車体とセンサを組み合わせる |
| `urdf/robot_base.urdf.xacro` | 車体 (`base_footprint`、`base_link`、駆動輪 2 つ、キャスター)。シミュレーション用の差動駆動・車輪スリップのプラグインを含む |
| `urdf/sensors.xacro` | センサのマクロ (`lidar_2d`、`lidar_3d`、`gps`、`realsense_d435`、`realsense_d435i`) |
| `launch/bringup.launch.py` | `robot_state_publisher` と `joint_state_publisher` |
| `rviz/display.rviz` | URDF の確認用の RViz2 設定 |

## launch

`launch/bringup.launch.py` は、`mg_bringup` の `bringup_common` から起動される ([mg_bringup](../mg_bringup/README.md))。

| 引数 | 内容 |
| :--- | :--- |
| `simulation` | `use_sim_time` に渡す |
| `xacro_file_name` | `urdf/` の xacro ファイル名 |

- この launch は、xacro に `simulation:=` を渡さない。常に実機用の URDF (`simulation=false`) になる。
- シミュレーション用の URDF (`simulation:=true`。センサの Gazebo プラグインを含む) は、`mg_simulation` の launch が xacro を直接展開して作る ([mg_simulation](../mg_simulation/README.md))。

## 座標系 (TF)

```
base_footprint
└─ base_link
   ├─ left_wheel_link
   ├─ right_wheel_link
   ├─ caster_link
   ├─ gps_link
   └─ top_frame_link
      ├─ top_lrf_link       上 LiDAR (RPLiDAR S2)
      ├─ livox_lidar_link   Livox
      └─ camera_link_base   RealSense D435i
```

各リンクの位置・向き・車輪の寸法 (`wheel_separation` など) は、`urdf/` の xacro が正。

- `map→odom` は EKF、`odom→base_footprint` は `mg_drivers` の `odometry_tf_broadcaster_node` が配信する ([doc/system_architecture.md](../doc/system_architecture.md#座標系-tf))。
- Livox はリンクだけで、ドライバは起動していない。

## センサのシミュレーション

`sensors.xacro` のマクロは、`simulation:=true` のとき Gazebo のセンサを定義する。

- 上 LiDAR (`lidar_2d`): 周期、サンプル数、測距範囲、ノイズは `sensors.xacro` が正
- GPS (`gps`)
- RealSense D435i (`realsense_d435i`)

シミュレーション用のトピックの対応は、`mg_simulation/config/bridge.yaml`。

## 依存

`package.xml` の `exec_depend` に、`realsense2_description` と `joint_state_publisher` が無い。`joint_state_publisher_gui` が書かれているが、launch は使っていない。
