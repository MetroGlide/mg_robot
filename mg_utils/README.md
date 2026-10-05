# mg_utils

他のパッケージから使う共通の補助。launch 引数の定義、深度画像からの点群の生成、rosbag の収録 launch。

## launch 引数の定義 (`mg_utils.launch_argument`)

launch の引数は、`LaunchArgumentCreator` で定義する。宣言 (`DeclareLaunchArgument`) と、値の参照 (`LaunchConfiguration`) を 1 か所で作る。

```python
from mg_utils.launch_argument import LaunchArgumentCreator

def generate_launch_description():
    arg = LaunchArgumentCreator()
    simulation = arg.create("simulation", default=EnvironmentVariable("SIMULATION"))
    return LaunchDescription([
        *arg.get_created_declare_launch_args(),   # 宣言をまとめて渡す
        Node(..., parameters=[{"use_sim_time": simulation.launch_config}]),
    ])
```

| API | 内容 |
| :--- | :--- |
| `LaunchArgumentCreator.create(name, default=None, description=None)` | 引数を作り、`LaunchArgument` を返す |
| `LaunchArgumentCreator.get_created_declare_launch_args()` | これまでに作った `DeclareLaunchArgument` の一覧 |
| `LaunchArgument.launch_config` | `LaunchConfiguration` |
| `LaunchArgument.declare_launch_arg` | `DeclareLaunchArgument` |

`SIMULATION` の環境変数で実機とシミュレーションを切り替えるときは、`default=EnvironmentVariable("SIMULATION")` を使う。
`sim_scenario_test` は、ロボットに依存しない基盤なので、これを使わない。

## 点群の生成 (`mg_utils.point_cloud`)

ROS に依存しない numpy の関数。

| API | 内容 |
| :--- | :--- |
| `build_point_array(depth_m, u_norm, v_norm, min_depth, max_depth, bgr=None)` | 深度画像から、有効な画素の点群を `float32` の `(N, 3)` (`bgr` を渡すと `(N, 4)`。x, y, z, rgb) で返す。有効な画素がなければ `None` |

`mg_drivers` の `depth_to_pointcloud_node` が使う。

## rosbag の収録 (`launch/record_bag.launch.py`)

`ros2 bag record` を、mcap 形式で起動する。`mg_navigation/bringup.launch.py` と `mg_slam/bringup_slam_toolbox.launch.py` が、`record_bag:=true` のときに include する。`mg_slam` には、独自の `record_bag.launch.py` もある。

| 引数 | 既定値 | 内容 |
| :--- | :--- | :--- |
| `caller_pkg_name` | `mg_navigation` | 収録するトピックの一覧 `params/record_topic_list.txt` を持つパッケージ |
| `record_bag_base_name` | `navigation_` | bag のディレクトリ名の接頭辞 |
| `simulation` | `$SIMULATION` | `true` なら `--use-sim-time` を付ける |

- 環境変数 `ROSBAG_PATH` が必須 (未設定ならエラーになる)。
- 保存先は `$ROSBAG_PATH/<YYYYMMDD>_bag/<接頭辞><番号>`。番号は、同じ日のディレクトリの最大値に 1 を足す。
- 収録するトピックは、`caller_pkg_name` のパッケージの `params/record_topic_list.txt` (1 行 1 トピック)。ファイルが無いときは、ログに出して、トピックの一覧を空にして起動する。

走行ログを手動で収録するスクリプトは、`tools/scripts/record.sh` ([tools の README](../tools/README.md))。

## テスト

```bash
make test pkg=mg_utils
```

`test/test_point_cloud.py` が、`build_point_array` を調べる。

## 依存

`package.xml` に、`ament_cmake` と `ament_cmake_python` しか書かれていない。`launch`、`launch_ros`、`ament_index_python`、`numpy` は、書かれていない。
