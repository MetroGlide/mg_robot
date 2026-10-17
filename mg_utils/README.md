# mg_utils

他のパッケージから使う共通の補助。launch 引数の定義、深度画像からの点群の生成、状態監視の共通部品（トピックの途絶・受信頻度の判定、イベントログ）、rosbag の収録 launch。

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

## 状態監視の共通部品

規約は [doc/monitoring.md](../doc/monitoring.md)。

### トピックの途絶・受信頻度の判定 (`mg_utils.monitoring`)

`driver_watchdog_node` (反応つきの監視) と `topic_monitor_node` (表示だけの監視) が、同じ判定と同じ設定の書き方を使う。

| モジュール | ROS | 内容 |
| :--- | :--- | :--- |
| `activity` | 不要 | `ActivityMonitor` (最後の受信からの途絶、起動直後の猶予、初回のメッセージまでの待機、復帰の保持、`ok=False` の受信を異常にする、受信頻度)、`RateEstimator`、`TickGuard` (監視ノード自身の周期の遅れ)。時刻は呼び出し側が渡す |
| `health` | 不要 | `evaluate_topic_health(monitor, now, expected_hz, hz_warn_ratio, fault_level)`: 診断に出すレベル (`ok` / `warn` / `error`)・メッセージ・values (`hz`、`expected_hz`、`stale_sec`) を作る |
| `thresholds` | 不要 | `level_for_high(value, warn, error)` (値が大きいほど悪いものの判定)、`worst(*levels)`、`MovingAverage(window)` (瞬間的な値で診断のレベルが揺れないようにする) |
| `level_tracker` | 不要 | `LevelChangeTracker`: 診断のレベルの変化を検出する (`event_relay_node` が使う) |
| `topic_source_config` | 不要 | 監視するトピックの YAML (`<id>.topic` / `type` / `expected_hz` / `stale_timeout_sec` / `armed_on_first_message` / `qos` / `ok_field` / `fault_level` / `enabled`) の読み取りと検証 |
| `topic_source` | 必要 (rclpy) | `declare_source_config(node, id, default_stale_timeout_sec)` と `TopicSource` (設定に従う購読と `ActivityMonitor` の組み立て)。`ok_field` を指定しないソースは、デシリアライズせず (`raw`) に受け取る |
| `diagnostic_task` | 必要 (`diagnostic_updater`) | `TopicHealthTask`: `evaluate_topic_health` の結果を `diagnostic_updater` のタスクとして出す |

- `fault_level: ok` は観測のみ。途絶や頻度の低下をメッセージと values に出すが、レベルは `ok` のまま (止まっているのが正常なトピック用)。
- 期待する周期より短い `stale_timeout_sec` は、設定の読み取りで拒否する。
- 受信頻度は、途絶 (`stale_timeout_sec` 超え) したら 0 になる。

### イベントログ

状態の遷移、アクションの実行、接続の変化を、`evt=<domain>.<event> key=value ...` の 1 行で出す。書式とイベント名は [doc/monitoring.md](../doc/monitoring.md)。

| 言語 | API |
| :--- | :--- |
| Python | `mg_utils.event_log`: `EventLog(logger).info/warn/error(name, **fields)` (イベント名は位置専用。`from_` のように末尾の `_` は取り除く。`None` のフィールドは出さない)。読む側に `parse_event(message)` (イベントでなければ `None`) |
| C++ | `mg_utils/event_log.hpp`: `mg_utils::EventLog(logger).Info/Warn/Error(name, {{"key", value}, ...})`。ROS に依存しない書式の部分だけが `mg_utils/event_format.hpp` (`FormatEvent`、`QuoteIfNeeded`) |

イベント名・キーの書式に反すると、Python は `ValueError`、C++ は `std::invalid_argument` になる。ヘッダだけのライブラリなので、使うパッケージは `find_package(mg_utils REQUIRED)` して `ament_target_dependencies` に足す。

## rosbag の収録 (`launch/record_bag.launch.py`)

`ros2 bag record` を、mcap 形式で起動する。`mg_navigation/bringup.launch.py` と `mg_slam/bringup_slam_toolbox.launch.py` が、`record_bag:=true` のときに include する。`mg_slam` には、独自の `record_bag.launch.py` もある。

| 引数 | 内容 |
| :--- | :--- |
| `caller_pkg_name` | 収録するトピックの一覧 `params/record_topic_list.txt` を持つパッケージ |
| `record_bag_base_name` | bag のディレクトリ名の接頭辞 |
| `simulation` | `true` なら `--use-sim-time` を付ける |
| `record_monitoring` | 状態監視とイベントのトピック (`params/record_topics_monitoring.txt`) も収録する |

- 環境変数 `ROSBAG_PATH` が必須 (未設定ならエラーになる)。
- 保存先は `$ROSBAG_PATH/<YYYYMMDD>_bag/<接頭辞><番号>`。番号は、同じ日のディレクトリの最大値に 1 を足す。
- 収録するトピックは、`caller_pkg_name` のパッケージの `params/record_topic_list.txt` (1 行 1 トピック。空行と `#` の行は除く) に、`record_monitoring` が `true` のとき、状態監視とイベントのトピック (`mg_utils/params/record_topics_monitoring.txt`) を加えたもの。重複は 1 回だけ。パッケージの一覧が無いときは、ログに出して、監視用のトピックだけで起動する。
- 一覧の読み込みと結合は `mg_utils.record_topics` (`read_topic_file`、`merge_topics`)。`tools/scripts/record.sh` は同じ監視用のファイルを bash で読む。

走行ログを手動で収録するスクリプトは、`tools/scripts/record.sh` ([tools の README](../tools/README.md))。

## テスト

```bash
make test pkg=mg_utils
```

- `test/test_point_cloud.py`: `build_point_array`
- `test/test_event_log.py`: イベントの書式の作成と読み取り
- `test/test_record_topics.py`: 収録するトピックの一覧の読み込み・結合と、監視用の一覧の中身
- `test/test_monitoring_*.py`: 途絶・受信頻度の判定、しきい値、レベルの変化の検出、設定の読み取り (ROS 不要)
- `test/test_event_format.cpp`: C++ のイベントの書式 (`colcon test`)

ROS が必要な `topic_source`、`diagnostic_task` は、`make test` (ROS を読み込まない) では調べない。`develop` コンテナで ROS を読み込んで動かして確かめる。

## 依存

`package.xml` には、`ament_cmake`、`ament_cmake_python`、`rclpy`・`rosidl_runtime_py` (`topic_source` 用)、`diagnostic_msgs`・`diagnostic_updater` (`diagnostic_task` 用)、テストの `ament_cmake_gtest` が書かれている。`launch`、`launch_ros`、`ament_index_python`、`numpy` は、書かれていない。
