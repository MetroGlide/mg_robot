# 開発ガイド

`slam_gnss_2d` を変更するときのルールと、開発の環境。構成は [design.md](./design.md)。

## 設計のルール

### 1. ROS に触れる場所を限る

Core 層の `pose_graph/`、`scan_matching/`、`gnss/`、`optimizer/`、`map_manager/` に、ROS のメッセージ型 (`LaserScan`、`Odometry` など) を持ち込まない。ログの `RCLCPP_*` だけを使う。
ROS のメッセージ型・通信に触れてよいのは、次の場所。

- `input/ros2/` (トピックと rosbag2 の入力)
- `core/slam_node_base.cpp`、`core/config_loader.cpp`、`core/geometry.cpp`
- `ros/` (可視化、TF、サービス)
- `nodes/`、`tools/`

### 2. 新しい実装は基底クラスを継承する

次の基底クラスに対して実装する。基底クラスのメソッドを変えるときは、先に [design.md](./design.md) を更新する。

| 変えたいもの | 基底クラス |
| :--- | :--- |
| 入力のソース (スキャン・オドメトリ・GNSS) | `ScanSourceBase`、`OdomSourceBase`、`GnssSourceBase` (`input/base.hpp`) |
| ポーズグラフの構築 | `PoseGraphBuilderBase` (`pose_graph/base.hpp`) |
| スキャンマッチング | `ScanMatcherBase` (`scan_matching/base.hpp`) |
| 地図の描画 | `MapRendererBase` (`map_manager/base.hpp`) |

差し替えは、`SlamNodeBase` と `component_factory` の、部品を作る部分の変更で済むように設計している。

### 3. スキャンの座標は入力の層で `base_link` に直す

`input/ros2/ros_adapter.cpp` が `ScanData` を作る時点で、スキャンの角度を `base_link` の座標系に直す。センサの取り付けの回転は、`/tf_static` (`robot_state_publisher` が URDF から配信) から取る。YAML には書かない。
センサの物理的な配置の唯一の正解は URDF にある。YAML との二重の管理は、URDF との食い違いが実行時まで見つからない。
ただし GNSS のレバーアーム (`gnss.lever_arm`) は、パラメータで持つ。URDF の `base_link`→`gps_link` と合わせる。

### 4. 反復して収束する処理には、連続失敗の対策を置く

自分の出力を、次回の入力の初期値にする処理 (ICP の初期値、ループの候補のスコアなど) は、収束の失敗が連鎖する。
`failure_streak` のカウンタと、上限に達したときのフォールバックを置く。

- 収束しなかったら、カウンタを増やす。上限に満たなければ、そのフレームを飛ばす。
- 上限に達したら、低い信頼度のエッジとして受け入れて (オドメトリへのフォールバック)、連鎖を抜ける。
- 成功したら、カウンタをリセットする。

実装は `ScanMatchingBuilder` (`scan_matching.max_failure_streak`) と `LoopClosureBuilder` (`loop_closure.max_failure_streak`)。

### パラメータを追加するとき

次の 3 か所を、同時に更新する。

1. `core/config.hpp`: `SlamConfig` のフィールド (既定値を付ける)
2. `core/config_loader.cpp`: `declare_params` に宣言を、`build_config` に読み出しを足す
3. `params/slam_gnss_2d.yaml`: エントリ (コメントに単位と効果を書く)。`parameters.md` にも書く

`declare_parameter` の前に `get_parameter` を呼ぶと、`ParameterNotDeclaredException` になる。

## 環境

### ルートの環境で動かす (通常)

ルートの `make` と compose を使う ([doc/commands.md](../../doc/commands.md))。

```bash
make build svc=slam
make slam-gnss-2d                           # SLAM ノードと RViz2
make offline-slam-gnss-2d BAG=<bag>
make reoptimize INPUT_DIR=<dir> SAVE_DIR=<dir>
make test pkg=...                           # Python のテスト (C++ は含まない)
```

### 単体の環境で開発する

`slam_gnss_2d/` 直下に、単体の `Dockerfile`、`compose.yaml`、`Makefile`、`.env.example` がある。ルートの環境とは別で、ホストの `slam_gnss_2d/` を `/app` にマウントし、`colcon build` から行う。
使う前に `cp .env.example .env` して、データのパス (`DATA_DIR`、`ROSBAG_FILE`、`INPUT_DIR`、`SAVE_DIR`) を編集する。

```bash
cd slam_gnss_2d
make build              # イメージ (dev) のビルド
make run                # オンラインの SLAM
make offline BAG=<bag>  # オフライン。BAG は ROSBAG_FILE を上書きする
make reoptimize INPUT=<dir> BAG=<bag> SAVE=<dir>
make test               # colcon test (gtest)
make shell              # コンテナの bash
```

ルートの環境と、環境変数の名前が違う。単体の `reoptimize` は `ROSBAG_FILE`、ルートは `BAG_PATH`。

## テスト

C++ の gtest (`slam_gnss_2d/test/`、16 本)。ルートの `make test` には含まれない。

```bash
# 単体の環境
cd slam_gnss_2d && make test

# ルートの develop コンテナ
make shell-develop
cd /root/ros2_ws && colcon build --packages-select slam_gnss_2d --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select slam_gnss_2d && colcon test-result --all
```

主なテスト: アンカー、デスキュー、幾何、ICP・NDT・CSM のマッチャー、ループクロージャ、iSAM2、カウンティングレンダラー、ナビゲーションブリッジのコア、地図の保存、時系列。

## 評価

- `make bag-eval-slam SLAM_DIR=<dir>`: SLAM の出力を、GNSS (RTK) と比べる。
- `tools/scripts/run_slam_variant.sh`: パラメータの変種を、同じ bag で比べる。

どちらも [tools の README](../../tools/README.md) を参照。
