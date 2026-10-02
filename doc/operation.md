# 運用の流れ

実機で、地図を作って走らせ、走行ログを解析するまでの流れ。各手順の詳細は、リンク先のパッケージのドキュメントを参照。
コマンドはすべて、リポジトリのルートで実行する。make ターゲットの全体は [commands.md](./commands.md)、`.env` は [environment.md](./environment.md)。

```
 1. 準備        ビルド・.env・デバイス
 2. 地図作成    走行しながら収録 → SLAM → 保存 (map / gnss_transform.yaml)
 3. 地図の配置  MAP_PATH に置く
 4. ウェイポイント  waypoint-tool で作成
 5. 自律走行    make navigation + Web UI
 6. 解析        rosbag の収録と、make bag-* での評価
```

## 1. 準備

```bash
cp .env.example .env            # 環境に合わせて編集する (environment.md)
make build-robot                # 実機向けにイメージをビルド
```

- センサは `/dev/ttyRobot-*` の名前で接続する (udev ルール: `mg_drivers/config/usb-serial-devices.rules`。デバイスの一覧は [system_architecture.md](./system_architecture.md#センサとデバイス))。
- 実機では `.env` の `SIMULATION=false` にする。
- GUI (RViz2) を使う PC では、先に `make xhost` を実行する。

## 2. 地図作成

GNSS 拘束付き 2D SLAM (`slam_gnss_2d`) で作る。詳細は [slam_gnss_2d の README](../slam_gnss_2d/README.md)。

```bash
make slam-gnss-2d               # SLAM ノードと RViz2 だけを起動する (センサのドライバは含まない)
```

センサのドライバと SLAM を一緒に起動するには、`mg_bringup` の launch を使う。compose にはこの launch を起動するサービスがないので、develop コンテナの中で実行する。

```bash
make shell-develop
ros2 launch mg_bringup bringup_slam_gnss_2d.launch.py
```

1. 地図にしたい範囲を走行する。必要なら、同時に rosbag を収録しておく (`./tools/scripts/record.sh -m slam`。[tools の README](../tools/README.md))。
2. 走行が終わったら、地図を保存する。

   ```bash
   make shell-develop
   ros2 run slam_gnss_2d save_slam_map_cli -d /root/ros2_data/map/<地図名>
   ```

   `map.pgm`、`map.yaml`、`gnss_transform.yaml`、`pose_graph.json` が保存される。
3. 収録した bag があれば、あとからオフラインで再処理や再最適化ができる。

   ```bash
   make offline-slam-gnss-2d BAG=<bag のパス>
   make reoptimize INPUT_DIR=<SLAM の出力> SAVE_DIR=<結果の保存先> BAG_PATH=<bag のパス>
   ```

slam_toolbox による SLAM (`make slam`) もある。GNSS を使わない場合の方法で、[mg_slam の README](../mg_slam/README.md) を参照。

## 3. 地図の配置

走行に使う地図を、`MAP_PATH` (`/root/ros2_data/map`) に置く。

| ファイル | 内容 |
| :--- | :--- |
| 測位用の地図 (`*.yaml` + `*.pgm`) | AMCL が使う。SLAM の `map.yaml` / `map.pgm` |
| 計画用の地図 | Nav2 の経路計画が使う。測位用と同じファイルでもよい |
| `gnss_transform.yaml` | GNSS (UTM) を地図の座標に変換する。SLAM の出力に含まれる |
| `map_list.txt` | `waypoint-editor` と `make bag-plot-gnss-map` が読む地図の一覧 |

`make navigation` が読む地図ファイルは起動引数で決まる ([environment.md](./environment.md#地図の指定))。

## 4. ウェイポイントの作成

[waypoint-tool](https://github.com/Chu-son/waypoint-tool) で、地図の上にウェイポイントを置き、`waypoint.yaml` を出力する (`WAYPOINT_PATH` に置く)。
形式と手順は [mg_waypoint_navigation の doc/waypoint_format.md](../mg_waypoint_navigation/doc/waypoint_format.md)。
地図を位置合わせして回した場合は、同じツールで `gnss_transform.yaml` も出力できる。

## 5. 自律走行

```bash
make navigation                 # 自己位置推定 + Nav2 + ウェイポイントシーケンサ
make ui-all                     # Web UI (foxglove-bridge・system-manager・web-ui・diagnostics)
```

- ブラウザで `http://<ロボットの IP>:8080` を開く。操作は [mg_ui の README](../mg_ui/README.md)。
- 起動時は、GNSS の測位が良くなるのを待って、AMCL と EKF の初期姿勢を決める。自己位置推定の仕組みと、ずれたときの動作は [mg_navigation の README](../mg_navigation/README.md)。
- ウェイポイントの走行、一時停止、地図の切り替えは Web UI から行う。

## 6. 走行ログの収録と解析

```bash
./tools/scripts/record.sh -m all       # 全センサを収録 (-m slam なら軽量版)
make bag-summary                       # ROSBAG_FILE の統計サマリー
make bag-plot-gnss CIRCLES=1           # GNSS の軌跡と精度
make bag-eval-localization BAG=<bag>   # 自己位置推定の評価
```

解析ツールの一覧と使い方は [tools の README](../tools/README.md)。bag の統計は、同じディレクトリの `summary.md` に保存される。

## シミュレーションとテスト

実機がなくても、Gazebo で同じナビゲーションスタックを動かせる。

```bash
make build-sim
make gazebo-simulation                 # 端末 1
make navigation                        # 端末 2 (.env の SIMULATION=true)
make scenario-test-all TIER=smoke      # 実装を変えたあとの回帰テスト
```

- シミュレータは [mg_simulation の README](../mg_simulation/README.md)。
- シナリオテストは [mg_scenario_test の README](../mg_scenario_test/README.md)。
