# Docker 環境

ローカルに ROS 2 環境は不要。ビルド・実行・テストはすべて Docker コンテナ内で行う。
コンテナにはリポジトリを `/app/` にマウントし、ROS ワークスペースは `/root/ros2_ws` (`src/mg` が `/app` へのシンボリックリンク) に置く。

## 前提条件

- Docker Engine 24.0 以上
- Docker Compose v2.20 以上
- BuildKit 有効 (Docker 23.0 以上は既定で有効)
- GPU を使う場合 (Gazebo のシミュレーションとシナリオテストが対象)
  - **Nvidia**: [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) が必要
  - **AMD**: ROCm 対応の `/dev/kfd`、`/dev/dri` が使えること

## イメージ構成

`docker/Dockerfile.base` のマルチステージ構成。

```
runtime-base      apt / pip / 依存解決 (vcs import、rosdep) をキャッシュ層に分離
├─ runtime        全ソースを colcon build。実機用 (Gazebo を含まない)
│   └─ web-ui     runtime + foxglove-bridge + ビルド済みの React フロントエンド
└─ develop-pkgs   develop 系の apt (RViz2、foxglove-bridge、rqt、Docker CLI、Qt など)
    ├─ develop    develop-pkgs + runtime のビルド成果物
    └─ simulation-pkgs   ros-gz (sim / bridge / image / interfaces)
        └─ simulation    develop の成果物 + シミュレータ系パッケージの追加ビルド
frontend-builder  node:20-slim。Web UI をビルドして dist/ だけを web-ui へ渡す
```

| ステージ | 使うサービス (compose) | イメージ名 |
| :--- | :--- | :--- |
| `runtime` | slam / slam-gnss-2d / offline-slam-gnss-2d / reoptimize-slam / navigation / rosbag-replay / map-preview / diagnostics / foxglove-bridge / scenario-remote-stack | `mg_runtime` |
| `develop` | develop / rviz2 系 / obstacle-detection-replay / waypoint-editor / system-manager | `mg_develop` |
| `simulation` | gazebo-simulation / scenario-test / scenario-env | `mg_simulation` |
| `web-ui` | web-ui | `mg_web_ui` |

- `simulation` ステージは、`mg_simulator_client` の依存を `vcs_import.py` で取り込み、`mg_simulation` / `sim_scenario_test` / `mg_scenario_test` / `mg_simulator_client` / `ros_tcp_endpoint` だけを追加でビルドする。
- 実機向けの `runtime` と `develop` には Gazebo を含まない。
- `simulation-pkgs` に、`kisak-mesa` の PPA を追加する手順がコメントアウトされている (現在は無効)。

## ビルド

```bash
make build-robot                 # 実機向け: slam(runtime)・develop・web-ui を一括ビルド
make build-sim                   # シミュレータ込み: 上記 + gazebo-simulation
make build svc=slam              # 特定サービスのイメージだけビルド
make build-no-cache svc=slam     # キャッシュ無効でビルド
```

`build-real` は `build-robot` の、`build-all` は `build-sim` の別名。`make` の全ターゲットは [commands.md](./commands.md) を参照。

`make build` は、先に `docker/collect_deps.sh` を実行する。`make` を使わずに Docker を直接使うときは、同じスクリプトを自分で先に実行する。

```bash
bash docker/collect_deps.sh
docker compose -f compose.yaml build slam
docker compose -f compose.yaml -f compose.gpu.nvidia.yaml build gazebo-simulation
```

## 依存ファイルの収集 (`docker/collect_deps.sh`)

`package.xml`・`*.rosinstall`・`*.repos`・`requirements.txt` だけを `docker/deps/` に集める。`runtime-base` はこのファイルだけをコピーして依存を解決するため、ソースを変更しても rosdep などのキャッシュ層が無効にならない。

- 次のディレクトリは収集の対象外: `.git`、`build`、`install`、`log`、`node_modules`、`docker`、`mg_simulation`、`mg_scenario_test`、`sim_scenario_test`、`mg_simulator_client`。
  これらの依存は `simulation` ステージで解決する。
- 自動では集まらないファイルは、`docker/deps_extra.txt` にリポジトリルートからの相対パスで書く。現在は `docker/vcs_import.py`・`docker/pip_requirements.py`・`remote.xml` を指定している。
- `docker/vcs_import.py` は `*.rosinstall` / `*.repos` から外部リポジトリを取り込む (nmea_navsat_driver、rplidar_ros、kiss-icp の fork など。`mg_drivers/mg_drivers.rosinstall`)。
- `docker/pip_requirements.py` は、各パッケージの `requirements.txt` を pip でインストールする。

## GPU override

| ファイル | 内容 |
| :--- | :--- |
| `compose.gpu.nvidia.yaml` | `deploy.resources.reservations.devices` で Nvidia GPU を割り当てる |
| `compose.gpu.amd.yaml` | `/dev/kfd`、`/dev/dri` をマウントする |

`.env` の `USE_GPU` (`none` / `nvidia` / `amd`) に応じて、Makefile が `-f` で自動的に適用する。コマンドラインで `make gazebo-simulation USE_GPU=nvidia` のように上書きもできる。
対象サービスは `gazebo-simulation`・`scenario-test`・`scenario-env`。

## 共通のコンテナ設定

- ネットワークは `host`、`privileged: true`、`/dev` をマウントする (センサのシリアルポートを使うため)。
- GUI (RViz2、Gazebo) を使うときは、先にホストで `make xhost` を実行する。
- データの置き場として、ホストの `${HOME}/ros2_data` を `/root/ros2_data` にマウントする ([environment.md](./environment.md))。
- DDS は `rmw_cyclonedds_cpp`。開発 PC と実機 PC をまたぐシナリオテストでは、`docker/cyclonedds/remote.xml` で通信相手とインターフェースを指定する ([mg_scenario_test の README](../mg_scenario_test/README.md))。
- `develop` コンテナの起動コマンドは `docker/docker-entrypoint.sh` (`sleep infinity`)。
