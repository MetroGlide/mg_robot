# mg_simulator_client

Unity 製のシミュレータと ROS 2 をつなぐ、ROS-TCP-Endpoint を起動する launch。
Gazebo のシミュレーション ([mg_simulation](../mg_simulation/README.md)) とは別の経路で、現在は他のパッケージ・compose サービス・make ターゲットから使われていない。

## launch

`launch/bringup.launch.py`: `ros_tcp_endpoint` の `endpoint.py` を起動する。引数はない。

```bash
ros2 launch mg_simulator_client bringup.launch.py
```

## 構成

| パス | 内容 |
| :--- | :--- |
| `launch/bringup.launch.py` | ROS-TCP-Endpoint の起動 |
| `mg_simulator_client.rosinstall` | 取り込む外部リポジトリ: [Unity-Technologies/ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) (`main-ros2`) |
| `rviz/display.rviz` | RViz2 の設定 |

`ros_tcp_endpoint` は、Docker の `simulation` ステージが `vcs_import.py` で取り込んでビルドする ([doc/docker.md](../doc/docker.md))。実機向けの `runtime` と `develop` には含まれない。
