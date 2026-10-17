# geometry2_pkg

geometry2 (ROS 2 Humble) のパッケージのうち、**apt の版に不具合があるため、修正済みの上流の版をリポジトリに取り込んでいるもの**。apt の同名のパッケージの代わりに、ワークスペースでビルドして使う ([nav2_pkg](../nav2_pkg/README.md) と同じ方式)。

| パッケージ | 版 | 上流 | このリポジトリでの変更 |
| :--- | :--- | :--- | :--- |
| `tf2` | 0.25.24 | [ros2/geometry2](https://github.com/ros2/geometry2) のタグ `0.25.24` (humble ブランチ) | なし (タグのソースをそのままコピー) |

## 取り込んだ理由

apt の `ros-humble-tf2` 0.25.23 (2026-09-07 ビルド) には、`tf2_ros::Buffer::waitForTransform` と `tf2::BufferCore::testTransformableRequests` が 2 つの mutex を逆順に取り合うデッドロック (ABBA) がある。0.25.23 に入った競合状態の修正 ([ros2/geometry2#974](https://github.com/ros2/geometry2/pull/974)) で生じたもので、0.25.24 ([ros2/geometry2#990](https://github.com/ros2/geometry2/pull/990)、`tf2/src/buffer_core.cpp` の変更) で直っている。2026-10-08 時点で apt の最新は 0.25.23。

- 症状: `controller_server` の中で、TF の受信スレッド (`TransformListener`) とローカルコストマップの LaserScan の `tf2_ros::MessageFilter` のスレッドが止まる。以後 TF を受け取れず、RPP が `Lookup would require extrapolation into the future` / `Transform data too old` を出し続け、ロボットが動かなくなる (EKF は TF を出し続けている)
- 起きる条件: TF をすぐに変換できず、`MessageFilter` が待ちに入るとき (CPU 負荷が高いときなど)。シナリオテスト `dynamic_persistent_recovery` で、0.25.23 では 5 回中 5 回起き、0.25.24 と 0.25.20 では 4 回中 0 回だった
- 実機も同じイメージの apt の版を使うので、同じ条件で起きうる

0.25.23 と 0.25.24 の差は `tf2/src/buffer_core.cpp` だけ (ヘッダの変更なし、ABI は同じ)。そのため `tf2` だけを取り込み、`tf2_ros` などは apt の版のまま使う。ワークスペースの overlay にあるので、`source /root/ros2_ws/install/setup.bash` したあとは、apt の `/opt/ros/humble/lib/libtf2.so` より優先して読み込まれる。

## 外すとき

apt の `ros-humble-tf2` が 0.25.24 以上になったら、このディレクトリを消してイメージを作り直す。

```bash
docker run --rm --entrypoint bash mg_runtime:latest -c "apt-get update -qq && apt-cache policy ros-humble-tf2"
```

## 取り込みの方法

```bash
git clone -b humble https://github.com/ros2/geometry2.git
cd geometry2 && git archive 0.25.24 tf2 | tar -x -C <リポジトリ>/geometry2_pkg
```
