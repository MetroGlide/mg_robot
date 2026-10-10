# driver_watchdog_node

ドライバが止まったときに、走行を一時停止し、自分では復旧しないドライバは起動し直す、監視ノード。

## 背景

外部機器 (LiDAR・オドメトリ・モータドライバ) の接続が切れたとき、ドライバごとに挙動が違った。

| ドライバ | 切断時の挙動 |
| :--- | :--- |
| `wheel_odometry_node` | 連続エラーでシリアルを開き直し、デバイスの有無を 1 Hz で見る (自己復旧。`SerialDriverNode` で `motor_driver_node` と共通) |
| `motor_driver_node` | 同上 |
| `rplidar_ros` (`rplidar_node`) | 起動時に接続できなければ終了する。**走行中に止まると、プロセスは生きたままスキャンだけが出なくなり、復旧しない** |

そこで、復旧を次の 3 層に分けている。

```
層 1  ドライバ自身     自前のドライバ (オドメトリ・モータ) が切断を検知して再接続し、状態を配信する
層 2  launch の respawn  終了したプロセスを 2 秒後に起動し直す。起動時に機器が無くても、繋がるまで繰り返す
層 3  driver_watchdog   トピックの途絶を監視する。走行を一時停止し、止まった上流ドライバを終了させて層 2 に起動し直させる
```

再起動の判断をシーケンサに持たせないのは、シーケンサがドライバのプロセスを持っていないことと、手動ゴールや SLAM の収録など、シーケンサを通らない場面でも復旧してほしいため。シーケンサは一時停止の要求 (`pause_request`) を受けて止まり、全部の要求が解除されたら自動で再開するだけでよい。

## 動作

`bringup.launch.py` から、実機 (`simulation:=false`) のときだけ起動する。ドライバと同じ launch・同じコンテナで動くので、navigation と slam のどちらの bringup でも働く。

1. `monitored_sources` のトピックの最終受信時刻を見る。`stale_timeout_sec` を超えたら**異常**にする。`recover_hold_sec` のあいだ正常が続いたら**復旧**とする。
2. 異常のあいだ、`/diagnostics` に `driver_watchdog/<id>` を `ERROR` で出す。
3. `pause_on_fault: true` のとき、シーケンサの `pause_request` に `requester_id=driver_watchdog`、`active=true` を **1 Hz で出し続ける**。復旧したら `active=false` を出す。毎秒送るのは、`~/stop` で一時停止の要求が消えたり、シーケンサが再起動したりしても、異常のあいだは一時停止が戻るようにするため (異常のまま `~/start` もできない)。
4. `restart_node` を持つソースが `restart_after_sec` 以上途絶えたら、その名前のノードのプロセスに SIGTERM を送る。`kill_timeout_sec` たっても残っていれば SIGKILL を送る。launch の respawn が起動し直す。

### 再起動の仕組みと制約

- プロセスは、コマンドラインの `__node:=<ノード名>` で探す (`/proc` を見る)。**監視ノードと同じ PID 名前空間 (同じコンテナ) のプロセスだけ**が対象になる。
- `restart_cooldown_sec` のあいだは、続けて再起動しない (再起動した直後の立ち上がりで、また終了させないため)。
- 監視ノード自身の周期処理が `max_tick_gap_sec` 以上遅れたとき (CPU が詰まったときなど) は、判定を見送り、受信時刻を今に進めて、再起動しない。
- 起動直後は `startup_grace_sec` のあいだ、初回のメッセージが無くても異常にしない。
- `restart_enabled` は、launch で `respawn_drivers` に合わせる。起動し直されない状態で終了させないため。

## パラメータ

`params/driver_watchdog.yaml`。

| パラメータ | 既定値 | 内容 |
| :--- | :--- | :--- |
| `check_period_sec` | 0.5 | 判定の周期 |
| `stale_timeout_sec` | 1.0 | 途絶とみなす時間 (ソースごとに `<id>.stale_timeout_sec` で上書きできる) |
| `recover_hold_sec` | 2.0 | 復旧とみなすまでに、正常が続く時間 |
| `startup_grace_sec` | 30.0 | 起動直後の猶予 |
| `max_tick_gap_sec` | 2.0 | 周期処理の遅れの検知 |
| `pause_on_fault` | true | シーケンサに一時停止を出す |
| `pause_request_topic` | `/waypoint_sequencer_node/pause_request` | 一時停止を出すトピック |
| `pause_requester_id` | `driver_watchdog` | 一時停止の要求元の名前 |
| `restart_enabled` | true | 止まったドライバを終了させる (launch が `respawn_drivers` に合わせる) |
| `restart_after_sec` | 3.0 | 途絶がこの時間続いたら終了させる |
| `restart_cooldown_sec` | 10.0 | 続けて終了させない時間 |
| `kill_timeout_sec` | 3.0 | SIGTERM のあと、SIGKILL を送るまでの時間 |
| `monitored_sources` | `[top_lidar, odom, motor]` | 監視するソースの id |

ソースごとの設定 (`<id>.*`)。

| 項目 | 内容 |
| :--- | :--- |
| `topic` | 監視するトピック (必須) |
| `type` | `scan` (`LaserScan`)、`odom` (`Odometry`)、`connected` (`Bool`。`false` の間は異常) |
| `enabled` | 監視するか。launch が `top_lidar` を `use_lidar`、`odom` を `use_odom` に合わせる |
| `restart_node` | 途絶が続いたときに終了させるノード名。空なら再起動しない |
| `stale_timeout_sec` | このソースの途絶の判定時間 |
| `armed_on_first_message` | 初回のメッセージを受け取るまで監視しない |

既定のソース。

| id | トピック | 再起動するノード | 備考 |
| :--- | :--- | :--- | :--- |
| `top_lidar` | `scan_top_lidar` | `top_rplidar_node` | |
| `odom` | `odom` | なし | `wheel_odometry_node` が自分で再接続する |
| `motor` | `/motor_driver_node/connected` | なし | 1 Hz の接続状態。`armed_on_first_message` で、モータドライバが無い構成でも異常にならない |

## launch の引数 (`bringup.launch.py`)

| 引数 | 既定値 | 内容 |
| :--- | :--- | :--- |
| `use_driver_watchdog` | `true` | 監視ノードを起動する (シミュレーションでは起動しない) |
| `respawn_drivers` | `true` | ドライバの respawn。`false` にすると、監視ノードの再起動も無効になる |

## 無効にする・元に戻す

| したいこと | 方法 |
| :--- | :--- |
| 監視ごとやめる | `use_driver_watchdog:=false` |
| 一時停止だけやめる (通知と再起動は残す) | `pause_on_fault: false` |
| 再起動だけやめる | `restart_enabled: false` (または `respawn_drivers:=false`) |
| 従来のドライバの挙動に戻す | `use_driver_watchdog:=false respawn_drivers:=false` |

## 制約

- 診断の名前が `driver_watchdog/` のため、Web UI のセンサ欄 (`topic/` の `group: sensor`) には出ない。`/diagnostics` と `ros2 topic echo` で確認する。
- USB ハブごと落ちたときに、`/dev/ttyRobot-*` が戻るかはホスト側の問題で、このノードでは扱わない。
- LiDAR の途絶で止まるのはシーケンサの走行。手動ゴールなど、シーケンサを通らない走行の安全は、Nav2 の `collision_monitor` 側で別に守る。

## テスト

```bash
make test pkg=mg_drivers   # 判定ロジック (test/test_driver_watchdog_core.py)
```

実機なしの通し確認は、止まる偽の LiDAR を `respawn=True` で起動して、監視ノードが異常の検知 → 一時停止 → 終了 → 起動し直し → 復旧 → 一時停止の解除まで進むことを見る。
