# ドライバの切断からの復旧

[mg_drivers の README](../README.md) から分けた文書。値は yaml と launch が正。

シリアル接続の機器 (`wheel_odometry_node`、`motor_driver_node`) は、共通の基底クラスで、切断から自分で再接続する。

- `SerialDevice` (`include/mg_drivers/base/serial_device.hpp`): シリアルポートの開閉と、デバイスファイルの有無。`MotorDriver` と `WheelOdometry` が継承し、接続確認の通信 `probe()` を実装する (モータは停止の指令、オドメトリは読み出し)。
- `SerialDriverNode` (`include/mg_drivers/base/serial_driver_node.hpp`): 2 つのノードの基底クラス。通信の結果を `handle_serial_error()` で受け取り、次のことをする。

| 動作 | 内容 |
| :--- | :--- |
| 連続エラーでの開き直し | 連続 `<prefix>.error_recovery_count` (`odometry.` / `motor_driver.`) 回のエラーで、シリアルを開き直す (1 回だけ) |
| 定期の確認 (`health_check`) | デバイスファイルが消えたら、ポートを閉じる。戻ったら開き直し、`probe()` で通信を確かめる。開けていない、または連続エラーが閾値以上なら、同様に開き直して確かめる。エラーが 1 回以上あれば、`probe()` で確かめる (指令が無いあいだの切断に気づくため) |
| `~/connected` (`Bool`) | `true`: ポートが開いていて、デバイスがあり、連続エラーが 0 |
| 診断 `<ノード名>: connection` (`diagnostic_updater`) | 接続していれば `OK`、切れていれば `ERROR` (メッセージは理由: `device missing` / `port closed` / `communication errors`)。`hardware_id` は `motor_driver` / `odometry`。`values`: `consecutive_errors`、`device_present`、`reconnect_attempts` (今回の切断のあいだに開き直した回数)、`disconnect_count` (起動してからの切断の回数) |
| イベント (`driver.*`) | 接続の状態の変化を、`driver.connected` / `driver.disconnected` (`device`、`reason`) に出す。開き直しは `driver.reconnect_attempt` (`device`、`reason`、`count`) で、最初の 1 回と、以降は複数回ごとに出す (再試行で埋めないため)。`motor_driver_node` は E-Stop の変化を `driver.estop_changed` (`active`) に出す ([doc/monitoring.md](../../doc/monitoring.md)) |

`motor_driver_node` は、診断 `motor_driver_node: emergency_stop` も出す (E-Stop が有効なら `WARN`)。診断は集約で `/Robot/Drivers/Motor` に入る。

オドメトリは、閾値に達したあと 1 回だけ開き直し、以降は定期の確認で再試行する。

モータドライバは、チェックサムの誤りを接続の判断に使わない (応答があったため)。

自分では直せないものは、プロセスごと起動し直す。`respawn_drivers:=true` のとき、次のノードは終了コードによらず、終了の少し後に起動し直される。起動時に機器が無くて終了するもの (`rplidar_node` など) は、機器が繋がるまで繰り返す。`false` なら起動し直さない。

`wheel_odometry_node`、`top_rplidar_node`、`ublox_gps_node` (または `gps_driver`)、`motor_driver_node`

走行中に `rplidar_node` が止まる (プロセスは生きたまま、スキャンだけ出なくなる) 場合は、プロセスが終了しないので respawn では直らない。`driver_watchdog_node` がスキャンの途絶を見て、プロセスを終了させ、respawn に起動し直させる。あわせて、復旧するまでシーケンサを一時停止する ([doc](./driver_watchdog.md))。
