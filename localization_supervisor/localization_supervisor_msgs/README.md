# localization_supervisor_msgs

自己位置推定の検証と復旧のフレームワーク [localization_supervisor](../README.md) のメッセージとサービス。ノードは持たない。

## メッセージ (`msg/`)

| 型 | 内容 | 配信元 |
| :--- | :--- | :--- |
| `LocalizationStatus` | 監督の状態 (`NORMAL`〜`DEGRADED`)、疑わしい情報源 (`blame`・`contaminated`)、停止の要求、安全の信号 (footprint が計画用地図の致死セルにかかっているか)、判定のインスタンスごとの結果 (`checks`) | `localization_supervisor_node` |
| `LocalizationCheck` | 判定のインスタンスの 1 周期分の結果 (状態、`score`、累積、記録用の値) | `LocalizationStatus.checks` |
| `GateState` | ゲートの調停の状態。`desired` (要求元の意図)、`applied` (ゲートへの反映済みか)、`holders` (閉じている要求元) | `gate_arbiter_node` (トピック `~/state`、transient_local) |

## サービス (`srv/`)

| 型 | 内容 | 提供元 |
| :--- | :--- | :--- |
| `SetProfile` | 監督のプロファイルを切り替える (空は基の設定) | `localization_supervisor_node` (`~/set_profile`) |

各フィールドの意味は `.msg`・`.srv` ファイルのコメントを参照。
