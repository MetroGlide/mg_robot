# localization_supervisor_msgs

自己位置推定の検証と復旧のフレームワーク [localization_supervisor](../localization_supervisor/README.md) のメッセージ。ノードは持たない。

## メッセージ (`msg/`)

| 型 | 内容 | 配信元 |
| :--- | :--- | :--- |
| `LocalizationStatus` | 自己位置推定の監督の状態 (`NORMAL`〜`DEGRADED`) と、判定に使った値 | `localization_supervisor_node` |
| `GateState` | ゲートの調停の状態。`desired` (要求元の意図)、`applied` (ゲートへの反映済みか)、`holders` (閉じている要求元) | `gate_arbiter_node` (トピック `~/state`、transient_local) |

各フィールドの意味は `.msg` ファイルのコメントを参照。
