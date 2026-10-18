# localization_supervisor

自己位置推定が地図やほかの情報源と合っているかを検証し、ずれたときに復旧するためのフレームワーク。
特定のロボットに依存しない。ロボットごとのトピック・設定・部品は、使う側のパッケージから与える。

メッセージは [localization_supervisor_msgs](../localization_supervisor_msgs/README.md)。

## 構成

| ディレクトリ | 内容 |
| :--- | :--- |
| `localization_supervisor/core/` | ROS に依存しない部分 (姿勢の計算、占有格子とスキャンの照合、状態遷移、ゲートの調停、診断の内容) |
| `localization_supervisor/ros/` | ノードと、ROS に依存する部品 (診断のタスク、イベントの出力) |
| `scripts/` | ノードを起動するラッパー |

## ノード

| 実行ファイル | 内容 |
| :--- | :--- |
| `localization_supervisor_node.py` | AMCL・GNSS・EKF・スキャンを見比べて AMCL のずれを検知し、通知する。有効にすると、切り離して復旧する ([doc/supervisor_node.md](./doc/supervisor_node.md)) |
| `gate_arbiter_node.py` | `SetBool` で開閉するゲートを、要求元ごとに調停する ([doc/gate_arbiter.md](./doc/gate_arbiter.md)) |

launch は持たない。使う側のパッケージが、自分の設定で起動する。

## 依存

[package.xml](./package.xml) を参照。ロボット固有のパッケージには依存させない。

## テスト

```bash
make test pkg=localization_supervisor
```

`test/test_framework_independence.py` は、`package.xml` の依存と Python の import に、ロボット固有のパッケージ (`mg_` で始まるもの) が無いことを確かめる。

## ドキュメント

| ファイル | 内容 |
| :--- | :--- |
| [doc/supervisor_node.md](./doc/supervisor_node.md) | 監督ノードの検知・状態遷移・復旧・出力 |
| [doc/gate_arbiter.md](./doc/gate_arbiter.md) | ゲートの調停の規則・出力・パラメータ |
