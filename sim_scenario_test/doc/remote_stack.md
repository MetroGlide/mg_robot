# リモートスタック (--remote-stack)

シミュレータと scenario_runner を動かすマシン (開発PC) とは別のマシン (実機PCなど) で、
ナビゲーションスタックを動かしてシナリオテストを行う仕組みです。

`run` / `run-all` に `--remote-stack URL` を渡すと、シナリオごとに次の順で実行します。

1. プロファイルの `stack` (とシナリオの `stack_args`) を変数展開し、`POST /scenario-stack/start` で起動を依頼する
2. `scenario_full.launch.py launch_stack:=false` でシミュレータと scenario_runner だけをローカルで起動する
3. 終了後 (失敗・タイムアウトでも)、`GET /scenario-stack/logs` を `<results-dir>/<シナリオ名>/stack.log` に保存し、
   `POST /scenario-stack/stop` で停止する

スタックの起動に失敗すると ERROR (`remote stack failed to start`) となり、`--infra-retries` の対象になります。
`--attach` とは同時に指定できません。

## HTTP プロトコル

他のロボットでも、下記を実装したサーバがあれば使えます。応答はすべて JSON です。

| メソッド・パス | リクエスト | 応答 |
|---|---|---|
| `POST /scenario-stack/start` | `{"package": str, "file": str, "args": {キー: 値}}`。`file` はパッケージの share ディレクトリからの相対パス。起動済みのスタックがあれば作り直す | `{"success": bool, "message": str}` |
| `POST /scenario-stack/stop` | `{}` | `{"success": bool, "message": str}` |
| `GET /scenario-stack/logs` | なし | `{"logs": str}` |

## ネットワーク上の注意

- 両方のマシンで同じ `ROS_DOMAIN_ID` と DDS の設定にする。テスト用の domain を通常のものと分けると、誤って実機のドライバへ `/cmd_vel` が届くことを防げる。
- `/clock` はシミュレータ側から配信され、スタック側は `use_sim_time` で動く。
