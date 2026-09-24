# sim_scenario_test

Gazebo (Fortress) 上でロボットのナビゲーションを検証する、**ロボット非依存**のシナリオテスト基盤です。
シナリオは YAML で宣言的に書き、ロボット固有の設定は「プロファイル」と「プラグイン」で外から注入します。
このパッケージは `mg_*` など特定のロボットのパッケージに依存しません (テストで検査しています)。

## できること

- ロボットのリスポーン (移動 → 自己位置の初期化 → 収束確認 → コストマップのクリア)
- 単体ゴール・複数ゴール・waypoint ファイルによるナビゲーション (Nav2 の NavigateToPose)
- 障害物の静的・動的な配置と削除、等速で移動する障害物 (歩行者など)
- 時間・進捗・位置・ゴール到達をきっかけにしたイベント実行 (タイムライン)
- 走行中の常時監視 (monitor) と、走行後の判定 (expectation)。PASSED / FAILED / ERROR を区別
- シナリオごとにシミュレータとナビゲーションを起動し直すヘッドレス自動実行、JUnit と結果 JSON の出力
- Python の dict / pytest でシナリオを組み立てて実行

## 構成

```
シナリオ YAML ─┐
              ├─ Loader (厳格な検証) ─ ScenarioEngine ─ RunDriver (nav2_goals / プラグイン提供)
プロファイル ───┘        ▲                   │         ├─ Timeline (trigger → action)
 (ロボット固有)          │                   │         ├─ Monitor (常時監視)
プラグイン ─ 型を登録 ───┘                   │         └─ SimulationBackend (gazebo_fortress)
                                             └─ 結果 (PASSED / FAILED / ERROR, result.json, JUnit)
```

詳細は [docs/architecture.md](docs/architecture.md) を参照してください。

## 依存

ROS 2 (Humble) の `rclpy`, `nav2_msgs`, `nav2_simple_commander`, `tf2_ros`, `sensor_msgs`, `diagnostic_msgs`, `std_srvs`,
`ros_gz` 一式と Gazebo Fortress (`ign` コマンド)、PyYAML。ロボット固有のパッケージには依存しません。

## クイックスタート (別のロボットで使う)

1. **プロファイルを書く**: 起動する sim / ナビゲーションの launch、ロボットのエンティティ名、座標系、
   ワールドごとの地図などを YAML にまとめる ([docs/profile_reference.md](docs/profile_reference.md))。
   `share/<your_pkg>/profiles/<name>.yaml` に置き、CMake で
   `ament_index_register_resource("sim_scenario_test.profiles")` を呼ぶと名前で参照できる。
2. **(必要なら) プラグインを書く**: ロボット固有の走行ドライバや action / trigger を登録する
   ([docs/plugin_development.md](docs/plugin_development.md))。標準の Nav2 だけなら不要。
3. **シナリオを書く** ([docs/scenario_format.md](docs/scenario_format.md)) 。`profile:` にプロファイル名を指定する。
4. 検証と実行:

```bash
ros2 run sim_scenario_test scenario_cli.py validate scenarios/*.yaml        # 静的検証 (シミュレータ不要)
ros2 run sim_scenario_test scenario_cli.py list-types --profile <name>      # 使える型の一覧
ros2 run sim_scenario_test scenario_cli.py run scenario.yaml                # 1 本を起動から判定まで実行
ros2 run sim_scenario_test scenario_cli.py run-all scenarios/ --tags smoke  # スイート実行
```

## CLI

| コマンド | 内容 |
|---|---|
| `validate <files>...` | シナリオを検証する。未知のキー・型・未定義の障害物・プロファイルにないワールドなどをエラーにする |
| `list-types [--profile]` | 登録済みの action / trigger / expectation / monitor / driver / backend / waypoint 形式を一覧する |
| `run <files>...` / `run-all <files\|dirs>...` | `ros2 launch` でシミュレータとスタックを起動して実行する。1 本ごとに起動し直す |

`run` / `run-all` の共通オプション:

| オプション | 内容 |
|---|---|
| `--profile NAME` | シナリオの `profile:` を上書きする |
| `--scenario-dir DIR` | ファイルの代わりに名前 (`<name>` → `<name>.yaml`) を渡したとき、DIR 配下 (再帰) から探す。複数指定可。ディレクトリを渡すと配下 (再帰。`data/` は除く) のシナリオを集める |
| `--gui` | シミュレータの GUI を表示する (既定はヘッドレス) |
| `--attach` | 起動済みのシミュレータ・スタックに接続して実行する (起動は行わない) |
| `--results-dir DIR` | 結果の保存先 (既定 `~/.ros/scenario_results/<日時>`) |
| `--timeout SEC` | 1 本あたりの壁時計の上限 (既定 1800 秒)。超えると ERROR |
| `--tags a,b` / `--exclude-tags a,b` / `--repeat N` | いずれかのタグを持つものに絞り込み / いずれかのタグを持つものを除外 / 各シナリオを N 回繰り返す |
| `--seed N` | シナリオの seed を上書きする (繰り返しごとに +1) |

終了コードは 0=PASSED、1=FAILED、2=ERROR (複数実行では最悪のもの)。

## 結果

`<results-dir>/<シナリオ名>/` に `result.json` (チェックごとの結果とイベント)、`launch.log`、
ルートに `junit.xml` を保存します。`FAILED` は「ロボットが期待どおりに動かなかった」、
`ERROR` は「テスト基盤・セットアップの失敗 (起動しない、障害物が生成されない、シミュレータが止まった等)」を意味します。

## テスト

```bash
make test pkg=sim_scenario_test      # ROS・シミュレータ不要のユニットテスト
```

## ドキュメント

| ファイル | 内容 |
|---|---|
| [docs/architecture.md](docs/architecture.md) | 層構成、実行の流れ、結果の定義、拡張ポイント |
| [docs/scenario_format.md](docs/scenario_format.md) | シナリオ YAML (v2.0) の全項目と組み込みの型 |
| [docs/profile_reference.md](docs/profile_reference.md) | ロボットプロファイルの全項目、変数展開 |
| [docs/plugin_development.md](docs/plugin_development.md) | プラグイン (独自の型) の作り方 |
| [docs/python_scenarios.md](docs/python_scenarios.md) | Python / pytest でシナリオを書く方法 |
