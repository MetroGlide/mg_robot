# 設定ファイル

情報源・判定のインスタンス・総合・状態遷移・アンカー・プロファイルの組み合わせを書く YAML。ノードもオフライン評価も、このファイルを `core/config.py` で読む。
読み込むときにすべてを検証し、知らないキー・型の違い・存在しない情報源・種類の違いがあれば落ちる。プロファイルもすべて検証する。
最小の例は [config/examples/](../config/examples/)。値はロボットのセンサと地図に合わせて決める (オフライン評価で確かめる)。

## 全体

| キー | 内容 |
| :--- | :--- |
| `version` | 約束の版 ([plugins.md](./plugins.md#約束の版)) |
| `sources` | 情報源 (下記) |
| `timeline` | 時刻のそろえ方 |
| `detectors` | 判定のインスタンス (下記) |
| `enabled_detectors` | 書くと、ここに並べたインスタンスだけを有効にする (主にプロファイルで使う) |
| `fusion` | 総合の方式 `type` と `params` |
| `trust.require_normal` | アンカーを残すのに、正常であることを求めるインスタンス |
| `state_machine` | 状態遷移 (下記) |
| `recovery.convergence` | 復旧の収束の確認に使うインスタンス (すべて正常な周期を数える) |
| `anchors` | アンカー (下記)。書かなければ残さない |
| `grace.startup_sec`、`grace.after_recovery_sec` | 起動直後・復旧直後に総合しない時間 [s] |
| `events.prefix` | イベントの名前の前置き (`<prefix>.state_changed`) |
| `extras` | 注入した入力のアダプタが `Snapshot.extras` に入れる値の名前 (判定の `EXTRAS` はここにあるものに限る) |
| `profiles` | プロファイルの名前と、重ねる設定 (ファイルのパスは設定ファイルの場所から。または辞書) |

## sources

```yaml
sources:
  <名前>:
    kind: pose | motion | reference
    derived_from: [<名前>, ...]   # 融合の結果なら、元の情報源 (元は独立の情報源に限る)
    input: {type: <アダプタ>, ...} # 入力のアダプタとその引数 (plugins.md の入力のアダプタ)
```

`derived_from` を持つ情報源は、疑わしい情報源の特定 (`blame`) に使わず、出力が汚れているか (`contaminated`) の判定に使う ([design.md](./design.md#疑わしい情報源の切り分け-blamecontaminated))。

## timeline

| キー | 内容 |
| :--- | :--- |
| `motion_priority` | 時刻をそろえるのに使う動きの情報源の、優先の順 (書かなかった動きの情報源は後ろに続く) |
| `horizon_sec` | 標本の履歴を残す時間 [s] |
| `max_extrapolation_sec` | 動きの最新の標本より、これだけ新しい時刻までは最新の値を使う [s] |
| `alignment_std_per_m`、`alignment_std_per_rad` | 時刻を移すとき、移した距離・回転に比例して足す標準偏差 |

## detectors

```yaml
detectors:
  <インスタンスの名前>:
    type: <登録名 | パッケージ.モジュール:クラス>
    sources: {<引数>: <情報源の名前>, ...}   # 判定の SOURCES と同じ引数をすべて
    params: {...}                            # 判定の PARAMS
    weight: <数>      # cusum: 累積の重み (0 で総合に効かない)
    allowance: <数>   # cusum: score のうち、これを超えた分を累積する
    decay: <数>       # cusum: 前の周期の累積に掛ける (1 未満で古い証拠を忘れる)
    max_score: <数>   # cusum: 1 周期の score の上限 (1 回の大きな異常の効き方を決める)
    enabled: <true | false>
    options: {...}    # 総合の方式ごとの指定 (consecutive の suspect_after・confirm_after・corroborate_after)
```

書かなかったキーの既定値は `core/config.py` の `DetectorConfig`。

`allowance` と `confirm_level` の関係: `score` が `allowance + a` の周期が続くと、累積は 1 周期に `weight · a` ずつ増え、`confirm_level / (weight · a)` 周期で確認に達する。1 回で確認させたい判定 (大きな飛び) は `max_score` を `confirm_level + allowance` より大きくする。

## fusion

| `type` | `params` |
| :--- | :--- |
| `cusum` | `suspect_level` (総合の強さがこれ以上で疑い)、`confirm_level` (これ以上で確認)、`blame_margin` |
| `consecutive` | `blame_margin` (疑い・確認の回数はインスタンスの `options`) |

`blame_margin`: 他と共有しない疑い (1 位と 2 位の差) がこれ以上の情報源を `blame` とする。

## state_machine

| キー | 内容 |
| :--- | :--- |
| `recovery_enabled` | `false`: 確認したら `DEGRADED` で知らせるだけ。`true`: 切り離して再初期化する |
| `clear_ticks` | 異常がなくなった周期がこの回数続けば `NORMAL` に戻る |
| `isolate_hold_sec` | 切り離してから再初期化するまで待つ時間 [s] |
| `recover_ok_ticks` | 再初期化の後、収束を確認する回数 |
| `recover_timeout_sec`、`max_reinit_attempts` | 収束しないときの再試行の間隔と回数 |
| `degraded_retry_sec` | `DEGRADED` から再試行する間隔 [s] (`recovery_enabled` のとき) |

## anchors

| キー | 内容 |
| :--- | :--- |
| `pose` | 残す主の姿勢の情報源 |
| `motion` | 残す動きの情報源 (走行距離もこれで測る) |
| `min_interval_sec` | アンカーの間隔の下限 [s] |
| `max_age_sec`、`max_distance_m` | 残す時間と走行距離の上限 |

## profiles

```yaml
profiles:
  <名前>: profiles/<名前>.yaml   # または、その場に辞書で書く
```

プロファイルのファイルは、基の設定と同じキーで、変えるところだけを書く。辞書は再帰的に重ね、リストや値は置き換える。`type` が変わる辞書 (別の総合の方式など) は、重ねずに置き換える。`version` と `profiles` は書けない。
