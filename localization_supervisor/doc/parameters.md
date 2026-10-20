# 設定ファイル

情報源・判定のインスタンス・総合・状態遷移・アンカー・プロファイルの組み合わせを書く YAML。ノードもオフライン評価も、このファイルを `core/config.py` で読む。
読み込むときにすべてを検証し、知らないキー・型の違い・存在しない情報源・種類の違いがあれば落ちる。プロファイルもすべて検証する。
最小の例は [config/examples/](../localization_supervisor/config/examples/)。値はロボットのセンサと地図に合わせて決める (オフライン評価で確かめる)。

## 全体

| キー | 内容 |
| :--- | :--- |
| `version` | 約束の版 ([plugins.md](./plugins.md#約束の版)) |
| `period_sec` | 判定と総合の周期 [s] (ノード) |
| `sources` | 情報源 (下記) |
| `timeline` | 時刻のそろえ方 |
| `detectors` | 判定のインスタンス (下記) |
| `enabled_detectors` | 書くと、ここに並べたインスタンスだけを有効にする (主にプロファイルで使う) |
| `fusion` | 総合の方式 `type` と `params` |
| `trust.require_normal` | アンカーを残すのに、正常であることを求めるインスタンス |
| `state_machine` | 状態遷移 (下記) |
| `recovery` | 復旧 (下記) |
| `anchors` | アンカー (下記)。書かなければ残さない |
| `safety` | 安全の信号 (下記)。書かなければ求めない |
| `residuals` | 2 つの姿勢の情報源の残差から求める、分散の倍率 (下記) |
| `grace.startup_sec`、`grace.after_recovery_sec` | 起動直後・復旧直後に総合しない時間 [s] |
| `grace.after_reset_sec` | 外からの再初期化・地図の切り替えの後に総合しない時間 [s] |
| `external_reinit.topics` | 届いたら外からの再初期化として扱うトピック (`geometry_msgs/PoseWithCovarianceStamped`。ノード) |
| `events.prefix` | イベントの名前の前置き (`<prefix>.state_changed`) |
| `events.sink` | イベントの出力のプラグイン `{type, パラメータ...}` ([plugins.md](./plugins.md#イベントの出力-eventsink))。書かなければ `logfmt` |
| `output.status_topic`、`output.diagnostic_name`、`output.hardware_id` | 状態のメッセージのトピック、診断の名前とハードウェア ID (ノード) |
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
    weight: <数>      # cusum: 累積の重み (0 で総合に効かない。記録のためだけに判定する)
    allowance: <数>   # cusum: score のうち、これを超えた分を累積する
    decay: <数>       # cusum: 前の周期の累積に掛ける (1 未満で古い証拠を忘れる)
    max_score: <数>   # cusum: 1 周期の score の上限 (1 回の大きな異常の効き方を決める)
    enabled: <true | false>
    options: {...}    # 総合の方式ごとの指定 (consecutive の suspect_after・confirm_after)
    vouch: [<情報源の名前>, ...]   # 正常と判定できたとき、その情報源が正しいと示す (この判定が疑いをかける情報源のうち)
```

書かなかったキーの既定値は `core/config.py` の `DetectorConfig`。

`vouch`: 2 つの独立の情報源の食い違い (`pose_consistency` など) は、どちらが悪いかを決められない。その片方を `vouch` する別の判定 (例: AMCL の姿勢でスキャンを地図に照合する判定) が正常と判定できているあいだは、`blame` の切り分けで、その片方を疑わず、もう一方だけを疑う ([design.md](./design.md#疑わしい情報源の切り分け-blamecontaminated))。その情報源と独立でない判定 (同じスキャンと地図で推定している推定器に対するスキャンの一致など) には付けない。

`allowance` と `confirm_level` の関係: `score` が `allowance + a` の周期が続くと、累積は 1 周期に `weight · a` ずつ増え、`confirm_level / (weight · a)` 周期で確認に達する。1 回で確認させたい判定 (大きな飛び) は `max_score` を `confirm_level + allowance` より大きくする。

## fusion

| `type` | `params` |
| :--- | :--- |
| `cusum` | `suspect_level` (総合の強さがこれ以上で疑い)、`confirm_level` (これ以上で確認)、`blame_margin` |
| `consecutive` | `blame_margin` (疑い・確認の回数はインスタンスの `options`) |

`blame_margin`: 他と共有しない疑い (1 位と 2 位の差) がこれ以上の情報源を `blame` とする。

`vouch_hold_ticks` (両方式): `vouch` する判定が判定できない周期が続いても、正常と示していると見なす周期の数。数周期に 1 回しか判定できない判定のため。これを超えるか異常になれば、示すのをやめる。

## state_machine

| キー | 内容 |
| :--- | :--- |
| `recovery_enabled` | `false`: 確認したら `DEGRADED` で知らせるだけ。`true`: 切り離して再初期化する (`recovery.generators` と、ノードでは `recovery.resetters` が要る) |
| `request_hold` | `recovery_enabled` のとき、切り離してから戻すまで、走行を止めてほしい (`hold_requested`) と要求するか。`false` なら、切り離しても止めない |
| `clear_ticks` | 異常がなくなった周期がこの回数続けば `NORMAL` に戻る |
| `isolate_hold_sec` | 切り離してから再初期化するまで待つ時間 [s] |
| `recover_ok_ticks` | 再初期化の後、収束を確認する回数 |
| `recover_timeout_sec`、`max_reinit_attempts` | 収束しないときの再試行の間隔と回数 |
| `degraded_retry_sec` | `DEGRADED` から再試行する間隔 [s] (`recovery_enabled` のとき) |
| `isolate_only` | `blame` がこの情報源のときは、切り離す (信頼度を下げる) だけにして、走行を止めず、再初期化もしない。異常がなくなれば戻る (`recovery_enabled` のとき)。情報源の名前のリスト |

## recovery

再初期化が要るあいだ (状態が `RECOVERING`)、`selection.retry_sec` ごとに、候補を作り、評価し、選べたら推定器を初期化し直す ([design.md](./design.md#復旧))。

| キー | 内容 |
| :--- | :--- |
| `convergence` | 復旧の収束の確認に使うインスタンス (すべて正常な周期を数える) |
| `scan_buffer` | 復旧のために残すスキャン `{reference, motion, keyframe_distance_m, keyframe_angle_rad, window_distance_m, max_points_per_scan, max_range_m}`。動きでまとめた局所地図に使う。書かなければ、最新の 1 枚だけ使う |
| `matcher` | 照合 `{type, params}`。書かなければ `correlative` |
| `generators` | 候補を作るもの (下記) |
| `scorers` | 候補を評価するもの (下記) |
| `selection` | 選び方 (下記) |
| `resetters` | 選んだ姿勢の送り先 `[{type, name?, params?}, ...]` ([plugins.md](./plugins.md#再初期化の送り先-poseresetter))。ノードが使う。オフライン評価は使わない |
| `actuators` | 対処 `[{type, name?, params?}, ...]` ([plugins.md](./plugins.md#対処-actuator))。ノードが使う。`name` を書かなければ `type` が名前になる |
| `disabled_actuators` | いま働かせない対処の名前。対処は起動時に作るので、プロファイルで切り替えられるのは、これだけ。無効になった周期に一度だけ、元に戻す (`release`) |

```yaml
recovery:
  generators:
    <名前>:
      type: <登録名 | パッケージ.モジュール:クラス>
      sources: {<引数>: <情報源の名前>, ...}   # 候補を作るものの SOURCES と同じ引数をすべて
      params: {...}
      enabled: <true | false>                  # 書かなければ true。プロファイルで外すときに使う
  scorers:
    <名前>:
      type: ...
      sources: {...}
      params: {...}
      weight: <数>          # 評価の重み付き平均での重み
      veto_below: <数>      # この値を下回った候補は捨てる (書かなければ捨てない)
      enabled: ...
```

| `selection` のキー | 内容 |
| :--- | :--- |
| `min_score` | 最良の候補の評価 (重み付き平均) がこれに届かなければ、再初期化しない |
| `ambiguity_margin` | 最良と、それから `separation_m` 以上離れた次点の差がこれ未満なら、再初期化しない (似た場所の取り違えを避ける) |
| `separation_m` | 互いにこの距離以上離れた候補だけを、別の場所の候補として数える |
| `retry_sec` | 候補を選べなかったとき、次に計画するまでの間隔 [s] |

候補を作るものが 1 つもなければ、`recovery_enabled` は使えない。評価するものが 1 つもない (または、どれも評価できなかった) ときは、最初に作られた候補を選ぶ。
`anchor_motion`・`submap_match`・`anchor_agreement` は `anchors` が、`footprint_free` は `safety` が要る。`anchors.motion` と、候補を作るものの `motion` の情報源は同じにする。

## anchors

| キー | 内容 |
| :--- | :--- |
| `pose` | 残す主の姿勢の情報源 |
| `motion` | 残す動きの情報源 (走行距離もこれで測る) |
| `min_interval_sec` | アンカーの間隔の下限 [s] |
| `max_age_sec`、`max_distance_m` | 残す時間と走行距離の上限 |

## safety

| キー | 内容 |
| :--- | :--- |
| `pose` | footprint を置く姿勢の情報源 |
| `map` | 計画用地図の `reference` の情報源 (地図だけでよい。入力のアダプタ `occupancy_grid`) |
| `footprint` | base の座標の多角形 `[[x, y], ...]` (3 点以上)。経路計画のコストマップと同じ形にする |
| `max_age_sec` | 姿勢の標本の古さの上限 [s] |
| `depth_range_m` | 食い込みの深さを測る範囲 (footprint の外接円から) [m]。これより深いときは、この値になる |

## residuals

自己位置が信頼できる周期に、2 つの姿勢の情報源 a と b の位置の差を残し、a の自己申告の分散より実際の誤差が大きいとき、a の分散にかける倍率を求める ([design.md](./design.md#残差と分散の倍率))。

```yaml
residuals:
  <名前>: {a: <姿勢の情報源>, b: <姿勢の情報源>, ...}
```

| キー | 内容 |
| :--- | :--- |
| `a` / `b` | 倍率をかけたい情報源と、比べる相手 (どちらも `pose`。同じにはできない) |
| `window_sec` | 統計に使う直近の時間 [s] |
| `min_samples` | 倍率を更新するのに要る標本の数 |
| `max_age_sec` | これより古い標本は使わない [s] |
| `extra_std_m` | 時刻のずれなどによる位置の不確かさ [m] (b の側の分散に足す) |
| `scale_delay_sec` | 出した倍率が、a の標本の分散に効き始めるまでの遅れ [s] (受け取る側が処理するまでの時間。標本の分散から倍率を割り戻すときに使う) |
| `gain` | 1 回の更新で、求めた倍率へ近づく割合 |
| `max_scale` | 倍率の上限 |
| `notify_scale` | 倍率がこれ以上の状態を、イベント `residual_changed` で知らせる (これの 0.8 倍を下回ると戻ったとする) |

残差から求めた倍率は `StepResult.residuals` に、受け取る側に出す倍率 (a を切り離しているあいだは上限) は `StepResult.applied_scales` に出る。出力は対処 `variance_scale` が配信し、受け取る側 (例: GNSS のブリッジ) が a の分散にかける ([plugins.md](./plugins.md#対処-actuator))。

## profiles

```yaml
profiles:
  <名前>: profiles/<名前>.yaml   # または、その場に辞書で書く
```

プロファイルのファイルは、基の設定と同じキーで、変えるところだけを書く。辞書は再帰的に重ね、リストや値は置き換える。`type` が変わる辞書 (別の総合の方式など) は、重ねずに置き換える。`version` と `profiles` は書けない。
ノードは、入力のアダプタを起動時に基の設定で作るので、`sources` を変えるプロファイルは起動時に落とす。
