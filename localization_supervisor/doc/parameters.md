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
| `external_reinit.topics` | 届いたら外からの再初期化として扱うトピック (`geometry_msgs/PoseWithCovarianceStamped`。ノード)。プロファイルで減らせる。AMCL 検証ゲートを使うときは見ない |
| `gate` | AMCL 検証ゲート (下記)。ノードの `use_gate` で有効にする |
| `conduct` | 走行制約 (自己位置状況 × 危険度 → 減速。下記)。AMCL 検証ゲートがあるときだけ使う。書かなければ減速しない |
| `sensor_sets` | センサ構成 (下記)。書かなければ構成を変えない |
| `events.prefix` | イベントの名前の前置き (`<prefix>.state_changed`) |
| `events.sink` | イベントの出力のプラグイン `{type, パラメータ...}` ([plugins.md](./plugins.md#イベントの出力-eventsink))。書かなければ `logfmt` |
| `output.status_topic`、`output.diagnostic_name`、`output.hardware_id` | 状態のメッセージのトピック、診断の名前とハードウェア ID (ノード) |
| `profiles` | プロファイルの名前と、重ねる設定 (ファイルのパスは設定ファイルの場所から。または辞書) |

## sources

```yaml
sources:
  <名前>:
    kind: pose | motion | reference | hypotheses
    derived_from: [<名前>, ...]   # 融合の結果なら、元の情報源 (元は独立の情報源に限る)
    input: {type: <アダプタ>, ...} # 入力のアダプタとその引数 (plugins.md の入力のアダプタ)
    hold: {distance_m: <m>, rotation_rad: <rad>}  # pose だけ。動いたときだけ標本を出す推定器 (下記)
```

`derived_from` を持つ情報源は、疑わしい情報源の特定 (`blame`) に使わず、出力が汚れているか (`contaminated`) の判定に使う ([design.md](./design.md#疑わしい情報源の切り分け-blamecontaminated))。
`reference` と `hypotheses` (推定器の粒子のような、姿勢の仮説の集合) も `blame` の対象にしない。仮説の判定の疑義は、その推定器の姿勢の情報源にかける。

`hold` は、一定の距離か角度を動くまで推定を出さない推定器 (AMCL の `update_min_d` / `update_min_a` など) に書く。最後の標本の後の動き (動きの情報源で積算) が `distance_m` と `rotation_rad` (省略すると制限なし) の両方以下のあいだは、標本が判定の `max_age_sec` より古くても、今の時刻に移したものを使う。止まっているあいだも、その推定器と他の情報源を比べ続けられる (書かないと、止まっているあいだの判定は「分からない」になる)。値は推定器が推定を出す条件に合わせる。止まる直前の 1 回の外れ値も、止まっているあいだ残る (動いたときだけ出す情報源では、受け取る側にも同じだけ残っている)。

## timeline

| キー | 内容 |
| :--- | :--- |
| `motion_priority` | 時刻をそろえるのに使う動きの情報源の、優先の順 (書かなかった動きの情報源は後ろに続く) |
| `horizon_sec` | 標本の履歴を残す時間 [s] |
| `max_extrapolation_sec` | 動きの最新の標本の時刻より、これだけ先の時刻までは最新の値を使う [s] |
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
    vouch: [<情報源の名前>, ...]   # 正常と判定できたとき、その情報源が正しいと示す (この判定が疑義をかける情報源のうち)
```

書かなかったキーの既定値は `core/config.py` の `DetectorConfig`。

`vouch`: 2 つの独立の情報源の食い違い (`pose_consistency` など) は、どちらが悪いかを決められない。その片方を `vouch` する別の判定 (例: AMCL の姿勢でスキャンを地図に照合する判定) が正常と判定できているあいだは、`blame` の切り分けで、その片方を疑わず、もう一方だけを疑う ([design.md](./design.md#疑わしい情報源の切り分け-blamecontaminated))。その情報源と独立でない判定 (同じスキャンと地図で推定している推定器に対するスキャンの一致など) には付けない。

`allowance` と `confirm_level` の関係: `score` が `allowance + a` の周期が続くと、累積は 1 周期に `weight · a` ずつ増え、`confirm_level / (weight · a)` 周期で確認に達する。1 回で確認させたい判定 (大きな飛び) は `max_score` を `confirm_level + allowance` より大きくする。

## fusion

| `type` | `params` |
| :--- | :--- |
| `cusum` | `suspect_level` (総合の強さがこれ以上で疑義)、`confirm_level` (これ以上で確認)、`blame_margin` |
| `consecutive` | `blame_margin` (疑義・確認の回数はインスタンスの `options`) |

`blame_margin`: 他と共有しない疑義 (1 位と 2 位の差) がこれ以上の情報源を `blame` とする。

`vouch_hold_ticks` (両方式): `vouch` する判定が判定できない周期が続いても、正常と示していると見なす周期の数。数周期に 1 回しか判定できない判定のため。これを超えるか異常になれば、示すのをやめる。

## state_machine

| キー | 内容 |
| :--- | :--- |
| `recovery_enabled` | `false`: 異常を確認したら `DEGRADED` で知らせるだけ。`true`: 除外して再初期化する (`recovery.generators` と、ノードでは `recovery.resetters` が要る) |
| `request_hold` | `recovery_enabled` のとき、除外してから戻すまで、走行を止めてほしい (`hold_requested`) と要求するか。`false` なら、除外しても止めない |
| `clear_ticks` | 異常がなくなった周期がこの回数続けば `NORMAL` に戻る |
| `isolate_hold_sec` | 除外してから再初期化するまで待つ時間 [s] |
| `recover_ok_ticks` | 再初期化の後、収束を確認する回数 |
| `recover_timeout_sec`、`max_reinit_attempts` | 収束しないときの再試行の間隔と回数 |
| `degraded_retry_sec` | `DEGRADED` から再試行する間隔 [s] (`recovery_enabled` のとき) |
| `isolate_only` | `blame` がこの情報源のときは、除外する (信頼度を下げる) だけにして、走行を止めず、再初期化もしない。異常がなくなれば戻る (`recovery_enabled` のとき)。情報源の名前のリスト |
| `verify_return` | `true` なら、`RECOVERING` (再初期化の前) と `DEGRADED` から `NORMAL` に戻るのを、`recovery.convergence` の確認 (`recover_ok_ticks` 回) だけにする。異常がなくなった周期 (`clear_ticks`) では戻らない |

## recovery

再初期化が要るあいだ (状態が `RECOVERING`)、`selection.retry_sec` ごとに、候補を作り、評価し、選べたら推定器を初期化し直す ([design.md](./design.md#復旧))。

| キー | 内容 |
| :--- | :--- |
| `convergence` | 復旧の収束の確認に使うインスタンス (すべて正常な周期を数える) |
| `convergence_mode` | `all` (既定) なら `convergence` のすべてが正常な周期、`any` ならどれか 1 つが正常な周期を数える。`any` は、別々の姿勢で確かめる判定を並べ、どちらかで確かめられれば戻すときに使う |
| `scan_buffer` | 復旧のために残すスキャン `{reference, motion, keyframe_distance_m, keyframe_angle_rad, window_distance_m, max_points_per_scan, max_range_m}`。動きでまとめた局所地図に使う。書かなければ、最新の 1 枚だけ使う |
| `matcher` | 照合 `{type, params}`。書かなければ `correlative` |
| `generators` | 候補を作るもの (下記) |
| `scorers` | 候補を評価するもの (下記) |
| `selection` | 選び方 (下記) |
| `observe` | 観測を増やす指示と、撒き直しの条件 (下記)。書かなければ、指示を出さず、撒き直しも制限しない |
| `resetters` | 選んだ姿勢の送り先 `[{type, name?, role?, params?}, ...]` ([plugins.md](./plugins.md#再初期化の送り先-poseresetter))。ノードが使う。オフライン評価は使わない。`role` は `estimator` (推定器。既定) か `fused` (融合)。AMCL 検証ゲートを使うときは両方の役が要る (AMCL 検証ゲートが推定器を採って融合だけを直すときは、`fused` だけに送る) |
| `actuators` | 出力部 `[{type, name?, params?}, ...]` ([plugins.md](./plugins.md#出力部-actuator))。ノードが使う。`name` を書かなければ `type` が名前になる |

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

| `observe` のキー | 内容 |
| :--- | :--- |
| `motion` | 動きの情報源 (最後の再初期化からの走行距離と回転を測る) |
| `request_after_sec` | `RECOVERING` で、再初期化も収束の確認もできない時間がこれだけ続いたら、指示 `observe` を出す。0 なら出さない |
| `request_interval_sec` | 指示 `observe` を出す間隔の下限 [s] |
| `reseed_min_distance_m`、`reseed_min_rotation_rad` | 一度再初期化した後、`motion` の走行距離か回転がこれに届くまで、撒き直さない (`reinit_skipped` の `no_new_observation`)。どちらも 0 なら制限しない。`NORMAL` に戻るまで数え続ける |

候補を作るものが 1 つもなければ、`recovery_enabled` は使えない。評価するものが 1 つもない (または、どれも評価できなかった) ときは、最初に作られた候補を選ぶ。
`anchor_motion`・`submap_match`・`hypothesis_modes`・`anchor_agreement` は `anchors` が、`footprint_free` は `safety` が要る。`anchors.motion` と、候補を作るものの `motion` の情報源は同じにする。

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

残差から求めた倍率は `StepResult.residuals` に、受け取る側に出す倍率 (a を除外しているあいだは上限) は `StepResult.applied_scales` に出る。出力は出力部 `variance_scale` が配信し、受け取る側 (例: GNSS のブリッジ) が a の分散にかける ([plugins.md](./plugins.md#出力部-actuator))。

## gate

AMCL 検証ゲート ([motion_gate.md](./motion_gate.md)、自己位置監視との分担は [design.md](./design.md#AMCL 検証ゲートを入れたとき))。ノードのパラメータ `use_gate` で有効にする (`enabled` は書かない。オフライン評価は使わない)。判定と照合だけにするのはノードのパラメータ `gate_observe_only` (`observe_only` は書かない。このとき自己位置監視の `state_machine.recovery_enabled` も偽になる)。プロファイルでは変えられない。

| キー | 内容 |
| :--- | :--- |
| `period_sec` | AMCL 検証ゲートの周期 [s]。`period_sec` 以下にする |
| `settings` | 下の表 (`core/motion_gate.py` の `GateSettings`) |
| `closers` | 遮断条件 (判定の書き方に、異常が続く時間 `close_after_sec` と、姿勢補正の後の猶予の間も使うか `active_in_grace` を足したもの)。名前は `detectors` のインスタンスと分ける (状態のメッセージに両方を並べる) |
| `map_match` | 照合で決める規則 (`core/map_match.py` の `MapMatchConfig`) |
| `scan_buffer` | 照合に使う、直近のスキャンの重ね方 (`core/scan_buffer.py` の `ScanBufferConfig`)。`max_scan_age_sec` は、最新のスキャンがこれより古い間は照合も停止もせず、溜めたスキャンを捨てる時間 (0 なら見ない)。スキャンの周期の乱れより長く、そのスキャンが欠けて入るセンサ構成の `missing_after_sec` より短くする (読み込みで確かめる)。`recovery.scan_buffer` も同じキーを持つ |
| `initial_search` | 未初期化間に、GNSS の周りを向きの全周で照合する設定 (`core/initial_search.py` の `InitialSearchConfig`)。書かなければしない。`settings.gnss` が要る |
| `manual_pose_topic` | 手動の初期姿勢のトピック (空なら受けない) |

### gate.settings

調整の方針は、イベント ([motion_gate.md](./motion_gate.md#診断とイベント)) の何を見て、どちらへ動かすか。0 や偽で無効にできるキーは、その仕組みを切る手段にもなる ([mg_navigation の localization.md](../../mg_navigation/doc/localization.md#実機で試す))。

| キー | 内容 | 調整の方針 |
| :--- | :--- | :--- |
| `estimator`・`fused`・`motion`・`reference`・`gnss` | AMCL 検証ゲートをかける推定器・融合の結果・動き・地図とスキャン・GNSS の情報源の名前 (`sources` の名前。`gnss` は空なら使わない) | ロボットの構成で決まる。調整しない |
| `max_age_sec`・`gnss_max_age_sec` | 候補に使う標本の古さの上限 [s] | 候補が「標本がない」で落ちるなら、情報源の周期より長くする |
| `gnss_max_std_m`・`gnss_sigma_k` | GNSS を候補に使う申告の精度の上限と、GNSS の候補を探す範囲 (申告の標準偏差の倍) | GNSS の候補が誤った決着を出すなら上限を締める。GNSS が良いのに候補に入らないなら緩める |
| `continuation_xy_per_m`・`continuation_yaw_per_m` | 推測航法姿勢の候補の探す範囲を、確かめてから走った道のり 1 m あたり広げる量 | 推測航法姿勢の候補が照合で範囲の端に張りつくなら上げる (オドメトリの誤差の目安より大きく) |
| `manual_xy_m`・`manual_yaw_rad` | 手動の初期姿勢の候補の探す範囲 | 人が与える姿勢の誤差を覆う。`map_match` の範囲の上限の中にする |
| `retry_sec` | 決まらなかったときに照合し直す間隔 [s] | 照合の負荷が重ければ長くする |
| `confirm_distance_m` | AMCL 検証ゲートが開いている間に、検証済み姿勢を更新する道のり | 推測航法姿勢の候補の範囲が上限に張りつくなら短くする。負荷が重ければ長くする |
| `grace_after_reset_m`・`grace_after_reset_sec` | 姿勢補正の後に遮断条件を使わない道のりと時間 | 姿勢補正の直後に閉じる回が多ければ延ばす。時間の窓で比べる遮断条件の窓より長くする |
| `small_correction_m`・`small_correction_rad` | その場で補正する補正の大きさの上限。超えれば止まってから補正する | 姿勢補正の直後の `cmd_vel` の急な変化が出る大きさより下に置く。止まる回数が多すぎれば上げる |
| `stopped_speed_mps`・`stopped_yaw_rate_rps`・`stopped_sec`・`correction_hold_max_sec` | 止まったとみなす速さと続く時間、止まるのを待つ時間の上限 | 止まるのを待って上限に届く回が多ければ、止まったとみなす速さを緩める |
| `max_closed_sec`・`max_closed_m` | 閉じたまま決まらないときの上限 (時間か道のり)。超えたら止めて照合する | 故障なしで上限に届く回が出ないよう、決まるまでの時間の分布より上に置く。ずれたまま走った距離が長ければ下げる |
| `undecided_hold_sec`・`undecided_drive_m` | 決まらずに止める時間と、解いた後に走らせる道のり | 止まる時間が長すぎれば短くする。止まったままにしないための値で、0 にはしない |
| `reset_std_m`・`reset_std_yaw_rad` | 補正する姿勢の標準偏差 | 姿勢補正の後に AMCL が広がりすぎるなら小さくする |
| `initial_gnss_max_std_m`・`initial_gnss_consecutive` | 初期全周探索に使う GNSS の申告の精度の上限と、続けて満たす標本の数 | 起動の照合が始まらないなら緩める。誤った位置で始まるなら締める |
| `gnss_probation_sec`・`gnss_probation_ratio`・`gnss_probation_window_sec` | 申告の精度が急に良くなった GNSS を、遮断条件と候補に使わない再採用待機。0 なら使わない | 屋根を抜けた直後の誤った決着が残るなら延ばす。GNSS が良いのに候補に入らない時間が長ければ縮める |
| `gnss_hold_after_distrust` | 自己位置監視が除外した GNSS を、除外を解いた後も、AMCL 検証ゲートが検証済み姿勢と合うまで AMCL 検証ゲートと復旧に使わせない (自己位置監視が読む) | 除外が解けてから `localization.gnss_released` までが長すぎるなら偽 |
| `verify_min_interval_sec` | 走行側の検証要求を受け付ける間隔の下限 [s] | 要求が多すぎて止まる時間が増えるなら長くする |
| `wide_search_after_holds` | 決まらずに止めた回数がこれに達した後の停止で、続きの周りを全周で照合する。0 なら使わない | 拡大探索の後の姿勢補正が誤っていれば 0 |

## conduct

走行制約。自己位置状況 (未初期化・疑義・判定保留・見えない・復旧中) と危険度 (誤差見込み ÷ 余裕距離) から、対応表で対処 (`normal` / `slow`) を選び、速度の上限を `StepResult.conduct` に出す (考え方は [architecture_target.md](./architecture_target.md#状態--危険度--対処-未実装))。余裕距離は `safety` の footprint から計画用地図の致死セルまでの距離で、`safety` がなければ分からないものとして重いにする。止める対処は持たない。自己位置監視の復旧が無効 (`recovery_enabled: false`) なら速度の上限は出さない。値は各ロボットの設定ファイルが正。

| キー | 内容 |
| :--- | :--- |
| `error_per_m` | 最後に確かめてから走った道のり 1 m あたりに増やす、誤差見込み |
| `floors` | 自己位置状況ごとの誤差見込みの下限 [m] (`uninitialized`・`suspect`・`undecided`・`unobserved`・`recovering`)。書いた自己位置状況だけ既定に重ねる |
| `unobserved_after_m` | 地図照合で確かめないまま、これだけ走ったら「未観測」 |
| `undecided_after_sec` | AMCL 検証ゲートが閉じてから、これだけ決まらなければ「判定保留」 |
| `medium_ratio`・`heavy_ratio` | 危険度の境 (誤差見込み ÷ 余裕距離)。`0 < medium_ratio <= heavy_ratio` |
| `lower_hold_sec` | 危険度を軽くするのは、軽い側がこの時間続いてから (上げるのはすぐ) |
| `slow_speed_mps` | 減速の速度の上限 [m/s] |
| `table` | 自己位置状況 → 危険度 (`light`・`medium`・`heavy`) → 対処 (`normal`・`slow`)。書いた升目だけ既定に重ね、プロファイルで升目ごとに差し替えられる。正常の自己位置状況は書けない (正常では減速しない) |

- 疑義・見えないから正常へは、AMCL 検証ゲートが地図照合で確かめ (`pose_confirmed`・`gate_opened`)、その後の自己位置監視の新しい判定がどれも異常でないときだけ戻る (地図が現場と食い違うと、照合は誤った姿勢も確かめるので、GNSS などとの食い違いが残る間は戻さない)。

## sensor_sets

センサ構成の並び。どれかの情報源 (`missing`) が `missing_after_sec` 届かなければ、その構成に入る (上から順に最初に当てはまる組)。欠けていた情報源が全部 `present_after_sec` 届き続ければ、全部のセンサ構成 (空の名前) に戻る。構成の変化はイベント `localization.sensor_set_changed` と状態の `sensor_set` に出す。

| キー | 内容 |
| :--- | :--- |
| `name` | 構成の名前 |
| `missing` | 欠けると入る情報源 (姿勢か地図・スキャンの情報源) |
| `missing_after_sec` | 欠けてから構成に入るまでの時間。センサの途絶の見張りが起動し直しを試す時間より長くする |
| `present_after_sec` | 戻ってから全部のセンサ構成に戻るまでの時間 |
| `speed_limit_mps` | 構成の速度の上限 (走行制約の上限と小さい方。0 は上限なし)。自己位置監視の復旧が無効なら出さない |

## profiles

```yaml
profiles:
  <名前>: profiles/<名前>.yaml   # または、その場に辞書で書く
```

プロファイルのファイルは、基本設定と同じキーで、変えるところだけを書く。辞書は再帰的に重ね、リストや値は置き換える。`type` が変わる辞書 (別の総合の方式など) は、重ねずに置き換える。`version`・`profiles`・`gate` は書けない。
使うときは、カンマで区切って複数を指定できる (例: `a,b`)。左から順に重ねるので、同じキーは右のプロファイルの値になる。
ノードは、入力のアダプタを起動時に基本設定で作るので、`sources` を変えるプロファイルは起動時に落とす。
