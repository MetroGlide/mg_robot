# プラグインの約束

判定・総合・復旧の候補を作るもの・評価するもの・照合・再初期化の送り先・対処・入力のアダプタ・イベントの出力は、基底クラスを継承したプラグインとして作る。標準のものは登録名で、外部のものは `パッケージ.モジュール:クラス` で、設定ファイルの `type` に書く。
読み込むときに、パラメータの型と必須、情報源の名前と種類をすべて検証し、合わなければ設定の読み込みで落ちる (`ConfigError`)。

## 約束の版

版は `core/interfaces.py` の `CONTRACT_VERSION`。基底クラス (`core/interfaces.py`、`ros/adapters/base.py`、`ros/resetters/base.py`、`ros/actuators/base.py`)・`Snapshot`・設定のスキーマは、版 1 に固定している (mg ではない設定 `config/examples/` のテストが通ることを確かめている)。
プラグインを足すだけなら版は変わらない。リリースの後に既存の約束を変えるときは、`core/interfaces.py` の `CONTRACT_VERSION` と、設定の `version` を上げる (リリースの前は、版を上げずに約束を変えてよい)。設定の `version` が合わなければ、読み込みで落ちる。

## 共通

| 宣言 | 内容 |
| :--- | :--- |
| `PARAMS` | パラメータの名前と `ParamSpec(型, 既定値, 説明)`。既定値が `None` のものは必須。宣言にない名前を設定に書くと誤り。`float` には整数も書ける |

読み込んだパラメータは、既定値を補って `self.params` に入る。インスタンスの名前は `self.name`。

## 判定 (`Detector`)

```python
from localization_supervisor.core.interfaces import Detector, Evidence, ParamSpec, SourceKind


class SpeedLimitCheck(Detector):
    SOURCES = {'pose': SourceKind.POSE}      # 引数の名前と、求める情報源の種類
    SUBJECTS = ('pose',)                     # 異常のとき疑いをかける引数 (既定は reference 以外のすべて)
    PARAMS = {'threshold': ParamSpec(float, None, '...')}

    def evaluate(self, snapshot) -> Evidence:
        sample = snapshot.pose(self.sources['pose'], max_age=2.0)
        if sample is None:
            return Evidence.unknown()
        value = ...
        return Evidence.from_score(value / self.params['threshold'], value=value)
```

| メソッド | 内容 |
| :--- | :--- |
| `evaluate(snapshot)` | 周期ごとに呼ばれる。`Evidence` を返す。判定できなければ `Evidence.unknown()` |
| `after_step(evidence, normal)` | 総合と状態遷移の後に呼ばれる。`normal` はその周期の状態が `NORMAL` か (正常時の基準の学習に使う) |
| `reset()` | 内部の状態を捨てる (地図の切り替え、外からの再初期化) |

`Evidence.score` は 1.0 が異常のしきい値に当たるよう正規化する (`from_score` は 1.0 を超えれば異常とする)。`values` は記録用の値 (オフライン評価の `timeline.jsonl` に出る)。

### Snapshot から読めるもの

| メソッド | 内容 |
| :--- | :--- |
| `pose(name, max_age)` | 最新の姿勢の標本 (時刻は移さない) |
| `pose_aligned(name, max_age, stamp)` | 最新の姿勢の標本を `stamp` (既定は周期の時刻) に移したものと、元の標本の古さ。移せなければ `None` |
| `pose_history(name)` | 姿勢の標本の履歴 |
| `motion_at(name, stamp)` | 動きの情報源の、その時刻の姿勢 (補間) |
| `reference(name)` | 地図 (`occupancy`)・最新のスキャン (`scan`)・距離場のキャッシュ (`fields`) |
| `cached(key, compute)` | 同じ周期の中で計算を使い回す |

### 標準の判定

| 登録名 | 引数 (種類) | 内容 |
| :--- | :--- | :--- |
| `pose_consistency` | `a`, `b` (pose) | 2 つの姿勢を共通の時刻に移した位置の食い違い。`metric` が `mahalanobis` なら両方の共分散を使ったマハラノビス距離の二乗、`distance` なら距離。標準偏差が上限を超える標本は使わない |
| `pose_jump` | `pose`, `motion` (疑うのは `pose`) | 姿勢と動きが示す `map→odom` の、新しい標本ごとの変化 (飛び)。新しい標本がない周期は判定しない |
| `covariance` | `pose` | 自己申告の共分散の大きさ (`trace_xy` / `determinant_xy` / `max_eigenvalue_xy`) |
| `scan_map` | `pose`, `reference` (疑うのは `pose`) | その姿勢でスキャンを地図に重ねた一致率を、正常なときの中央値と比べる。姿勢の周りでずらしたほうが合う量 (利得) も見られる |
| `submap_map` | `pose`, `motion`, `reference` (疑うのは `pose`) | 直近の一定距離のスキャンを、動きで 1 枚の局所地図にまとめ (キーフレームに間引く)、その姿勢で地図に重ねる。一致率と、位置・向きをずらしたほうが合う量 (利得) を見る。照合は `interval_sec` ごとで、間の周期と、走行距離が足りない間は判定しない |

推定器が同じスキャンと地図で推定しているとき、その推定器の姿勢でのスキャンの判定は推定器と独立ではない (誤った場所に収束していても一致率は良く見えやすい)。総合での重みは、この独立性を考えて決める ([design.md](./design.md#判定と-evidence))。
スキャンの判定の値の大きさは、地図とセンサと環境で変わる。正しい姿勢での値の分布を、オフライン評価 (重み 0 で記録だけする) で確かめてから有効にする。

## 総合 (`FusionStrategy`)

`update(evidences, active) -> FusionResult` と `reset()` を実装する。`self.instances` (インスタンスの名前ごとの `InstanceInfo`: `subjects`、`weight`、`allowance`、`decay`、`max_score`、`options`)、`self.independent` (独立の情報源)、`self.derived` (融合の結果と、その元) を使える。
疑わしい情報源の切り分け (最大値の記録、`vouch`、`attribute`) は、`core/fusion.py` の `AttributingFusion` を継承すれば共通で使える。その場合は `_accumulate` (インスタンスごとの累積と、疑い・確認の判定) と `_reset_counts` だけを書く。標準の方式 (`cusum`、`consecutive`) は [design.md](./design.md#総合)。

## 復旧の候補を作るもの (`CandidateGenerator`)

`core/interfaces.py`。再初期化が要るあいだ、周期ごとに `generate(context) -> List[Candidate]` が呼ばれる。作れなければ空を返し、理由を `self.status` に入れる (見送りのイベントに添える)。
`SOURCES` (引数の名前と種類)、`REQUIRES` (設定に要る項目: `anchors`・`safety`) を宣言する。`Candidate` は姿勢 `pose`、共分散 `cov` (3x3)、作ったもの `origin`、評価 `scores`・`total`。
`RecoveryContext` から、`snapshot`、`anchors` (`latest_before(疑いの始まり)`)、`scans` (動きでまとめるスキャンのバッファ)、`safety`、`matcher` を使える。

| 登録名 | 引数 (種類) | 内容 |
| :--- | :--- | :--- |
| `pose_source` | `pose` | 指定した情報源の今の姿勢。位置の標準偏差が上限以下の標本が指定の回数続くまで待つ。向きを持たない測位の向きの不確かさは、パラメータで与える |
| `anchor_motion` | `motion` | 疑いが始まる前のアンカーに、その後の動きを足した姿勢 (推測航法)。動いた距離に比例して共分散を増やす |
| `submap_match` | `motion`, `reference` | 上の姿勢の周りで、スキャン (動きでまとめた局所地図。バッファが足りなければ最新の 1 枚) を地図に照合し、一致の良い互いに離れた姿勢を複数。探す範囲は予測の共分散から決める |

## 復旧の候補を評価するもの (`CandidateScorer`)

`score(candidate, context) -> Optional[float]` (0〜1。大きいほど良い。評価できなければ `None`)。設定で、重みと、下回った候補を捨てる `veto_below` を決める。

| 登録名 | 引数 (種類) | 内容 |
| :--- | :--- | :--- |
| `scan_match` | `reference`, `motion` | 候補の姿勢でスキャンを地図に重ねた一致率 (スキャンのバッファが足りれば局所地図、足りなければ最新の 1 枚) |
| `footprint_free` | `reference` | footprint が地図 (計画用地図) の致死セルにかからなければ 1、かかれば 0。footprint は `safety.footprint` |
| `source_agreement` | `pose` | 指定した姿勢の情報源の今の姿勢との近さ (位置と、重みを掛けた向きの差) |
| `anchor_agreement` | `motion` | 推測航法の予測との近さ。スキャンの照合が決めきれないとき、動きの事前情報で選ぶ |

## 照合 (`Matcher`)

`match(field, points_base, center, window, max_peaks) -> List[MatchPeak]`。点群 (base の座標) が地図の距離場に最もよく合う姿勢を、一致の良い順に、互いに離れたものを最大 `max_peaks` 個返す。探す範囲は `SearchWindow` (位置と向きの範囲と刻み)。
標準は `correlative` (numpy の総当たり)。別の実装 (C++ のマッチャーなど) に差し替えるには、`recovery.matcher.type` に `パッケージ.モジュール:クラス` を書く。

## 再初期化の送り先 (`PoseResetter`)

`ros/resetters/base.py`。`reset(candidate)` で、選んだ姿勢を推定器に送る。コンストラクタは `(名前, パラメータ, ノード, 自分のメッセージの目印)`。
トピックに送るものは、送るときに `own.add(トピック, 時刻の sec, nanosec)` で目印を残す (`TopicResetter` を継承すれば自動)。目印があるメッセージは、外からの再初期化として扱わない。
設定の `recovery.resetters` に `[{type, name?, params?}]` を書く。複数あれば、すべてに送る。

| 登録名 | 内容 |
| :--- | :--- |
| `amcl` | `PoseWithCovarianceStamped` を AMCL の初期姿勢のトピックに送り、少し待ってから `request_nomotion_update` を呼ぶ (止まっていても、初期化し直した姿勢で粒子を更新させる) |
| `robot_localization` | 同じメッセージを robot_localization の `set_pose` に送る |

## 対処 (`Actuator`)

`ros/actuators/base.py`。周期ごとに `apply(result)` が、周期の結果 (`state`・`blame`・`hold_requested`・`directives`) を受け取って、外に働きかける。
状態に応じた働きかけ (ゲート・一時停止) は、毎周期の結果から決めて、変わったときだけ送る (失敗しても次の周期でやり直せる)。設定の `recovery.actuators` に `[{type, name?, params?}]` を書く。
プロファイルが `recovery.disabled_actuators` に名前を挙げた対処は、`apply` が呼ばれなくなり、無効になった周期に一度だけ `release()` が呼ばれる。`release()` では、働きかけたものを元に戻す (止めていた走行を戻す、閉じたゲートを開けるなど)。働きかけを持たない対処は、何もしなくてよい。

| 登録名 | 内容 |
| :--- | :--- |
| `gate` | ゲート (`SetBool` で開閉するサービス。要求元ごとに調停するゲートの調停ノードの窓口) を、`hold_requested` で `blame` が指定のものにあるあいだ閉じ、戻ったら開ける |
| `service_call` | 指示 (`reinit`・`isolate`・`restore`) が出たとき、サービス (`Empty` または `Trigger`) を、指定の遅れの後に呼ぶ |
| `variance_scale` | 監督が求めた、設定の `residuals` の分散の倍率 (`StepResult.applied_scales`。a の実際の誤差が自己申告の何倍か。a を切り離しているあいだは上限) を、周期ごとに `std_msgs/Float64` で配信する。受け取る側が分散にかけ、一定時間届かなければ 1 に戻す |

## 入力のアダプタ (`SourceAdapter`)

`ros/adapters/base.py`。メッセージから標本を作り、`SampleStore` に入れる。ノード (購読) とオフライン評価 (rosbag) の両方で同じものを使う。

| 宣言・メソッド | 内容 |
| :--- | :--- |
| `KINDS` | 作れる情報源の種類 |
| `subscriptions()` | 購読するトピック (`Subscription(topic, msg_type, handler, qos)`)。`qos` は `default` / `sensor` / `latched` |
| `self.context.lookup(parent, child, stamp)` | TF を引く |

| 登録名 | 種類 | 入力 |
| :--- | :--- | :--- |
| `pose_with_covariance` | pose | `geometry_msgs/PoseWithCovarianceStamped` (`topic`) |
| `odometry` | pose / motion | `nav_msgs/Odometry` (`topic`)。pose なら地図座標の姿勢と共分散、motion ならその座標での姿勢 |
| `map_scan` | reference | `nav_msgs/OccupancyGrid` (`map_topic`) と `sensor_msgs/LaserScan` (`scan_topic`)。センサの取り付け位置は `sensor_pose` か、TF (`base_frame` → スキャンの frame) |
| `occupancy_grid` | reference | `nav_msgs/OccupancyGrid` (`topic`) だけ。計画用地図など、安全の信号にだけ使う地図。`occupied_threshold` 以上のセルを致死とする |

## イベントの出力 (`EventSink`)

`ros/event_sinks/base.py`。`emit(name, level, fields)` を実装する (`level` は `info` / `warn` / `error`)。コンストラクタは `(名前, パラメータ, ロガー)`。
設定の `events.sink` に `{type: <登録名 | パッケージ.モジュール:クラス>, <パラメータ>...}` を書く。標準は `logfmt` (ロガーに `evt=<name> key=value ...` の 1 行を出す)。

## 追加の手順

1. 基底クラスを継承し、`PARAMS` と (判定なら) `SOURCES` を宣言する。
2. 設定ファイルの `type` に `パッケージ.モジュール:クラス` を書く。
3. 人工の入力でテストする (例: `test/test_supervisor_core.py`)。故障注入を使ったオフライン評価で、検知率・誤検知率を確かめてから有効にする。
