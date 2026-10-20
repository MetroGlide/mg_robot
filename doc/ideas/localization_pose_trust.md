# 自己位置の「信頼できる」を、状態の値として出す

## 背景

走行側の BT の `IsLocalizationHealthy` は、`/localization/status` の `state` が `NORMAL` のときだけ成功する ([bt_plugins.md](../../mg_navigation/doc/bt_plugins.md))。計画用地図の障害物からの脱出 (`FindFreeStart`) は、自己位置が正常なときだけ動く。

困る場面: GNSS だけがずれている間は、監督が GNSS を切り離して `ISOLATED` になる ([mg_localization_supervisor](../../mg_localization_supervisor/README.md))。姿勢は AMCL とオドメトリで信頼できるのに `state` が `NORMAL` でないので、脱出が働かない。

## 構想

状態のメッセージに、「いまの姿勢 (走行側が使う EKF の出力) を信頼してよいか」を表す値を足す。`state` とは別に、監督が決める。

- 切り離しているだけで、姿勢の元になる情報源 (例: AMCL) が疑われていないときは、信頼してよい。
- `DEGRADED`、決めきれない (`unknown`)、または姿勢の元の情報源が疑わしいときは、信頼しない。
- `IsLocalizationHealthy` は、この値を見る。

`state` の意味を変えず、走行側が欲しい問い (姿勢を信じてよいか) に直接答える形になる。

## 入る場所

- メッセージ: `localization_supervisor_msgs/LocalizationStatus` に値を足す。ロボットに依存しない定義にする (姿勢の元になる情報源は設定で宣言する)。
- 判断: `core/status.py` に `StatusView` を作る (状態、`blame`、`contaminated`、`isolate_only` から決める)。
- 利用側: `mg_navigation` の `IsLocalizationHealthy`、Web UI の自己位置のタイル。

## 着手の前に確かめること

- 「信頼してよい」の条件を、`blame` と `contaminated` のどこまで許すか。EKF が汚れている (`contaminated`) ときは信頼しない、で足りるか。
- メッセージの変更は約束の版に関わる ([plugins.md](../../localization_supervisor/doc/plugins.md))。値の追加なら互換を保てるか。

## やらないこと

- `hold_requested` が false の間を「正常」とみなす案: `notify_only` や `gnss_fallback` のように止めない設定でも、自己位置が疑わしいまま脱出が動いてしまうため、外した。
