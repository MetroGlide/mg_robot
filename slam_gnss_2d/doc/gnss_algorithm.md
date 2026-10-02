# GNSS 拘束のアルゴリズム

SLAM の座標系 (起動した地点と向きが原点) と、地球の座標系 (UTM) を合わせる処理。
`GraphOrchestrator` (`core/graph_orchestrator.cpp`) と `GnssAnchorManager` (`gnss/anchor_manager.cpp`) が担当する。パラメータは `gnss.*` ([parameters.md](./parameters.md#gnss))。

## 全体の流れ

```
測位 ─► アンカーを決める ─► 初期方位を決める ─► (グラフを UTM の向きに回す) ─► 以降、prior を逐次追加
        (INITIALIZING)        init_distance_m 走る                              (RUNNING)
                                                                  └─► RTK Fix が集まったら、アンカーの位置を計算し直す (動的な再アンカー)
```

## 1. アンカーの設定 (`GnssAnchorManager`)

- 条件 (`gnss.anchor.min_fix_status` 以上の測位) を満たす最初の測位で、その緯度経度と、経度から決めた UTM のゾーンの座標を、アンカー (地図の原点) として記録する。
- 以降の測位は、アンカーとの差を、地図の平面の座標 (`to_local`) にして使う。

## 2. 初期方位の推定 (`initialize_with_gnss_if_ready`)

起動したときの、ロボットの向きと真北の関係は分からない。そこで、アンカーから `gnss.anchor.init_distance_m` (既定 30 m) 以上離れるまで走ったところで、決める。

1. GNSS の軌跡と、SLAM (オドメトリ) の軌跡を、それぞれ主成分分析 (PCA) して、進行方向を求める。
2. 両方の直線性が 85% 以上なら、2 つの進行方向の差を、初期方位の回転 `rot` とする。
3. そうでなければ、最初のノードから現在位置への 2 点のベクトルで、`rot` を求める (ログに WARN が出る)。
4. 最初のノードを、位置 (0, 0)・向き `node0.yaw + rot` に固定する (標準偏差は `anchor.sigma_m`、`anchor.init_yaw_sigma_rad`)。
5. 既存のノードを `rot` だけ回して、オプティマイザに登録し直し、エッジを足す。状態を `RUNNING` にする。

この結果、**地図の X 軸は UTM の東、Y 軸は北に、揃うようになる**。`gnss_transform.yaml` の `rotation_rad` に、この `rot` が記録される ([gnss_transform.md](./gnss_transform.md))。

## 3. prior の追加 (`add_gnss_prior`)

スキャンと同時に測位が来て、次の条件をすべて満たしたとき、ノードの位置の拘束 (prior) を足す。

| 条件 | パラメータ | 内容 |
| :--- | :--- | :--- |
| 新しい測位 | - | 前回と同じタイムスタンプの測位は使わない |
| 測位の種類 | `gnss.prior.min_fix_status` | 0 (すべて)、1 (RTK Float 以上)、2 (RTK Fix のみ) |
| 精度 | `gnss.validation.max_sigma_m` | 標準偏差が、これ以下 |
| 地図と合っている | `gnss.max_innovation_m` | SLAM の推定位置との差が、これ以下 (0 以下で無効) |
| 間隔 | `gnss.min_interval_m` | 前回の prior の位置から、これ以上動いている |

- **標準偏差**: 共分散の `xx` の平方根。共分散が分からないときは、`gnss.sigma.fix_m` (RTK Fix) または `gnss.sigma.float_m` (RTK Float) を使う。NavPVT 入力では、水平精度 (`h_acc`) に `gnss.navpvt_hacc_scale` (1.2) をかけたものが、標準偏差になる。
- **向きの拘束**: GNSS は向きが観測できないので、yaw の分散は `gnss.sigma.factor_yaw_variance` (1e8 rad²) と、非常に大きくする。
- **ロバストカーネル**: `gnss.robust_kernel` (`huber` または `cauchy`)、`gnss.robust_kernel_scale`。
- **レバーアームの補正** (`gnss.lever_arm`): GNSS アンテナは `base_link` から (0.26, -0.13) m ずれている。prior の予測値を、`位置 + R(yaw) × レバーアーム` として、アンテナの位置に対する観測にする。旋回のときは、向きにも拘束がかかる。`0` / `0` で無効。
- 棄却した測位は、理由別に数えられる (標準偏差、間隔、乖離、測位の種類)。

## 4. 動的な再アンカー (`gnss.dynamic_reanchor`)

起動直後の測位が悪くても、あとから RTK Fix が得られたら、アンカーの位置を計算し直す。

1. 標準偏差が 0.5 m 以下で、`dynamic_reanchor.min_fix_status` 以上の測位を、SLAM の位置との組として集める (前のサンプルから 0.5 m 以上動いたとき)。直近 `min_samples + 5` 個、かつ、25 m 以内の範囲に限る。
2. `min_samples` 個以上、かつ、サンプルの最大の広がりが `min_distance_m` 以上になったら、SLAM の位置から UTM の位置への剛体変換 (SVD) を求める。
3. 残差の RMS が `max_residual_rms_m` 以下なら採用し、**アンカーの位置だけ**を更新する (求めた回転は適用しない)。更新は 1 回だけ。
4. 更新したときは、地図を描き直す。

## 5. 一括最適化

オフライン処理の終了時、`optimization.batch_on_finalize` が `true` なら、蓄積した全グラフを Levenberg-Marquardt で一括最適化する (`batch_max_iterations`)。
保存した `pose_graph.json` を、あとから再最適化するのは `reoptimize_node` ([README](../README.md#ポーズグラフの再最適化))。

## 関連

- 評価: `make bag-eval-slam` は、SLAM の出力を GNSS (RTK) と比べる ([tools の README](../../tools/README.md))。
