# odom→base の TF を、オドメトリの測定の時刻で打刻する

## 背景

- ホイールオドメトリの補正ノードは、`/odom` の header の時刻を、実測した遅れ (`time_offset`) の分だけ過去へずらす ([wheel_odom_corrector](../../mg_drivers/doc/wheel_odom_corrector.md))。
- 一方、odom→base の TF を出す `odometry_tf_broadcaster_node.py` は、`/odom` を受けた時点の `now()` で打刻し直す。そのため TF は、実際より遅れの分だけ新しい時刻を名乗る。
- TF を使う側 (AMCL、コストマップ、自己位置監視の照合) は、スキャンの時刻の姿勢として、遅れの分だけ古い姿勢を使っている。旋回中は向きがずれ、遠くの点ほど投影がずれる (例: 0.8 rad/s で 0.05 s なら約 2.3°、3.5 m 先で約 14 cm)。補正ノードの遅れの補正は、TF を使う側では打ち消されている。
- map→odom の打刻が古いことによる制御の待ちは、EKF の `transform_time_offset` で解いている ([localization.md](../../mg_navigation/doc/localization.md#構成))。この構想は、その対策とは独立した正しさの修正。

## 構想

broadcaster の打刻を `msg.header.stamp` にする。打刻の元を選ぶパラメータ (`now` / `header`) を足し、既定は今の動き (`now`) にして、切り替えて比べられるようにする。

- 長所: TF の時刻とデータの時刻が一致し、スキャンの投影が正しい時刻の姿勢になる。map→odom との打刻の差が縮むので、EKF の `transform_time_offset` を短くできる。
- 短所: 「今の時刻」の odom→base を求めるノードは、その時刻を覆う TF が届くまで、遅れの分だけ長く待つ。該当するのは `collision_monitor` (`base_shift_correction: True` のとき、速度指令を受けるたびに今の時刻の odom→base を待つ)。待つ間は速度指令を流さないので、指令が遅れる (2026-11-28 の実機の記録からの試算で、待ちの中央値が約 0.02 s → 約 0.07 s)。

## 入る場所

- `mg_drivers/scripts/odometry_tf_broadcaster_node.py` のパラメータと、`mg_drivers/launch/bringup_postprocess.launch.py` での指定。
- 影響を受けない側: スキャン・点群の時刻で TF を引くノード (スキャンは受信の時点ですでに遅れの分より古い時刻を名乗るので、新たに待たない。試算で、待つのはスキャンの約 3 % で 1 ms 以下)、時刻を指定しない lookup、EKF (上流の実装では、world フレームが map のとき odom→base を最新の値で引く。着手の前に原文で確かめる)。
- シミュレータでは補正ノードの遅れが入らず、header の時刻と `now()` がほぼ同じなので、振る舞いは変わらない。

## 着手の前に確かめること

- AMCL と AMCL 検証ゲートへの影響: 再生評価 (`tools/scripts/run_localization_variant.sh`) の起動するスタックに broadcaster が入っているので、打刻の元を切り替えて、位置の誤差・AMCL 検証ゲートの閉鎖の回数をオフラインで比べる。今の調整 (AMCL のプリセット、AMCL 検証ゲートのしきい値) は、ずれを含んだ状態で決めている。
- `collision_monitor` の扱い: 指令の遅れを許すか、`base_shift_correction` を `False` (待たずに最新の TF を使う) にするか。多角形を有効にして使うなら、補正をやめるとスキャンの遅れの分だけ点がずれることを考える。
- 実機の記録で、`collision_monitor` を通った後の速度指令の遅れを測る方法。

## やらないこと

- 補正ノードの `time_offset` を 0 にして、TF と `/odom` の時刻を揃える案: EKF が使う `/odom` の時刻が実際より新しくなり、遅れて届く AMCL・GNSS との時刻合わせが崩れる。
