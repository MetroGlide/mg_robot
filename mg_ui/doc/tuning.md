# 負荷を下げる設定と既知の制約

実機の PC とタブレットの負荷を下げるために入れている設定と、その調整・元に戻す方法。負荷が増える方向の変更は、行っていない。

## 負荷を下げる設定

| 設定 | 効果 | 調整・元に戻す方法 |
| :--- | :--- | :--- |
| タブが非表示の間の購読の停止 | 見ていないタブがトピックを受信しない (bridge の送信量とデコードが減る) | 自動。`FoxgloveConnection.setPaused` |
| ビューワーの描画を要求ベースにする | 常時 60 fps だった描画を、20 fps 程度にする | `RosViewer.tsx` の `RENDER_INTERVAL_MS` (大きいほど軽い) |
| 描画の解像度の上限 | 高 DPI の端末での描画の負荷を抑える | `RosViewer.tsx` の `MAX_DEVICE_PIXEL_RATIO` |
| 可視化のプリセット (軽量・標準・すべて) | 表示するレイヤーを減らして、購読と描画を減らす | Settings の Visualization |
| 画像を canvas に直接描画する | dataURL への再エンコードをやめる | - |
| 地図・コストマップのテクスチャの使い回し | 更新のたびの GPU メモリの確保・解放をなくす | - |
| foxglove_bridge の `capabilities` と `sysinfo` | UI が使わない機能 (connectionGraph、parameters、assets、sysinfo) を止める | `compose.yaml` の `foxglove-bridge` の `capabilities` と `sysinfo` の 2 行を削除する (既定の動作になる) |
| コンテナの状態の取得を軽くする | Docker への問い合わせを減らす (1 秒のキャッシュ、inspect を省略) | `docker_ops.py` の `_STATUS_CACHE_TTL_S` |
| ログの配信 | `docker logs` を共有し、バッファに上限を付けて、まとめて送る | `routers/logs.py` の `MAX_BUFFERED_ENTRIES`、`FLUSH_INTERVAL_S` |
| 新 UI の運用ビュー (`/ops/waypoint`、`/ops/slam`) | scan・costmap・点群・画像を購読せず、地図・経路・TF と低頻度の状態だけを購読する (旧ビューワーの標準構成より、購読するトピックが少ない。実測は [移行の ToDo](../mg_web_ui/doc/ui_migration_todo.md))。描画は 10fps、`dpr=1` | `components/operate/OperateMap.tsx` の `OPERATE_RENDER_INTERVAL_MS`。重い表示はセンサビュー (`/ops/sensors`) に分けている |
| 新 UI の値の更新の間引き | 速度・GNSS は 4Hz / 1Hz に間引いて再描画する (最後の値は必ず反映) | `KpiRow.tsx` の `ODOM_HZ`、`GPS_HZ`。`hooks/useThrottledTopic.ts` |
| 新 UI のセンサ・トピック・ノードの状態 | 高レートのトピックを購読せず、`/diagnostics` (1Hz) だけを使う | `components/operate/HealthTabsCard.tsx` |
| 新 UI の低負荷モード | 影とトランジションを切る (端末ごとに保存) | 上部バーの「低負荷」。`index.css` の `:root.low-load` |
| ページ単位の遅延読み込み | 初期に読み込む JS を 1.5 MB から 0.27 MB にする | `App.tsx` の `lazy` |

## 計測

ブラウザの Performance (メインスレッドの占有率と FPS)、実機 PC の `top` (foxglove_bridge と system_manager の CPU)、`nethogs` や `iftop` (bridge の送信量) を、変更の前後で比べる。

## 既知の制約

- system_manager のテストには `httpx` が必要で、`requirements.txt` に足してある。既存の develop イメージには入っていないので、`make build svc=develop` で再ビルドするまで、`make ui-test` の pytest は失敗する。
- ジョイスティックは、操作中にタブの切り替え・ページの遷移・切断が起きたときに、速度 0 を送る。ただし、ドラッグ中に通信が切れたり、ブラウザが落ちたりしたときは、UI から停止を送れない。`motor_driver_node` に `cmd_vel` のタイムアウトがないので、最後の速度が保持される。
