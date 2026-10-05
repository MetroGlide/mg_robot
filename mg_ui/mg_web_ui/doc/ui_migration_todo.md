# 新 UI (`/ops`) への移行の ToDo

旧ページ (`/` `/waypoint` `/slam` `/slam-gnss-2d` `/scenario-test` `/system` `/setting`) は、実機での確認が終わるまで残す。
機能の移行は終わっている。このドキュメントは、旧ページを削除するまでの作業と条件をまとめる。新 UI の構成は [frontend_development.md](./frontend_development.md#新-ui-ops) を参照。

## 現在の対応

| 旧ページ | 新 UI | 備考 |
| :--- | :--- | :--- |
| `/waypoint` | `/ops/waypoint` | 走行の操作、手動のゴールと初期姿勢、状態の表示、Actions 相当の操作、Navigation コンテナ、シミュレーション・Rosbag 再生 (設定で有効にしたとき)、ジョイスティック、GPS のミニ地図、Nav2 のゴール状態・衝突検知・ライフサイクルの確認 |
| `/slam` | `/ops/slam` | SLAM の起動・停止、地図の保存、シミュレーション・Rosbag 再生 (設定で有効にしたとき) |
| `/slam-gnss-2d` | `/ops/slam-gnss-2d` | ポーズグラフ、最適化前後の経路、衛星画像への重ね表示、地図の保存・プレビュー・再最適化 |
| `/scenario-test` | `/ops/scenario-test` | 作業ビュー。旧ページと同じ本体 (`ScenarioWorkspace`) を使う |
| `/system` | `/ops/system` | 作業ビュー。`SystemPage` を新しい配色にして表示する |
| `/setting` | 上部バーの「設定」 | 設定のモーダルを新しい配色にした。テーマと低負荷モードのタブを追加 |
| `/` (TOP) | `/ops/waypoint` の KPI と状態の表示で代替 | アラート一覧は `/ops/system` の Diagnostics |
| (旧ビューワー) | `/ops/sensors` | レイヤーとプリセットの設定は旧 UI と共有 |

### 旧 UI から変えた点

- Nav2 のライフサイクルは、旧 UI のように 5 秒ごとには呼ばず、ロボットの詳細カードの「確認」を押したときだけ調べる。普段は `/diagnostics` のノードの状態で見る。
- 旧 UI の速度ゲージ、システムメトリクス、GPS の状態の重ね表示は、新 UI の KPI の行に同じ情報があるので移していない。ジョイスティックと GPS のミニ地図は、地図ツールバーのボタンで切り替える (GPS のミニ地図の既定はオフ)。
- 旧 UI では各ページにあった API Log は、システムビューにまとめた。
- 地図の保存は、system_manager の `/map/common/save` を呼ぶ (旧 `SlamPage` が呼んでいた `/map/save` は存在せず、保存に失敗していた)。
- 操作の結果は、`alert` ではなく、画面上のカードに出す (SLAM-GNSS-2D)。

## 旧ページの削除 (実機での確認後に実施する ToDo)

次を満たしたら、旧ページを削除する。

1. 実機で新 UI を使った走行と SLAM を行い、旧 UI との差がないことを確認している
2. 負荷を比べて、新 UI が旧 UI 以下であることを確認している ([tuning.md](../../doc/tuning.md))。同じ rosbag で旧 `/waypoint` と `/ops/waypoint` を比べ、ブラウザのフレーム時間・CPU・GPU、foxglove-bridge の CPU (`docker stats`) を見る
3. 既定のルート `/` を `/ops` に切り替えている

### 削除の対象

- `frontend/src/pages/`: `TopPage`、`WaypointNavPage`、`SlamPage`、`SlamGnss2DPage`、`ScenarioTestPage`、`SettingPage` のうち旧ページ専用の部分 (`SettingPage` と `SystemPage` は新 UI も使うので、旧ルートだけを消す)
- `frontend/src/components/layout/`: `NavBar`、`RobotPageLayout`、`SideAccordion`、`LegacyLayout`
- `App.tsx` の旧ルート
- `hooks/useNav2Status.ts` (旧 `WaypointNavPage` だけが使う。5 秒ごとのサービス呼び出しを含む)
- 旧ページだけが使う部品 (`components/panels/` の一部 (`VelocityGauge`、`SystemMetrics`、`GpsStatusOverlay` など)、`components/status/`、`components/waypoint-actions/` のうち使われなくなったもの)。削除前に新 UI から使われていないことを確認する
- `VisualizationContext` の `OverlayKey` のうち、新 UI が使わないもの (`velocityGauge`、`systemMetrics`、`gpsStatus`) と、設定のモーダルの対応する項目。設定の「Visualization」のレイヤーの切り替えは、センサビューの `LayerPanel` と重なるので、そのとき整理する
- `components/scenario/ScenarioWorkspace` を呼ぶ旧ルート (`ScenarioTestPage`)

## 先送りした項目

- 地図上のクリックでの対象の選択と、詳細カードへの表示 (今の詳細カードはロボットだけ)
- 複数の端末から同時に操作したときの調停 (今は後勝ち。`/pause_request` の `requester_id` が `web_ui` で共通)
- 診断の対象の拡充: GNSS と RealSense の状態。点群などの重いトピックは `mg_diagnostics` で購読しない。`camera_info` のような軽い付随トピックで代用する
- 旧ページのトークン化 (`gray-*` の直書きの置き換え)
- 実機の走行中の表示の確認 (ダーク・ライトのレイアウトは、foxglove_bridge に接続した状態のスクリーンショットで確認済み。1366×768)
- 新しく移した画面 (SLAM-GNSS-2D、Scenario Test、システム、設定、運用ビューの追加の節) の、実データでの表示の確認
- 低負荷モードで、HUD の更新頻度も下げる (今は影とトランジションを切るだけ)
- タッチパッド操作の実機での確認 (2 本指のスクロール、ピンチ、ドラッグ。ブラウザごとのイベントの違い)
- 診断が一度も届いていない場合の表示 (今は「データなし」と「更新されていません」)
- 複数のカードが重なる画面 (Waypoint の左下にジョイスティックと進捗、SLAM-GNSS-2D の地図カードなど) の、1366×768 での重なりの確認
