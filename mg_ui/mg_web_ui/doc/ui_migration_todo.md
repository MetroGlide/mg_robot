# 新 UI (`/ops`) への移行の ToDo

旧ページ (`/` `/waypoint` `/slam` `/slam-gnss-2d` `/scenario-test` `/system` `/setting`) は、新 UI (`/ops/*`) への移行が終わるまで残す。
このドキュメントは、移行で残っている作業と、旧ページを削除する条件をまとめる。新 UI の構成は [frontend_development.md](./frontend_development.md#新-ui-ops) を参照。

## 現在の対応

| 旧ページ | 新 UI | 状態 |
| :--- | :--- | :--- |
| `/waypoint` | `/ops/waypoint` | 移行済み。走行の操作 (開始・停止・一時停止・再開)、手動のゴールと初期姿勢の指定、状態の表示 |
| `/slam` | `/ops/slam` | 移行済み。SLAM の起動・停止、地図の保存 |
| `/slam-gnss-2d` | (旧ページへのリンク) | 未移行 |
| `/scenario-test` | (旧ページへのリンク) | 未移行 |
| `/system` | `/ops/system` | 旧 `SystemPage` をそのまま表示 (濃色の面に置いている)。新しい配色への置き換えは未実施 |
| `/setting` | 上部バーの「設定」(旧 `SettingModal`) | 旧モーダルをそのまま使う |
| `/` (TOP) | `/ops/waypoint` の KPI と状態の表示で代替 | アラート一覧は `/ops/system` の Diagnostics |
| (旧ビューワー) | `/ops/sensors` | 移行済み。レイヤーとプリセットの設定は旧 UI と共有 |

## 未移行の機能

### 運用ビュー (`/ops/waypoint`)

旧 `/waypoint` にあり、新 UI にまだないもの。
(Actions 相当は移行済み: 左の「操作」カードに、走行 (Jump・Reload WPs・START IMMEDIATE・手動ゴールの BT)、自己位置 (AMCL・GNSS の入/切、AMCL の初期化)、地図の切り替え、サービスの呼び出し、トピックの publish がある。)

- Nav2 のライフサイクル、アクション状態 (旧 `useNav2Status`)。新 UI は、負荷を増やさないため、5 秒ごとのサービス呼び出しをしていない。`/diagnostics` のノードの状態 (`node/*`) で代替している
- 衝突検知のポリゴンごとの状態 (`/collision_detector_state`)
- コンテナ (Navigation) の起動・停止
- シミュレーション用の姿勢リセット、Rosbag 再生
- ジョイスティック、速度ゲージ (旧オーバーレイ)

### SLAM-GNSS-2D と Scenario Test

どちらも、表やフォームが中心で状態が多い (`SlamGnss2DPage.tsx` 586 行、`ScenarioTestPage.tsx` 327 行)。
地図を主役にした運用ビューに合わないので、次のどちらかを決めてから移す。

- 運用ビューに必要な最小限 (SLAM の状態、地図の保存、シナリオの進捗) だけを `OperateLayout` に置き、残りは設定やドロワーにする
- 旧ページの内容を、新しい配色の「作業ビュー」として別の枠で作る

### システム・設定

- `SystemPage` と `SettingModal` の配色をトークンにする (今は濃色を直接書いている)
- 低負荷モードとテーマの切り替えを、設定の画面にも置く (今は上部バー)

## 先送りした項目

- 地図上のクリックでの対象の選択と、詳細カードへの表示 (今の詳細カードはロボットだけ)
- 複数の端末から同時に操作したときの調停 (今は後勝ち。`/pause_request` の `requester_id` が `web_ui` で共通)
- 診断の対象の拡充: GNSS と RealSense の状態。点群などの重いトピックは `mg_diagnostics` で購読しない。`camera_info` のような軽い付随トピックで代用する
- 旧ページのトークン化 (`gray-*` の直書きの置き換え)
- ダークテーマとライトテーマのレイアウトは、foxglove_bridge に接続した状態 (実データ) のスクリーンショットで確認した (1366×768)。実機の走行中の表示の確認は未実施
- 負荷の実測。ロボットが接続されていない環境で作ったので、実データでの確認 (表示の崩れ、更新の頻度)、ブラウザのフレーム時間・CPU・GPU、foxglove-bridge の CPU (`docker stats`) は未実施。同じ rosbag で旧 `/waypoint` と `/ops/waypoint` を比べる
- 低負荷モードで、HUD の更新頻度も下げる (今は影とトランジションを切るだけ)
- タッチパッド操作の実機での確認 (2 本指のスクロール、ピンチ、ドラッグ。ブラウザごとのイベントの違い)
- 診断が一度も届いていない場合の表示 (今は「データなし」と「更新されていません」)

## 旧ページを削除する条件

次を満たしたら、旧ページを削除する。

1. 上の「未移行の機能」のうち、運用で使うものが新 UI に移っている (使わないと判断したものは、理由をここに残して削除する)
2. 実機で新 UI を使った走行と SLAM を行い、旧 UI との差がないことを確認している
3. 負荷を比べて、新 UI が旧 UI 以下であることを確認している ([tuning.md](../../doc/tuning.md))
4. 既定のルート `/` を `/ops` に切り替えている

### 削除の対象

- `frontend/src/pages/`: `TopPage`、`WaypointNavPage`、`SlamPage`、`SlamGnss2DPage`、`ScenarioTestPage`、`SystemPage`、`SettingPage` (`SystemOpsPage` が `SystemPage` を使っている間は、置き換えてから)
- `frontend/src/components/layout/`: `NavBar`、`RobotPageLayout`、`SideAccordion`、`LegacyLayout`
- `App.tsx` の旧ルートと、`useCases.ts` の `LEGACY_LINKS`
- 旧ページだけが使う部品 (`components/panels/` の一部、`components/status/`、`components/waypoint-actions/` など。削除前に新 UI から使われていないことを確認する)
