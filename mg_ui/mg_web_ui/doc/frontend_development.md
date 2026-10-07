# フロントエンドの開発ガイド

`mg_web_ui` のフロントエンド (`frontend/`) に、機能・ページ・ROS のトピックやサービスを足すときのガイド。
技術スタックは React 18、TypeScript、Vite、Tailwind CSS、three.js (`@react-three/fiber`)。

- ROS との通信: foxglove_bridge `ws://localhost:8765` (`ros/foxgloveConnection.ts`)
- system_manager の API: `http://<ホスト>:8001` ([system_manager.md](../../mg_system_manager/doc/system_manager.md))

変更したら、`make ui-lint` と `make ui-test` を実行する。

## 構成と依存の向き

```
frontend/src/
  ros/          通信層 (React に依存しない): foxgloveConnection、codec、topics、services、schemas、namespaces
  hooks/        通信層を React から使うフック (useFoxgloveClient、useTopicSubscriber、useServiceCaller など)
  contexts/     設定・状態の Provider
  components/
    layout/     旧 UI のページの骨格 (LegacyLayout、NavBar、RobotPageLayout、SideAccordion、SectionCard)
    shell/      新 UI (/ops) の枠 (AppShell、TopBar、ConnectionBanner)
    operate/    新 UI の運用ビューの部品 (地図、KPI、進捗、状態、操作、コンテナ、SLAM-GNSS-2D の地図)
    work/       新 UI の作業ビュー (地図を使わず、表とフォームが中心) の枠 (WorkLayout、Panel)
    sensors/    新 UI のセンサビューの部品
    panels/     計器・ログなどの表示パネル
    status/     コンテナの状態・サービス操作のカード
    sections/   複数の操作をまとめた節 (rosbag の再生など)
    scenario/   シナリオテストの部品と、画面の本体 (ScenarioWorkspace)
    waypoint-actions/  ウェイポイントナビの「Actions」(AMCL・GNSS の入/切、地図の切り替え、任意のサービス・トピック)
    ros-viewer/ three.js による 2D/3D のビューワー (hooks/ と layers/)
    ui/         ボタンなどの汎用の部品
  pages/        ルーティングされるページ (旧 UI のページ)
  pages/ops/    新 UI のページ (運用ビューのユースケース、センサ、システム)
  utils/、types/
```

- import の向きは `pages → components / hooks / contexts → ros / utils / types`。**逆向きは禁止**。特に、`ros/` から `hooks/` や `components/` を import しない。
- import はファイルの先頭に書く (遅延 import と、`try/catch` での握りつぶしは禁止。ルートの [AGENTS.md](../../../AGENTS.md))。
- **新しい**再エクスポートだけのファイルは作らない。実体のパスから import する。既存の `ros/interfaces.ts` (`NODE_NS`・`TOPICS`・`SERVICES`・`SCHEMAS` の再エクスポート) と `types.ts` (`types/ros` の再エクスポート) は、既存のコードが使っている。
- 接続は、`App.tsx` で 1 回だけ作り、Props で渡す。`App.tsx` が `useFoxgloveClient()` と `useSystemManagerClient()` を 1 回ずつ呼び、`client` と `sysManager` として各ページに渡す。

```
App.tsx
├── client = useFoxgloveClient()              → FoxgloveClientHandle
├── sysManager = useSystemManagerClient()     → SystemManagerHandle
└── <SomePage client={client} sysManager={sysManager} />
```

### Context

`App.tsx` が、次の 4 つの Provider で全体を包む。`ThemeProvider` だけは `main.tsx` で `App` を包む。

| Context | フック | 用途 |
| :--- | :--- | :--- |
| `ThemeContext` | `useTheme()` | テーマ (light / dark / system) と低負荷モード。**端末ごとに** localStorage へ保存する (他の設定は system_manager 経由で全端末に共有されるが、これは共有しない) |
| `RobotProfileContext` | `useRobotProfile()` | ロボットの名前 (既定 `MG-02`)。画面の見出しとタブのタイトルに使う。system_manager の設定 `robot.name` に保存し、全端末で共有する。設定の「全般」タブで変える |
| `GnssSourceContext` | `useGnssSource()` | KPI の GNSS と詳細が購読するトピック(`navpvt` か `navsatfix`、既定は `navpvt`)。system_manager の設定 `gnss.source` に保存し、全端末で共有する。設定の「全般」タブで変える。GPS のミニ地図と衛星画像は、この設定によらず NavSatFix を使う |
| `SimulationContext` | `useSimulation()` | `isSimulation` で、シミュレーションと実機の表示を切り替える |
| `RosbagReplayContext` | (`RosbagReplayProvider`) | rosbag の再生の状態 |
| `VisualizationContext` | `useVisualization()` | 3D ビューワーのレイヤーとオーバーレイの表示の切り替え |
| `TeleopContext` | `useTeleop()` | テレオペの速度ゲージの設定 |

## ROS 通信のルール

- 購読は `useTopicSubscriber(client, topic, schemaName)` を使う。購読は `FoxgloveConnection` がリスナーの登録として保持し、未接続・再接続・ノードの再起動 (チャネル id の変更) があっても、自動で張り直す。`client.status` で購読を出し分けなくてよい。
- **高頻度のトピック (scan、点群、画像、costmap、tf、odom など) は、表示するときだけコンポーネントをマウントして購読する**。表示していないレイヤーが、購読し続けないようにする。タブが非表示の間は、自動で購読が止まる。
- 連続して publish するトピック (`/cmd_vel` など) は、使う前に `client.advertise(topic, schemaName)` を呼んでおく。呼ばずに初めて publish すると、DDS のマッチングを待つため、最初のメッセージが約 0.4 秒遅れる。
- publish するメッセージの型は、`ros/schemas.ts` にスキーマを足す。cdr でエンコードできないときは例外になる (壊れたデータは送らない)。
- 任意のトピックに 1 回だけ publish するときは、`client.publishOnce(topic, schemaName, data)` を使う (送信後に publisher を片付ける)。型のスキーマが `ros/schemas.ts` になくても、ブリッジが同じ型のトピックを公開していれば、それを使う。どちらにもなければ JSON で送る。publisher は volatile なので、あとから起動した購読者には届かない。
- サービスは、ブリッジが公開しているスキーマで encode / decode するので、`ros/schemas.ts` への追加は要らない。公開中のサービスとトピックの一覧は、`client.listServices()` と `client.listTopics()` で取る (呼んだ時点のスナップショット)。
- 新しいトピックは `ros/topics.ts`、サービスは `ros/services.ts` に定義する。
- ゲートの状態など、ノード側の状態を表示したいときは、変化したときだけ配信する latched (transient_local) のトピックを、ノード側に足して購読する。UI 側で、別のトピックの流れ具合から推定しない (センサの停止とゲートの OFF を区別できないため)。

## 機能を足す

| 足すもの | 場所 |
| :--- | :--- |
| ROS トピックの表示 | `ros/topics.ts` に定義 → `useTopicSubscriber` で購読 → レイヤーまたはパネルとして表示 |
| ビューワーのレイヤー | `components/ros-viewer/layers/` に追加し、`VisualizationContext` の `LayerKey` と `DEFAULT_LAYERS` に登録 |
| system_manager の操作 API | `mg_system_manager/routers/` に追加。入力は `config.py` の正規表現で検証する。テストを書く |
| 操作対象の compose サービス | `mg_system_manager/config.py` の `SERVICES` に追加 (UI の System ページには自動で反映) |
| UI 設定の保存 | `saveSettings(key, value)` (`utils/settingsApi.ts`。キー単位で保存される) |

### トピックを購読する

```ts
// 1. types/ros.ts に受信データの型を足す
export interface MyMessage { value: number; label: string }

// 2. ros/topics.ts にトピック名を足す
export const TOPICS = {
  // ...
  MY_TOPIC: nodeNs(NODE_NS.MY_NODE, '/my_topic'),   // 名前空間あり
  MY_GLOBAL: '/some_global_topic',                    // グローバル
} as const

// 3. コンポーネントで購読する
const data = useTopicSubscriber<MyMessage>(client, TOPICS.MY_TOPIC, 'pkg/msg/MyMessage')
```

- 第 3 引数は `'pkg/msg/MsgType'` の形の文字列。
- 戻り値は `T | null` (未受信のときは `null`)。
- 型は `types/ros.ts` (ROS のメッセージ)、`types/api.ts` (system_manager の API)、`types/scenarioTest.ts` (シナリオテスト)、`types/ros-types.ts`。

### ノードの名前空間

```ts
// ros/namespaces.ts
export const NODE_NS = {
  WAYPOINT_SEQUENCER: 'waypoint_sequencer_node',
  DIAGNOSTICS: '',     // 空文字 = グローバル
  LOCALIZATION: '',
} as const

nodeNs('waypoint_sequencer_node', '/status')   // → '/waypoint_sequencer_node/status'
nodeNs('', '/diagnostics')                     // → '/diagnostics'
```

新しいノードを足すときは、`NODE_NS` に定数を足してから `nodeNs()` を使う。

### サービスを呼ぶ

```ts
// ros/services.ts
export const SERVICES = { MY_SERVICE: nodeNs(NODE_NS.MY_NODE, '/my_service') } as const

// コンポーネント
const { call, loading, error } = useServiceCaller(client)
const result = await call(SERVICES.MY_SERVICE, { key: 'value' })
```

`call()` は、タイムアウト 10000 ms で `Promise<unknown>` を返す。

### トピックに publish する

```ts
// ros/schemas.ts にスキーマを足す (依存する複合型は '===' 区切りで全部展開する。既存の PauseRequest を参照)
'pkg/msg/MyMessage': { encoding: 'cdr', schemaName: 'pkg/msg/MyMessage', schema: 'int32 value\nstring label' }

client.publish(TOPICS.MY_TOPIC, 'pkg/msg/MyMessage', { value: 1, label: 'hello' })
```

## 新 UI (`/ops`)

地図を全面に置き、計器と操作を重ねる新しい UI。旧ページ (`/waypoint` など) とは別のレイアウトで、移行が終わるまで両方を残す
(移行の状況は [ui_migration_todo.md](./ui_migration_todo.md))。

| パス | 内容 | 実装 |
| :--- | :--- | :--- |
| `/ops/:useCase` | ユースケースごとのビュー (`waypoint`、`slam`、`slam-gnss-2d` は地図を使う運用ビュー。`scenario-test` は地図を使わない作業ビュー) | `pages/ops/OperatePage.tsx` が `pages/ops/useCases.ts` から選ぶ |
| `/ops/sensors` | RViz ライクなセンサビュー (全レイヤー、2D/3D) | `pages/ops/SensorsPage.tsx` |
| `/ops/system` | サービスの操作・診断・ログ (作業ビュー) | `pages/ops/SystemOpsPage.tsx` (`SystemPage` を `WorkLayout` に置く) |

`App.tsx` は、旧 UI の枠 (`LegacyLayout`) と新 UI の枠 (`AppShell`) を、別のレイアウトルートにしている。旧ページの URL は変わらない。

### ユースケースを足す

1. `pages/ops/MyOperate.tsx` を作る。`{ client, sysManager }` を受け取り、`OperateLayout` のスロット (`map`、`topLeft`、`toolbar`、`topRight`、`notice`、`bottomLeft`、`bottomRight`) に、`components/operate/` の部品を置く。地図は `OperateMap` (ユースケース固有の描画は `sceneChildren`)。地図を使わず表とフォームが中心なら、`WorkLayout` と `Panel` (`components/work/`) に置く。
   コンテナの操作は `ContainerControl`、ジョイスティックと GPS のミニ地図は `MapOverlays` が使える。
2. `pages/ops/useCases.ts` の `USE_CASES` に 1 行足す (`id` は `sensors`、`system` と重ねない)。上部バーには自動で出る。

操作のロジックは、画面から切り離してフックにする (例: `hooks/useWaypointControl.ts`)。旧ページと新 UI の両方が使える。

### 操作状態の保持

運用の画面は、ルートを切り替える (センサビューへ移って戻る) とコンポーネントが破棄される。**追従・カードの開閉・タブの選択・入力値のように、戻ったときに残っていてほしい状態は、`useState` ではなく `useOpsValue(key, 既定値)` に持つ** (`hooks/useOpsValue.ts`、実体は `utils/opsViewStore.ts`)。

- 値は端末の localStorage にも保存するので、再読み込みでも残る。全端末で共有される `settingsApi` は使わない。
- key は `"<ユースケースや部品>.<項目>"` (例: `waypoint.follow`、`health.view`、`actions.map`)。値は JSON にできるものに限る。
- 地図のカメラ (位置・拡大・向き) は、`OperateMap` の `viewKey` で、画面を離れるときに保存する (メモリのみ。再読み込みでは残さない)。
- 走行に関わる操作の状態 (姿勢・ゴールの指定モードなど) は、誤操作を避けるため保持しない。
- `ui/Disclosure.tsx` は、開閉を `useOpsValue` で保持する節。

### 1 画面での表示

走行中は操作しない運用を想定し、運用ビューは状態の表示を 1 画面 (1366×768 以上) に収める。

- 切り替えて見るもの (センサ・トピック・ノード) は、タブの「全て」で 3 つを並べて見られる (`HealthTabsCard`)。
- 操作のカード (`ActionsCard`) は節ごとに閉じられ、開閉は保持される。閉じておけば、状態の表示だけが残る。
- カードは地図の上に重ね、右端のツールバーを避けて置く (`OperateLayout`)。

### テーマ (色のトークン)

- 色は `index.css` の CSS 変数 (`--surface`、`--text`、`--accent`、`--ok`、`--warn`、`--error` など。値は `R G B`) で定義し、`tailwind.config.js` で `bg-surface`、`text-content`、`text-muted`、`bg-accent`、`text-ok` などの名前にしている。**新 UI の部品は、`gray-*` や色名を直接書かず、このトークンを使う**。
- ライトは `:root`、ダークは `.dark` に値を定義する。`ThemeProvider` が `<html>` に `dark` クラスを付ける。
- 旧ページの枠 (`LegacyLayout`) は、`dark` クラスを付けて、トークンを常に濃色にしている。旧ページの `gray-*` の直書きは、テーマの影響を受けない。
- 新旧で共有する部品 (`components/waypoint-actions/` など) は、トークンで色を書く。旧ページでは `dark` の中なので、従来どおり濃色に見える。
- 地図の配色は `gridColors.ts` のパレット (`map` が旧ビューワー、`mapLight` / `mapDark` が新 UI のライト / ダーク)。

### 負荷を抑えるルール (新 UI の部品)

- **運用ビューでは、高頻度のトピック (scan、costmap、点群、画像) を、既定では購読しない**。そうした表示は、センサビューに置く。運用ビューが購読するのは、地図・経路・TF のほかは、低頻度の状態のトピックだけ。例外は、地図ツールバーの「センサ表示」ボタン (`useSensorsVisible`、既定はオフ。再読み込みでは残さない)。オンにしたときだけ `SensorLayers` が描かれ、そのレイヤー (LiDAR・コストマップ・点群など) の ON/OFF は、センサビューのレイヤーパネルの設定に従う。オフに戻すと購読を止める。画像は運用ビューに出さない。
- GNSS は、設定で選んだ 1 トピック(NavPVT か NavSatFix)だけを `useGnss` で購読する(2Hz に間引く)。KPI の GNSS タイルと、押すと開く詳細カード(緯度経度、UTM、精度、PDOP、高さ)は、同じ購読の値を共有する。測位の判定と UTM 変換は `utils/gnssReading.ts`。
- センサ・トピック・ノードの状態は、`/diagnostics` (1Hz) から `hooks/healthSummary.ts` で分類する。Hz を数えるためにトピックを購読しない。
- 人が読むだけの値 (速度、GNSS など) は `useThrottledTopic(client, topic, schema, maxHz)` で、再描画を間引く。間引いても最後の値は必ず反映される。**E-Stop や FSM の状態のように、変化を見逃せない値は間引かず `useTopicSubscriber` を使う**。
- 値には鮮度を付ける。`useFreshness(client, topic, maxAgeSec)` は、最後の受信 (ブラウザの時計) からの経過秒を返す。古くなった値は `StatTile` が `--` と経過秒の表示にする。Wi-Fi の瞬断で、古い値が現在値に見えないようにするため。
- 切断中は `ConnectionBanner` を出し、送信系のボタンを無効にする (`client.status !== "connected"`)。
- WebGL のキャンバスの上のカードでは、`backdrop-filter` (ぼかし) を使わない。キャンバスが更新されるたびに再合成が走る。半透明の単色と影にする (`components/ui/Card.tsx`)。アニメーションは transform と opacity に限る。
- 地図の上の色は `components/ros-viewer/sceneColors.ts` (テーマ別) にまとめる。LiDAR の点とウェイポイントのピンは画面上の px で大きさを決め、ズームしても小さくならない。縁取りの色で、地図の明暗によらず見えるようにする。
- `OperateMap` は 10fps の描画要求、`dpr=1`。`RenderTicker` の間隔は `intervalMs` で指定できる (`RosViewer` の既定は 50ms)。
- 上部バーの「低負荷」(`ThemeContext.lowLoad`。設定のタブにもある) で、影とトランジションを切る。
- 地図の上に重ねる GPS のミニ地図 (`overlays.gpsMap`) は、地図タイルを読み込み、GNSS を購読するので、既定はオフ。ツールバーのボタンで、使うときだけオンにする。
- Nav2 のライフサイクルは、定期的に呼ばず、ロボットの詳細カードの「確認」を押したときだけ調べる (`useNav2LifecycleCheck`)。衝突検知の状態は 2Hz に間引いて購読する。

## ページを足す

(旧 UI のページ。新しい運用の画面は、上の「新 UI」に足す。)

ロボットの操作系のページは `RobotPageLayout` を使い、左のサイドバー (`SideAccordion`) と右のビューワーを並べる。

```tsx
// pages/MyPage.tsx
import RobotPageLayout from '../components/layout/RobotPageLayout'
import type { AccordionItem } from '../components/layout/SideAccordion'

export default function MyPage({ client, sysManager }: { client: FoxgloveClientHandle; sysManager: SystemManagerHandle }) {
  const accordionItems: AccordionItem[] = [{ id: 'status', label: 'Status', children: <div>...</div> }]
  return <RobotPageLayout client={client} accordionItems={accordionItems} defaultOpen={['status']} viewerMode="2d" />
}
```

手順:

1. `pages/MyPage.tsx` を作る。
2. `App.tsx` に、`lazy` の import と `<Route path="/my-path" element={...} />` を足す。
3. `components/layout/NavBar.tsx` に、ナビのリンクを足す。

### RobotPageLayout の Props

| Prop | 型 | 内容 |
| :--- | :--- | :--- |
| `client` | `FoxgloveClientHandle` | foxglove のクライアント |
| `accordionItems` | `AccordionItem[]` | サイドバーのアコーディオンの項目 (`id`、`label`、`children`) |
| `defaultOpen` | `string[]` | 初期に展開する項目の id |
| `viewerMode` | `'2d' \| '3d' \| undefined` | ビューワーのモード。`undefined` でビューワーなし |
| `interactionMode` | `ViewerInteractionMode` | ビューワーの操作 (姿勢の指定など) |
| `onPoseSet` | `(x, y, yaw) => void` | 姿勢を指定したときの処理 |
| `extraPanels` | `ReactNode` | ビューワーの下に足すパネル |
| `extraOverlay` | `ReactNode` | ビューワーの上に重ねる要素 |
| `viewerOverride` | `ReactNode` | `RosViewer` の代わりに描画する要素 |
| `extraSceneChildren` | `ReactNode` | three.js のシーンに足す要素 (ページ固有のレイヤー) |

## 3D ビューワー (RosViewer)

`viewerMode` を指定すると、右側に `RosViewer` が出る。表示するレイヤーは `VisualizationContext` の `layers` で決まり、Settings で切り替えられる (プリセット: 軽量・標準・すべて)。

### レイヤー (`LayerKey`)

| キー | 内容 |
| :--- | :--- |
| `map` | 占有格子の地図 |
| `globalCostmap` / `localCostmap` | グローバル / ローカルのコストマップ |
| `lidarTop` | LiDAR のスキャン |
| `robotPose` | ロボットの位置 (矢印) |
| `particleCloud` | AMCL のパーティクル |
| `planPath` / `actualPath` | 計画した経路 (赤) / 走った経路 (紫) |
| `waypointMarkers` | ウェイポイントのマーカー |
| `collisionPolygons` | 衝突判定のポリゴン |
| `pointCloud` | 3D 点群 |
| `colorImage` / `depthImage` | カラー / 深度の画像 (ビューワーの下に表示) |

### オーバーレイ (`OverlayKey`)

`joystick` (右下のジョイスティック)、`velocityGauge` (左上の速度ゲージ)、`systemMetrics` (左上のシステムメトリクス)、`gpsStatus`、`gpsMap` (GNSS の状態と地図)。

### レイヤーを足す

1. `contexts/VisualizationContext.tsx` の `LayerKey` と `DEFAULT_LAYERS` に足す。
2. `components/ros-viewer/layers/MyLayer.tsx` を作る。
3. `components/ros-viewer/RosViewer.tsx` の `Scene` の中に `{layers.myLayer && <MyLayer client={client} />}` を足す。

```tsx
export default function MyLayer({ client, tfBuffer }: { client: FoxgloveClientHandle; tfBuffer: TfBuffer }) {
  const data = useTopicSubscriber<MyMsg>(client, TOPICS.MY_TOPIC, 'pkg/msg/MyMsg')
  if (!data) return null
  return <mesh>...</mesh>      // three.js のプリミティブを返す
}
```

- レイヤーは `@react-three/fiber` の `Canvas` の中で描画されるので、戻り値は three.js のプリミティブ (`<mesh>`、`<points>` など)。
- `tfBuffer.lookupTransform(targetFrame, sourceFrame)` で、`THREE.Matrix4 | null` が得られる。
- `components/ros-viewer/hooks/` に、専用のフック (`useLaserScan`、`useOccupancyGrid`、`usePath`、`usePointCloud2`、`useMarkerArray`、`useParticleCloud`、`usePolygonStamped`、`useTfBuffer`) がある。あれば使う。

## 定義の参照先

| ファイル | 役割 |
| :--- | :--- |
| [ros/topics.ts](../frontend/src/ros/topics.ts) | トピック名 |
| [ros/services.ts](../frontend/src/ros/services.ts) | サービス名 |
| [ros/schemas.ts](../frontend/src/ros/schemas.ts) | publish のスキーマ |
| [ros/namespaces.ts](../frontend/src/ros/namespaces.ts) | `NODE_NS` と `nodeNs()` |
| [types/ros.ts](../frontend/src/types/ros.ts) | ROS のメッセージの型 |
| [types/api.ts](../frontend/src/types/api.ts) | system_manager の API の型 |

## 書式と検査

- 整形は Prettier (`frontend/.prettierrc.json`)。**新規・変更したファイルにだけ**適用し、既存のファイルを一括で整形しない。
- ESLint の `react-hooks/exhaustive-deps` は、警告にしている。新しく書くコードでは、警告を出さない。
- `make ui-lint` (型チェック `tsc -b` と ESLint)、`make ui-test` (vitest と、system_manager の pytest)。
