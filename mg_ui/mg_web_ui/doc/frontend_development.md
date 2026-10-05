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
    layout/     ページの骨格 (NavBar、RobotPageLayout、SideAccordion、SectionCard)
    panels/     計器・ログなどの表示パネル
    status/     コンテナの状態・サービス操作のカード
    sections/   複数の操作をまとめた節 (rosbag の再生など)
    scenario/   シナリオテストのページの部品
    waypoint-actions/  ウェイポイントナビの「Actions」(AMCL・GNSS の入/切、地図の切り替え、任意のサービス・トピック)
    ros-viewer/ three.js による 2D/3D のビューワー (hooks/ と layers/)
    ui/         ボタンなどの汎用の部品
  pages/        ルーティングされるページ
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

`App.tsx` が、次の 4 つの Provider で全体を包む。

| Context | フック | 用途 |
| :--- | :--- | :--- |
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

## ページを足す

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
| `lidarTop` / `lidarFront` | 上 (シアン) / 前 (緑) の LiDAR のスキャン |
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
