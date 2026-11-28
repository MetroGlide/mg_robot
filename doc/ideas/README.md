# 構想

着手していない拡張のアイデアを、実装の前に残しておく場所。実装した内容や、今の動作の説明は書かない (それは各パッケージの文書)。
種類と置き場所の規約は [documentation_guide.md](../documentation_guide.md)。

## 書くこと

構想の文書は、次の節を持つ。

- 背景: 何が困るのか、今はどう扱っているか (今の動作は、各パッケージの文書へのリンクで示す)
- 構想: 何をするか。複数あれば案ごとに分けて、長所と短所を書く
- 入る場所: 今の構成のどこに足せるか (既存の拡張点へのリンク)。足したくなったときに、今の設計を壊さないことの確認
- 着手の前に確かめること: 構想が効くかどうかを決める事実と、確かめ方
- やらないこと: 検討して外した案と、その理由

数値は「例」と明記する。設定値や既定値は書かない (規約の「値を書かない」と同じ)。

## 扱い

- 着手したら、設計を正式な文書 (`architecture.md` など) に移し、構想の文書は削除する。構想を古いまま残さない。
- 検討して見送った構想は、理由を「やらないこと」に書いて残す。
- 関係するパッケージの文書からは、「将来の拡張」としてリンクだけ張る (内容は書き写さない)。

## 一覧

| 構想 | 内容 |
| :--- | :--- |
| [gnss_map_distortion.md](./gnss_map_distortion.md) | 地図の歪みによる GNSS の場所ごとのずれを、信頼度 (分散) に反映する |
| [plateau_map_reference.md](./plateau_map_reference.md) | PLATEAU の建物の形を基準にして、地図と GNSS のずれを RTK に依らずに測る |
| [localization_gate_awareness.md](./localization_gate_awareness.md) | AMCL を意図して止めている区間で、自己位置監視が AMCL の判定を自動で外す |
| [localization_single_scan_checks.md](./localization_single_scan_checks.md) | 1 枚のスキャンを使う自己位置の判定 (光線の突き抜けと、少しずつのずれ) を、室内の地図などで試す |
| [localization_graph_aggregator.md](./localization_graph_aggregator.md) | 窓つきの因子グラフで、オドメトリ・GNSS・AMCL の仮説をまとめ、自己位置監視の判定の基準と復旧の候補に使う |
| [localization_directional_constraint.md](./localization_directional_constraint.md) | 照合の拘束の強さを向きごとに見て、通路の進行方向のずれを防ぐ |
| [localization_without_lidar.md](./localization_without_lidar.md) | LiDAR を失った後の自己位置の精度を上げる (GNSS の動きの判定、IMU、カメラ) |
| [localization_exceptions.md](./localization_exceptions.md) | 合格の条件を満たせない状況を「例外」として名前をつけて持つ |
| [odom_tf_stamp.md](./odom_tf_stamp.md) | odom→base の TF を、`now()` ではなくオドメトリの測定の時刻で打刻し、スキャンの投影の時刻のずれをなくす |
| [localization_pose_trust.md](./localization_pose_trust.md) | 自己位置の状態に「姿勢を信頼してよいか」の値を足し、GNSS だけを除外している間も走行側の脱出が働くようにする |
