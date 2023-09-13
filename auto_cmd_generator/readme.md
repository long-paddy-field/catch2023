# auto_cmd_generator

## ステッパーステートについて

- OwnGiri: 自陣のワークにギリギリぶつからない高さ（把持待機状態）
- OwnCatch: 自陣ワークを取るときの高さ
- CmnGiri: 共通のワークにギリギリぶつからない高さ（把持待機状態）
- CmnCatch: 共通のワークを取るときの高さ
- CmnAbove: 共通のワークを引き抜くのに必要な高さ
- ShtGiri: シューティングボックス似移動するときの壁にギリギリぶつからない高さ
- Shoot: 把持を離すときの高さ
- ShtAbove: シュートしたワークにギリギリぶつからない高さ

**追加が必要かもしれないやつ**

- ShootTouch: 揺れ抑制用に地面にこする高さ

## 状態について

- Init: 初期状態(高さは自陣ギリ)
- MoveToOwnWork: 自陣エリアへ移動（高さは自陣ギリ）
- CatchOwn: 自陣の把持（高さは自陣キャチ）
- MoveToWaypoint: ウェイポイントへ移動（共通エリアの）(高さは共通ギリ)
- MoveToCmnWait: 共通エリア待機状態へ移動(高さは共通ギリ)
- MoveToCmnWork: 共通エリア上空へ移動（高さは共通ギリ）
- CatchCmn: 共通の把持（高さは共通キャチ）
- CmnAbove: ワークの抜き出し(高さは共通抜けられるくらい)
- MoveToShotBox: シューティングボックス前へ移動(高さは共通ギリ)
- MoveToRelease: シュートしに行く（高さは地面）
- MoveToBonus: ボーナスエリアへ行く（高さは地面）
- Release: リリース（高さはワークに当たらない）

## indexの各特殊値
- own_area_index=0: 競技開始時の位置
- cmn_area_index=0: ウェイポイント
- sht_area_index=0: シューティングボックス前

