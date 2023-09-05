# auto_cmd_gen

## 作戦

１個めはボーナスじゃない SB にさっさと入れる
２〜10 個目は共通エリア

## ステートの説明

### 初期化（Init）

- 今回は競技開始前に電源を入れる。キャリブレーション後に ROS を起動し、競技開始時点での位置に移動したあと待機。

＜遷移条件＞

- START ボタンで次へ遷移（競技開始）

### ワークへの移動（MoveToWork）

- ワークへ移動する。
- 1 個目は自陣、２〜10 個目は共通、以降は自陣
- 左右十字で一個進む/戻る、上下で共通 ⇔ 自陣

＜共通エリアのみの仕様＞

- 自動では共通エリアの手前までとりあえず行く
- 上キーで共通エリアに侵入、下キーで下がる

＜遷移条件＞

- START ボタンが押されるか、目標位置に到着してから 2 秒経過後

### 下がって掴む（GraspWork）

- ステッパーを下げて、下がったら掴む
  ＜遷移条件＞
- 掴んだあと（多分掴むのに時間がかかるので、掴む司令を出してから約 1 秒後）

### シューティングボックスへ移動（MoveToSB）

- ボーナスエリアが空なら、ボーナスエリアの上空に移動
- ボーナスエリアに 3 こ入っているなら、ボーナスエリアの手前の上空に移動 → その後下がる

＜遷移条件＞

- START ボタンが押されるか、目標位置に到着してから 2 秒経過後

### シュート（Shoot）

- ボーナスエリアが空なら、下がって離す
- ボーナスエリアに 3 個入っているなら、x を小さくして差し込む → 離す

＜遷移条件＞

- 離したあと