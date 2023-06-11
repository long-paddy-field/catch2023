# catch2023

## 各ノードについて

### launcher_catch

- 全体の起動 launcher
- 起動時に必要なパラメータを param_catch.yaml で管理。
- リトライ時に備えて、セーブしたいデータを保持＆シャットダウン時に保存

### joy_controller_catch

- joy コントローラの入力を指令値に変換

### rviz_monitor_catch

- シミュレータモードとモニターモードが存在
- シミュレータモードでは指令値を rviz に反映
- モニターモードではセンサーデータを rviz に反映

### gui_catch

### device_catch

- マイコンとの通信の環境
