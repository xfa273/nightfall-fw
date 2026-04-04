# classic unit001 s20260404-nationals

## 概要

全日本マイクロマウス大会（クラシックサイズ）出場時の調整済み構成。
このバージョンを nightfall-fw の最初の安定版とする。

## 構成

- **機体**: classic r1.0 (unit001)
- **FW**: v1.0.0
- **Params/Tune**: t1.0
- **MCU**: STM32F405
- **ビルドターゲット**: `nightfall_classic_r1_0`

## 調整状況

- 探索: 調整済み
- 最短走行: 調整済み（mode1〜mode7）
- センサ校正: 実機Flash上に保存済み（distance_params, flash_params）
- 迷路保存: 実機Flash上（eeprom セクタ11）

## 既知の制約

- 機体識別ブロック未導入
- ログは RAM + UART 方式
- 安定版の自動適用ツールは未整備
