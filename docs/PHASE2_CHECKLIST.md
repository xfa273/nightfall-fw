# Phase 2 チェックリスト（STM32F413 bring-up）

このチェックリストは、`docs/NIGHTFALL_FW_2026_DEVELOPMENT_PLAN.md` の Phase 2 を実施するための運用用メモです。

---

## 1. 現在の進捗

- [x] `nightfall_stm32f413` がビルドできる
- [x] 起動時に `SWO` 出力の初期化を行う
- [x] `trace_printf()` API の土台を導入する
- [ ] 実機で `SWO` 起動ログを受信確認する
- [ ] 最低限のセンサ/モータ初期化方針を文書化して固定する

---

## 2. 運用・品質の定型手順

### 2.1 ビルド確認（毎回）

- `cmake --build --preset Debug-stm32f405`
- `cmake --build --preset Debug-stm32f413`

`main` に入れる前は、公式サポート中の MCU 系列を両方ビルドして壊れ検知を行う。

### 2.2 実験ログ更新（作業単位）

- `docs/EXPERIMENT_LOG.md` に topic / 目的 / 結果 / 参照コミットを追記する。

### 2.3 進捗反映（フェーズ節目）

- `docs/NIGHTFALL_FW_2026_DEVELOPMENT_PLAN.md` の進捗サマリを更新する。

---

## 3. SWO 受信確認手順（実機到着後）

### 3.1 ファーム書き込み

- `cmake --build --preset Debug-stm32f413`
- `./tools/flashing/flash_uart --bin build/Debug/nightfall_stm32f413.bin`

### 3.2 デバッガ接続

- `ST-Link V3 MINIE` で `SWDIO` / `SWCLK` / `SWO` / `NRST` / `GND` を接続する。

### 3.3 期待ログ

起動直後に以下のような文字列が確認できること。

- `[NIGHTFALL] STM32F413 bring-up`
- `FW=<version> TARGET=nightfall_stm32f413 BUILD=<type>`
- `GIT=<sha> DIRTY=<0/1>`

受信不可の場合は、`SWO` 配線、クロック設定、デバッガ側の Trace 設定を順に確認する。
