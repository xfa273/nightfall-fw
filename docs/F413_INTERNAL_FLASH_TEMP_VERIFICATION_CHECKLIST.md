# STM32F413（FRAM無し暫定運用）実機確認チェックリスト

このチェックリストは、`STM32F413` 実機完成後に、
`FRAM` を使わず `internal flash` のみで NVM を暫定運用できるかを確認するための手順です。

対象領域（128KiB sector 単位）:

- maze: sector 12 (`0x08100000`)
- distance params: sector 13 (`0x08120000`)
- flash params: sector 14 (`0x08140000`)
- identity: sector 15 (`0x08160000`)

---

## 1. 事前準備

- [ ] `ST-Link V3` と書き込み用UART配線ができている
- [ ] `STM32F413` が bootloader 書き込み可能状態である
- [ ] PCで以下コマンドが実行できる
  - [ ] `cmake --build --preset Debug-stm32f413`
  - [ ] `python3 -m py_compile tools/flashing/flash_uart tools/flashing/make_identity_block.py`

---

## 2. F413ファームを書き込み

- [ ] `cmake --build --preset Debug-stm32f413` を実行
- [ ] 生成された `nightfall_stm32f413.bin` を通常どおり書き込み
- [ ] 起動ログに致命的エラーがないことを確認

補足:

- 通常アプリ書き込みでは `flash_uart --erase app` を使い、NVM保護領域（sector 12-15）を消さない運用にする。

---

## 3. identity 書き込み確認

### 3.1 identity blob を生成

- [ ] 以下を実行（値は機体に合わせて調整）

```bash
python3 tools/flashing/make_identity_block.py \
  --out build/identity/f413_unit_test.bin \
  --family classic \
  --board-id 0x00010000 \
  --hw-rev-major 1 \
  --hw-rev-minor 0 \
  --unit-serial 1 \
  --default-param-profile 0 \
  --capability-flags 0x00000000 \
  --uid0 0x00000000 \
  --uid1 0x00000000 \
  --uid2 0x00000000
```

### 3.2 identity 予約領域へ書き込み

- [ ] 以下を実行

```bash
python3 tools/flashing/flash_uart \
  --bin build/identity/f413_unit_test.bin \
  --base 0x08160000 \
  --allow-protected
```

### 3.3 起動時読込確認

- [ ] 再起動後、起動ログで `ID family=... board=... rev=... unit=...` が出る
- [ ] `SAFE` 遷移しない

---

## 4. NVM保護ポリシー確認（重要）

- [ ] 通常アプリ書き込み時に `--allow-protected` を付けない
- [ ] `flash_uart --erase app` 実行時、保護セクタが `12,13,14,15` と表示される
- [ ] NVM更新が必要なときだけ `--allow-protected` を付ける

---

## 5. distance / sensor / maze の保存読込確認

現行の `nightfall_stm32f413` bring-up ファームでは、UART 1文字コマンドで検証する。

- [ ] 書き込み後の起動ログで以下が表示される
  - [ ] `[NVM-TEST] UART command mode ready`
  - [ ] `[NVM-TEST] commands: ...`
- [ ] シリアルモニタから `a` を送信して save+load を実行し、以下が `PASS` になる
  - [ ] `[NVM-TEST][Distance] save/load_and_apply: PASS`
  - [ ] `[NVM-TEST][Sensor] save/load compare: PASS`
  - [ ] `[NVM-TEST][Maze] save/load compare: PASS`
  - [ ] `[NVM-TEST][Overall] PASS`
- [ ] 再起動後に `A` を送信して load-only を実行し、以下が `PASS` になる
  - [ ] `[NVM-TEST][Distance] load_only: PASS`
  - [ ] `[NVM-TEST][Sensor] load_only: PASS`
  - [ ] `[NVM-TEST][Maze] load_only: PASS`
  - [ ] `[NVM-TEST][Overall][LoadOnly] PASS`

補足:

- `d/s/m` は個別 save+load、`D/S/M` は個別 load-only を実行する。
- この試験はダミー検証値を書き込むため、既存のdistance/sensor/mazeパラメータを上書きする。

- [ ] `distance_save` を1回実行し、再起動後に `distance_load_and_apply` が `true` になることを確認
- [ ] `sensor save` 後に `sensor load` で一致データが読めることを確認
- [ ] `maze save` 後に `maze load` で一致データが読めることを確認

注意:

- `STM32F413` の `nvm_params_distance_load_and_apply` は、blob整合性検証後に `sensor_distance_set_warp_*_3pt` で補正適用まで実装済みです。
- 実機では、再起動後の `distance_load_and_apply == true` と走行挙動の両面で反映を確認してください。

記録テンプレート（試験ログ貼り付け用）:

```text
[F413 FRAM-less NVM Verification]
Date:
Board/Unit:
FW Binary:

[Distance]
- save 実行: pass / fail
- reboot 後 load_and_apply: pass / fail
- 備考:

[Sensor Params]
- save 実行: pass / fail
- load 一致確認: pass / fail
- 備考:

[Maze]
- save 実行: pass / fail
- load 一致確認: pass / fail
- 備考:

[Protection]
- erase app で保護セクタ維持: pass / fail
- 備考:

[Overall]
- 判定: PASS / FAIL
- 課題・次アクション:
```

---

## 6. 暫定運用の判定

以下を満たせば「FRAM無し暫定運用OK」とする。

- [ ] F413で起動・通常動作が安定
- [ ] identity 読込/書込が再現できる
- [ ] distance / sensor / maze の保存読込が再現できる
- [ ] `erase app` でNVM領域が誤消去されない
- [ ] 開発運用上、教育用用途で必要な更新作業が実施可能

---

## 7. 次段（FRAM移行）へ進む条件

- [ ] 上記 1〜5 を満たした
- [ ] 暫定運用中に課題（更新時間・寿命・運用負荷）を記録した
- [ ] `docs/NVM_MIGRATION_PLAN.md` の次段に、FRAM backend追加と教育用FRAM無し切替方針を反映した
