# 実験ログ（EXPERIMENT LOG）

ここには、`exp/*` ブランチで行った実験の概要と結果を残します。

目的:
- 何を試したか
- どのブランチ/タグ/コミットに対応するか
- 失敗した場合の理由（次回の再挑戦のため）

---

## テンプレ

- topic:
- 目的:
- ブランチ:
- 参照（タグ or コミット）:
- 結果: success / abandoned
- メモ:

---

## 記録

- topic: phase2/trace-api-bootstrap
- 目的: F413基板到着前に、F405(UART)とF413(SWO)を同じ呼び出しで扱える `trace_printf()` の土台を作る
- ブランチ: main
- 参照（タグ or コミット）: 1c35f4b
- 結果: success
- メモ: `platform/trace/trace.c` を追加し、`trace_init()` と `trace_printf()` を導入。Phase2チェックリストとSWO受信確認手順も文書化。

- topic: phase3/nvm-foundation-bootstrap
- 目的: 機体識別ブロック導入前に、`NVM` APIと識別ブロック検証ロジックの土台を先行実装する
- ブランチ: main
- 参照（タグ or コミット）: 377318d
- 結果: success
- メモ: `nvm/nvm*.{h,c}` を追加し、`docs/NVM_IDENTITY_BLOCK_SPEC.md` と `docs/PHASE3_CHECKLIST.md` を作成。

- topic: phase3/f413-identity-safe-mode-bootstrap
- 目的: F413起動初期で識別ブロックを読み、整合性異常時にセーフモードへ遷移する最小フローを追加する
- ブランチ: main
- 参照（タグ or コミット）: 984e157
- 結果: success
- メモ: `platform/stm32f413/.../main.c` で `HAL_Init()` 直後に `nvm_identity_read()` を実行し、`INTEGRITY_ERROR/HW_ERROR` をセーフモード扱いにした。

- topic: phase3/f405-identity-safe-mode-and-f413-identity-address
- 目的: F405にも起動初期の識別ブロック判定を導入し、F413の識別ブロック予約領域アドレスを確定する
- ブランチ: main
- 参照（タグ or コミット）: pending (この作業コミットで確定)
- 結果: success
- メモ: `platform/stm32f405/.../main.c` に `HAL_Init()` 直後の識別読出しと safe mode 判定を追加し、`nvm/nvm.c` と仕様書で F413 識別領域を `0x08160000`（sector 15, 128KiB）に確定した。

- topic: phase3/nvm-migration-plan
- 目的: `distance_params` / `flash_params` / `eeprom` のNVM抽象化移行計画を確定する
- ブランチ: main
- 参照（タグ or コミット）: pending (この作業コミットで確定)
- 結果: success
- メモ: `docs/NVM_MIGRATION_PLAN.md` を追加し、読み出し経路集約→形式統一→backend分離の段階移行計画を定義。

- topic: phase3/nvm-stepA-distance-maze-wrap
- 目的: Step Aとして、distance/maze呼び出しの `nvm` ラッパ経由化を開始する
- ブランチ: main
- 参照（タグ or コミット）: working tree
- 結果: success
- メモ: `nvm/nvm_params.{h,c}` を追加し、`main.c` / `test_mode.c` / `search.c` / `sensor.c` の distance/maze/sensor params 呼び出しを `nvm_params_*` / `nvm_maze_*` へ移行。

- topic: phase3/nvm-stepB-maze-blob-format
- 目的: maze保存形式を `magic/version/length/crc` 付きへ統一し、旧形式互換を維持する
- ブランチ: main
- 参照（タグ or コミット）: working tree
- 結果: success
- メモ: `nvm_maze_save_map()` / `nvm_maze_load_map()` を実装し、`search.c` の maze 保存/読込を一括APIへ移行。新形式検証（length/checksum）と旧形式（raw halfword）互換読込、未初期化（全`0xFFFF`）時のゼロ初期化を追加。

- topic: phase3/nvm-stepC-backend-dispatch-minimal
- 目的: `nvm_read/write/erase` をbackend分離方針へ進め、F405で最小write/eraseを有効化する
- ブランチ: main
- 参照（タグ or コミット）: working tree
- 結果: success
- メモ: `nvm.c` に backend分岐を追加し、`STM32F405` の internal flash backend で `nvm_erase()` / `nvm_write()` を実装。`nvm_write()` は境界チェックと `0->1` 禁止チェックを持つ。続いて `STM32F413` でも `NVM_AREA_IDENTITY` の internal flash backend `nvm_erase()` / `nvm_write()` を最小実装した（他areaは未対応）。

- topic: phase3/nvm-identity-write-path
- 目的: 識別ブロックの更新経路を `nvm_identity` API内へ追加する
- ブランチ: main
- 参照（タグ or コミット）: working tree
- 結果: success
- メモ: `nvm_identity_write()` を追加し、入力ブロックから `magic/schema/length/crc` を再構築して `nvm_erase(NVM_AREA_IDENTITY)` → `nvm_write(NVM_AREA_IDENTITY)` で保存する実装を導入。

- topic: phase3/identity-block-host-generator
- 目的: 識別ブロックをホスト側で生成し、予約領域へ書き込める運用経路を追加する
- ブランチ: main
- 参照（タグ or コミット）: working tree
- 結果: success
- メモ: `tools/flashing/make_identity_block.py` を追加。`nvm_identity_block_t` 互換の68byteバイナリを生成し、`magic/schema/length/crc` を自動構成。`python3 -m py_compile` とサンプル生成実行で確認済み。

- topic: phase3/identity-block-operation-and-mode9-rewrite
- 目的: 識別ブロック書き込み運用手順を確定し、実機側診断入口を追加する
- ブランチ: main
- 参照（タグ or コミット）: working tree
- 結果: success
- メモ: `docs/NVM_IDENTITY_BLOCK_OPERATION.md` を追加し、`--allow-protected` 前提の書き込み手順を確定。`tools/flashing/flash_uart` に `--allow-protected` を追加。`platform/stm32f405/Core/Src/test_mode.c` の Mode9 に sub=1（identity rewrite: read->UID反映->`nvm_identity_write`）を追加。

- topic: phase3/f413-temporary-internal-flash-nvm
- 目的: F413でFRAM無し暫定運用（教育用想定）に向けてNVM areaを内蔵Flashで運用可能にする
- ブランチ: main
- 参照（タグ or コミット）: working tree
- 結果: success
- メモ: `nvm.c` で `STM32F413` の `NVM_AREA_DISTANCE_PARAMS` / `NVM_AREA_FLASH_PARAMS` / `NVM_AREA_MAZE_MAP` を area table と sector対応へ追加し、`nvm_write`/`nvm_erase` を有効化。`nvm_params.c` に F413向け `distance_save` / `distance_load_and_apply(整合性検証 + sensor_distance warp適用)` / `sensor save/load/defaults` / `maze save/load` の `nvm` API実装を追加。`flash_uart` の F413保護セクタを 12-15 に拡張。

- topic: phase3/f413-framless-checklist
- 目的: F413 FRAM無し暫定運用の実機確認手順を標準化する
- ブランチ: main
- 参照（タグ or コミット）: working tree
- 結果: success
- メモ: `docs/F413_INTERNAL_FLASH_TEMP_VERIFICATION_CHECKLIST.md` を追加し、ビルド/書き込み/identity検証/保護ポリシー確認/次段移行条件をチェックリスト化。関連ドキュメントから参照リンクを追加。

- topic: phase3/f413-framless-checklist-step2-4-device-check
- 目的: F413実機で `erase app` 時の保護セクタ維持と identity 読込継続を確認する
- ブランチ: main
- 参照（タグ or コミット）: working tree
- 結果: success
- メモ: `python3 tools/flashing/flash_uart --bin build/Debug/nightfall_stm32f413.bin --base 0x08000000 --erase app --timeout 45` 実行で `Device ID: 0x0463` / `Flash profile: stm32f413` / `Protected sectors: [12, 13, 14, 15]` を確認。再起動ログで `ID family=1 board=131072 rev=2.0 unit=1` を確認し、SAFE遷移なし。

- topic: phase3/f413-framless-checklist-step5-device-check
- 目的: F413実機で `distance/sensor/maze` の保存読込と再起動後読込（load-only）を確認する
- ブランチ: main
- 参照（タグ or コミット）: working tree
- 結果: success
- メモ: UARTコマンド検証で `a` 実行時に `[Distance] save/load_and_apply: PASS` / `[Sensor] save/load compare: PASS` / `[Maze] save/load compare: PASS` / `[Overall] PASS` を確認。再起動後 `A` 実行で `[Distance] load_only: PASS` / `[Sensor] load_only: PASS` / `[Maze] load_only: PASS` / `[Overall][LoadOnly] PASS` を確認。FRAM無し暫定運用のチェックリスト項目5を満たした。

- topic: phase4/f413-fram-backend-enable
- 目的: F413の `distance/sensor/maze/trace` を内蔵Flash暫定運用から外付けFRAM backendへ切り替える
- ブランチ: main
- 参照（タグ or コミット）: working tree
- 結果: success
- メモ: `nvm.c` のbackend分岐を更新し、`NVM_AREA_IDENTITY` は内蔵Flash維持、`NVM_AREA_DISTANCE_PARAMS` / `NVM_AREA_FLASH_PARAMS` / `NVM_AREA_MAZE_MAP` / `NVM_AREA_TRACE_LOG` を外付けFRAMへディスパッチ。FRAM領域を1MB（CY15B108想定）で割当。`cmake --build --preset Debug-stm32f413` 成功、`flash_uart --erase app` で `nightfall_stm32f413.bin`（49456 bytes）を書き込み済み。

- topic: phase4/f413-fram-distance-save-fix
- 目的: FRAM backend切替後にdistance saveのみ失敗する不具合を修正する
- ブランチ: main
- 参照（タグ or コミット）: working tree
- 結果: success
- メモ: 原因は `nvm_get_area_info()` が `base_address==0` を未対応扱いしていたこと。FRAM先頭オフセット `0x00000000`（distance area）を有効化するよう判定を修正。`cmake --build --preset Debug-stm32f413` 成功、再書き込み済み（49464 bytes）。

- topic: phase4/f413-fram-backend-device-verify-all-areas
- 目的: F413実機でFRAM backendの `distance/sensor/maze/trace` 保存読込と再起動後load-onlyを確定する
- ブランチ: main
- 参照（タグ or コミット）: working tree
- 結果: success
- メモ: UARTコマンド検証で `a` 実行時に `[Distance] save/load_and_apply: PASS` / `[Sensor] save/load compare: PASS` / `[Maze] save/load compare: PASS` / `[Trace] save/load compare: PASS` / `[Overall] PASS` を確認。再起動後 `A` 実行で `[Distance] load_only: PASS` / `[Sensor] load_only: PASS` / `[Maze] load_only: PASS` / `[Trace] load_only: PASS` / `[Overall][LoadOnly] PASS` を確認。個別確認として `t` / `T` でも `[Trace] ... PASS` を再確認。ログ上のビルド識別は `GIT=7f02576 DIRTY=1`。

---

## F413 FRAM-less 実機記録テンプレート（貼り付け例）

```text
[F413 FRAM-less NVM Verification]
Date: 2026-04-xx
Board/Unit: STM32F413 pre-order / unit-xx
FW Binary: build/Debug/nightfall_stm32f413.bin

[Distance]
- save 実行: pass
- reboot 後 load_and_apply: pass
- 備考: 3回連続再起動で再現

[Sensor Params]
- save 実行: pass
- load 一致確認: pass
- 備考: 全フィールド比較一致

[Maze]
- save 実行: pass
- load 一致確認: pass
- 備考: 任意セル更新後に一致確認

[Protection]
- erase app で保護セクタ維持: pass
- 備考: sector 12/13/14/15 が erase 対象外であることをログで確認

[Overall]
- 判定: PASS
- 課題・次アクション: FRAM backend移行時の比較試験を追加予定
```

