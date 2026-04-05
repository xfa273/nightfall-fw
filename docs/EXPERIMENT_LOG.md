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

