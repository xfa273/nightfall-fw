# Classic r1 調整記録

## 2026-10-10: 探索走行の吸引ファン duty 調整を保存

- 保存先: `feature/classic-r1/competition-2026`。
- 基準: `b426b1f` (`v1.0.0` / 大会基準版)。
- ユーザが編集した `params/classic_r1_0/search_run_params_split.c` を保存。
  - `searchRunParams[0].fan_duty`: 100 → 80。
  - `searchRunParams[1].fan_duty`: 150 → 110。
- ユーザ報告: 機体と迷路での走行確認後、吸引ファン duty のみの調整で良さそうだった。走行モード・case の網羅性と実走ログは未確認。
- 他の走行パラメータ・制御コードは変更なし。時間ベース経路導出はこのブランチには未導入。
- 検証: `cmake --build --preset Release --target nightfall_classic_r1_0` 成功（既存ビルドが最新）、`git diff --check` 成功。
- 今回の保存作業では実機への書込み、モータ・ファン操作、NVM 操作なし。
