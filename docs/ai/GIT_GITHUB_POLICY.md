# Git And GitHub Policy For Codex

この文書は、Codex が `nightfall-fw` でドキュメント、AI運用資料、HIL補助ツールを扱うときの Git/GitHub 方針です。

## 1. 基本方針

- 正本として今後も読む資料、実行するツール、開発者が参照するREADMEは Git で管理する。
- 変更前コピーや一時退避バックアップは GitHub に載せない。履歴上のバックアップは commit、branch、PR で担保する。
- 古い Windsurf/Cascade/Devin 互換資料は、現在の正本から明示参照されていない限り背景資料として扱う。
- F405/F413 firmware behavior changes と AI運用/ドキュメント整理は、できるだけ別 commit/PR に分ける。

## 2. ブランチ

- `main` または detached `HEAD` から始める場合は `codex/<topic>` ブランチを作る。
- 大きな実機移植作業や危険な同期前には、必要に応じて `backup/<timestamp>-<reason>` ブランチを作って push する。
- 生成物、ログ、build directory、capture CSV はブランチに含めない。必要な結果は `docs/ai/WORKLOG.md` や専用READMEに要約する。

## 3. コミット

- 1 commit はレビュー可能な1目的に絞る。
- ドキュメントだけの変更では、原則として firmware build は必須にしない。ただし CMake、linker、platform、NVM、tools の実行仕様に触れた場合は近い check を走らせる。
- 実機HILを行った commit では、`docs/ai/WORKLOG.md` に build id、接続手段、コマンド列、モータ許可の有無、結果を書く。
- NVM/identity/calibration/maze/trace format を変える変更は、通常のドキュメント整理 commit に混ぜない。

## 4. Pull Request

- Codex が push する変更は draft PR を標準にする。
- PR本文には、変更理由、主な変更点、検証、実機操作の有無、残リスクを書く。
- ドキュメント移行PRでは、旧資料の扱い、正本化した入口、Git管理外にしたバックアップを明記する。
- firmware/HIL PR では、モータ駆動を行ったか、機体を固定/浮かせたか、取得したログと解析結果を明記する。
- Codex code review を使う場合は、PRコメントで `@codex review` を依頼する。全PRへの自動レビューはリポジトリ内ファイルではなく Codex settings の Code review / Automatic reviews で有効化する。
- Codex review に見せたい恒久的な観点は、トップレベル `AGENTS.md` の `Review guidelines` に置く。

## 5. バックアップとアーカイブ

- Gitで追うべきアーカイブは、将来も読む価値がある資料だけに限定する。
- 変更前退避は `docs/ai/archive/codex-migration-backup-*/` に置けるが、`.gitignore` で Git管理外にする。
- 退避バックアップは通常のAI参照対象でもないため、`.codeiumignore` でも除外する。
- 退避バックアップを恒久化したい場合は、ファイルコピーではなく専用 branch または tag を使う。

## 6. 今回のCodex移行整理

- Codex移行の正本は `AGENTS.md` と `docs/ai/*.md` に置く。
- 安全なF413 HIL入口は `tools/hil/` に置く。
- 旧Windsurf workflowは互換記録として残すが、現行作業の入口にはしない。
