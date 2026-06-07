---
description: はてなブログ記事をdocs/blogへ1本移行する
---

1. 対象記事URLを決め、`docs/blog_admin/MIGRATION_TRACKER.md` の状態を `doing` に変更する。
2. `docs/blog/posts/YYYY-MM-DD_slug/` を作成する。
3. 記事本文を `README.md` に移す。
4. 画像を `resources/` に保存し、本文リンクを相対パスで修正する。
5. `docs/blog/README.md` の記事一覧にリンクを追加する。
6. 体裁チェック（見出し、画像リンク、誤記）を行う。
7. `docs/blog_admin/MIGRATION_TRACKER.md` を `review` か `done` に更新する。
8. 必要なら重い改稿作業も Codex が直接実施し、必要に応じて `docs/ai/WORKLOG.md` に記録する。
