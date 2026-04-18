---
description: 新規マイクロマウス記事をMarkdownで作成する
---

1. テーマと想定読者を決める（例: 初心者向け、調整ログ、機体紹介）。
2. `docs/blog_admin/POST_TEMPLATE.md` を元に `docs/blog/posts/YYYY-MM-DD_slug/README.md` を作る。
3. 画像を `resources/` に配置し、本文に差し込む。
4. 関連するコードや資料へのリンクを追加する。
5. `docs/blog/README.md` の記事一覧へリンクを追加する。
6. 公開前チェック（誤記、画像リンク、見出し階層）を行う。
7. 記事執筆が重い場合も Cascade が直接対応し、必要に応じて `docs/ai/WORKLOG.md` に記録する。
