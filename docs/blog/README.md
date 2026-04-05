# Micromouse Blog on `nightfall-fw`

このディレクトリは、マイクロマウス関連記事を `nightfall-fw` 上でMarkdown管理・公開するための場所です。

## 目的

- はてなブログよりMarkdown運用しやすい環境で記事を書く
- コードと記事を近い場所に置き、読者が実装と記事を行き来しやすくする

## まずやること（最短）

1. 新規記事は `docs/blog/POST_TEMPLATE.md` をコピーして作成する
2. 既存はてな記事は `docs/blog/MIGRATION_TRACKER.md` に追加して優先度を付ける
3. 移行した記事は `docs/blog/posts/` に置く
4. `README.md` の「記事一覧」にリンクを追加する

## ディレクトリ構成

- `docs/blog/README.md`: ブログ運用の入口
- `docs/blog/MIGRATION_PLAN.md`: 移行計画（フェーズ別）
- `docs/blog/MIGRATION_TRACKER.md`: 記事移行の進捗台帳
- `docs/blog/POST_TEMPLATE.md`: 記事テンプレート
- `docs/blog/posts/`: 記事本体（画像付き）

## 記事一覧

- [【既製品で作る】クラシックサイズDCマイクロマウスの足回り（2024-05-09版）](./posts/2024-05-09_classic-dc-footwork/README.md)
- [【改良版】既製品で作るクラシックサイズDCマイクロマウスの足回り](./posts/2026-04-05_classic-dc-footwork-improved/README.md)

## 執筆ルール（現時点）

- 文体は既存試し記事に合わせて「です・ます調」
- 移行記事の公開日は、移行日ではなく移行元の記事公開日を使う
- 画像は記事フォルダ内の `resources/` に置く
- 画像の直後に短いキャプションを `**太字**` で置く
- 見出しは `#` / `##` を基本とする
- コードや設定値はバッククォートで明示する

## 執筆フロー

### あなたが書くとき

1. `docs/blog/POST_TEMPLATE.md` をコピーして新規フォルダを作る
2. 記事本文を書いて、画像を `resources/` に配置する
3. `docs/blog/README.md` の記事一覧にリンクを追加する
4. 必要なら私に校正・見出し整理・図表差し替えを依頼する

### 私が書くとき

1. テーマ（例: センサ校正、機体紹介、大会振り返り）を指定してもらう
2. 私が下書きを `docs/blog/posts/.../README.md` に作成する
3. あなたが内容確認し、必要なら追記・修正する
4. 私が最終整形とリンク更新を実施する

## 公開の考え方

- まずはGitHubリポジトリ上でMarkdownを直接公開（この構成で開始）
- 将来必要ならGitHub Pages化して、同じ `docs/blog/posts/` を公開元にする
