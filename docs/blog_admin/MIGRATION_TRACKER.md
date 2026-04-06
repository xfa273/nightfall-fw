# マイクロマウス記事移行トラッカー

状態の意味:

- `todo`: 未着手
- `doing`: 移行中
- `review`: 内容確認待ち
- `done`: 移行完了

## 記事一覧

| 状態 | 優先度 | 旧URL | 新パス | メモ |
| --- | --- | --- | --- | --- |
| done | P0 | ローカル試し記事 | `docs/blog/posts/2026-04-05_classic-dc-footwork-improved/README.md` | 画像込みで移設済み |
| done | P1 | https://xfa273-backofchirashi.hatenablog.com/entry/2024/05/09/025442 | `docs/blog/posts/2024-05-09_classic-dc-footwork/README.md` | 公開日は移行元（2024-05-09）で移行 |
| done | P1 | https://xfa273-backofchirashi.hatenablog.com/entry/2024/12/26/225736 | `docs/blog/posts/2024-12-26_2024-season-machines/README.md` | 画像込みで移行済み |
| done | P1 | https://xfa273-backofchirashi.hatenablog.com/entry/2023/12/19/151923 | `docs/blog/posts/2023-12-19_ca-161-bis-introduction/README.md` | 画像込みで移行済み |
| done | P1 | https://xfa273-backofchirashi.hatenablog.com/entry/2025/12/14/002140 | `docs/blog/posts/2025-12-14_2025-micromouse-retrospective/README.md` | 画像込みで移行済み |

## 運用メモ

- 1記事移行するたびに、状態と新パスを更新する
- 画像は必ず同一記事フォルダ配下の `resources/` に置く
- 内容を大きく改稿した場合はメモ欄に記録する
