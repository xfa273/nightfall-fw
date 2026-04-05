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
| todo | P1 | https://xfa273-backofchirashi.hatenablog.com/entry/2024/12/26/225736 | - | 2024シーズンの機体紹介 |
| todo | P1 | https://xfa273-backofchirashi.hatenablog.com/entry/2025/12/14/002140 | - | 今年のマイクロマウス振り返り |
| todo | P1 | https://xfa273-backofchirashi.hatenablog.com/entry/2023/12/19/151923 | - | クラシックマウス"Ca.161/bis"の紹介 |

## 運用メモ

- 1記事移行するたびに、状態と新パスを更新する
- 画像は必ず同一記事フォルダ配下の `resources/` に置く
- 内容を大きく改稿した場合はメモ欄に記録する
