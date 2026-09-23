# 2026-09-23 独立PRの統合と保留

ユーザ指示により、mini_r3調整中のファーム本体に影響しないPR #16/#20/#21をmainへ統合する。ファーム/共通経路処理に影響するPR #1/#17/#18/#19は、mini_r3調整完了後まで保留する。

## 今回の統合

| PR | 元head | 範囲 |
|---|---|---|
| [#16](https://github.com/xfa273/nightfall-fw/pull/16) | `cfd8dc836f9dc70b60fb80240a6ec27940411ac6` | WeAct adapterのCAD/製造データ/設計記録 |
| [#20](https://github.com/xfa273/nightfall-fw/pull/20) | `9a2fdf9425a28adb7f22dcc5bafcf49d78c19789` | mini_r2 encoder半穴補強・本体取付穴Ø1.6 mmのCAD/製造データ |
| [#21](https://github.com/xfa273/nightfall-fw/pull/21) | `ab377c8b937db2fdbc515d2d55970becd16866be` | 実測データturn simulatorとlocalhost UI。ホスト解析専用 |

独立cloneで3本をmergeし、WORKLOGの追加行同士の競合は全行を保持して解消した。PRの元head/branchは書き換えない。統合前main `066caecfc9e1dbddccbe7e5465547d7de712b809` との全差分は `hardware/`、`tools/tuning/`、文書のみ。`common/app/platform/board/params/nvm/stable`、CMake、linker、`.ioc`、NVM/Log schema、実機操作経路は同一である。

追加互換修正は `tools/tuning/test_turn_dynamics.py` のみ。旧mainの固定omega上限用helperが現mainで撤去済みだったため、その抽出を削除し、実F413 C関数との比較で探索は上限なし/最短はmode指定上限という現行仕様を使う。firmware関数は変更しない。READMEの旧mode4/6=2200という記述も現行mode3〜7=3000へ更新した。実測モデルの適用範囲・安全未認定の扱いは維持する。

## 今回の検証

- tuning既存+動力学39件、dataset/simulator22件、計61件PASS。実F413 C関数をコンパイルするprofile parityを含む。Python構文検査PASS。
- localhost HTTPのHTML/state/simulate/tuneを確認。cross-origin POSTは403、safe recommendationはfalse。実機・外部デバイスへの接続なし。
- 残す通常checkoutをdata-rootにした歴史データinventory: 27走行/8条件、接触1件除外。元CSVのSHA照合PASS。撮影校正等の既存warning6件は残る。
- KiCad 10.0.3 DRC再実行: WeActはerror0/warning26、本体はerror4/warning112、encoder左右は各error6/warning0。全て未配線0。各PRで既知として記録されている結果と一致。
- 製造ZIP4個のCRC、encoder/本体ZIPのSHA-256・source manifestと現PCBの一致・同梱PDFと独立PDFの一致、WeAct ZIPと展開済みCAMファイルの一致を確認。
- `git diff --check` はPDFを除く全統合差分にPASS。PDFにはxref等の仕様上の末尾空白があり、テキストとして整形せずバイト一致を確認。
- firmware/build入力の差分なしをGitで確認したため、前段の両MCU buildを再実行していない。実機/HIL/flash/reset/UART/NVM/motor/fan/走行操作なし。

DRC残存違反や製造条件は設計資料どおり残る。encoderの特殊半穴の製造受付・剥離強度、本体上側穴の端部0.20 mmの実物強度は未確認。マージを製造/走行資格の認定とは扱わない。

## mini_r3調整完了まで保留

| PR | 保存head | 再開時の確認 |
|---|---|---|
| [#1](https://github.com/xfa273/nightfall-fw/pull/1) | `c7a5462cc4b394de3a56c7b8e072680b98e8ee73` | F405 main/mode/run/solverとCMake/paramsを変更する旧CSV UI。現行機体構成と後発実装との重複を比較 |
| [#17](https://github.com/xfa273/nightfall-fw/pull/17) | `c4c910b5c9b0689bb030921883a7b8a5aee44af6` | ホストsimだけでなくcommon/routeにも変更あり。現行route実装との競合を解消 |
| [#18](https://github.com/xfa273/nightfall-fw/pull/18) | `5b26890f2fa64b5799c83654b56b58ba1954fb78` | C探索/trace/ビルド/共通経路処理。現在の探索距離補正・trace保持・機体paramsを維持して検証 |
| [#19](https://github.com/xfa273/nightfall-fw/pull/19) | `6e8db0d8782b47c6b9e94529f010894f2c032721` | 既定OFFのhooksでも実探索ループに差分あり。#18と依存関係を整理して段階検証 |

上記PRはDraft/OPENのまま、head/baseを維持する。ワークツリーを撤去してもbranchとPRは残し、保留作業を破棄しない。再開条件はユーザがmini_r3調整の完了を確認し、統合を指示した時点。現在のmainへ自動的に取り込まない。

## ワークツリー撤去の保全

削除前snapshot: `/Users/xfa273/workspace/micromouse/nightfall-fw-preservation-20260923-170950`。全13ワークツリーをignoredデータごと再保存し、source/snapshotのSHA・権限を確認。前回16:10以降のユーザファイル変更はなかった。全refs bundleを検証し、削除予定12個のHEAD/差分をそれぞれ独立cloneに復元して確認した。

削除対象の6201のPCBエディターは、保存確認なしで正常終了した。終了時のignored表示設定1個と解除されたlock2個も保全記録に含め、CADソースは不変。acd0をcwdに持つ古いCUA runtimeプロセスは終了させない。

[統合PR #43](https://github.com/xfa273/nightfall-fw/pull/43)をmergeしてmainを `da51e1ed4536cee255c0f99068f462810f77bfd0` へ更新し、#16/#20/#21が全てMERGEDになった後、子のruntime-scaffoldから順に12個を `git worktree remove --force` で撤去済み。直前に全対象の全ファイル一覧・source/snapshotのSHA・権限・HEAD/statusを再照合してから削除した。通常checkoutだけが残り、branch/tag・stash2件・保全データは保持。通常checkoutのHEAD/branch/statusと185個の変更ファイルのSHA/権限も保全時と一致した。


### 撤去済み一覧

| 保存ID | 元の場所 | HEAD |
|---|---|---|
| `runtime-scaffold` | `/Users/xfa273/.codex/worktrees/0fb1/nightfall-fw/build/exploration_mcu/latest-check` | `6e8db0d8782b` |
| `0fb1` | `/Users/xfa273/.codex/worktrees/0fb1/nightfall-fw` | `5b26890f2fa6` |
| `165d` | `/Users/xfa273/.codex/worktrees/165d/nightfall-fw` | `4fb45ed1e1bb` |
| `1a6f` | `/Users/xfa273/.codex/worktrees/1a6f/nightfall-fw` | `b55bf0a44143` |
| `51c2` | `/Users/xfa273/.codex/worktrees/51c2/nightfall-fw` | `b55bf0a44143` |
| `6201` | `/Users/xfa273/.codex/worktrees/6201/nightfall-fw` | `9a2fdf9425a2` |
| `6743` | `/Users/xfa273/.codex/worktrees/6743/nightfall-fw` | `ab377c8b937d` |
| `9fe5` | `/Users/xfa273/.codex/worktrees/9fe5/nightfall-fw` | `7f84f59eb9c5` |
| `ab88` | `/Users/xfa273/.codex/worktrees/ab88/nightfall-fw` | `75ab08833b1c` |
| `acd0` | `/Users/xfa273/.codex/worktrees/acd0/nightfall-fw` | `58f60e53d528` |
| `c338` | `/Users/xfa273/.codex/worktrees/c338/nightfall-fw` | `0a1787f26e6c` |
| `mini-goal77` | `/Users/xfa273/workspace/micromouse/nightfall-fw-20260214-mini-goal77` | `0d4d22052f69` |

残存worktreeは `/Users/xfa273/workspace/micromouse/nightfall-fw` の1個。`fix/f413/search-distance-accounting` / `a461d4f` のdirty状態を維持している。main参照は別途同期し、未コミット調整の自動採用やbranch切替は行わない。再開時には `restore.py --worktree <保存ID> --destination <存在しないパス>` で変更を再現でき、`--all-files`でignoredデータも復元する。保留PR #1/#17/#18/#19のhead/baseは不変。古いCUA runtimeがacd0をcwdとして持っていたが、関連プロセスの強制終了は行っていない。

撤去の詳細証跡はローカル監査 `nightfall-fw-audit-20260923/steps3-4/` の `removal-preflight.json`、`removed-worktrees.json`、`removal-result.json`、`COMPLETION.md` に保存。ローカル容量削減や保全データの削除は対象外。
