# 2026-09-23 整理前保全（手順1）

目的は未保存データを失わず、後続の統合・正本確定へ進める状態を作ること。容量削減は目的にしない。
本コミットは保全用であり、FW/CADの採用・安定版認定・main統合を意味しない。
元の13ワークツリーのHEAD/index/ソース、stash、既存PRは維持する。

## ローカル完全コピー

保存先: `/Users/xfa273/workspace/micromouse/nightfall-fw-preservation-20260923-150154`

Git共通ディレクトリと全13ワークツリーのファイルを複製し、82,582通常ファイルを元データとSHA-256照合、3 symlinkのリンク先を照合した。動画・ログ・identity退避・CAD履歴・ignored buildも対象。入れ子のruntime-scaffoldは親buildと分けた復元単位にしている。書込み中で不一致となったファイルはなく、検証エラー0。

`FILES.jsonl` SHA-256: `5f0fb78b199d081f534e9dc6522409b2334adf401702cefd8a93049f3445294a`

未コミット11ファイル＋未追跡193ファイルの計204ファイルをすべて保全。
コード・CAD原本・メカSTL・BOM等131ファイルは以下のGit checkpointにも保存し、生成物・過去コピー等73ファイルは完全コピーに保存した。製造ZIP、検証出力、古いCAD退避も削除していない。

ローカル完全コピーは同一Mac内であり、別媒体への災害対策バックアップではない。GitHubにはソースcheckpointを保存し、動画・ログ・NVM退避・Git設定の全体を公開しない。

## ソースcheckpoint

表のSHAは元ファイル内容を照合したsource tip。currentブランチにはこの後に本資料とWORKLOGだけを追加する。

| 元worktree | 保全branch | source tip | ファイル数 |
|---|---|---|---:|
| main | `backup/20260923-step1-current` | `8aa21656e2f9f30cdc53bb35d3578581aac6c39b` | 112 |
| runtime-scaffold | `backup/20260923-step1-runtime-scaffold` | `1a721bc47a3f1edb2fcfd140e084846af68874ef` | 15 |
| ab88 | `backup/20260923-step1-legacy-f413` | `5eda77929ef15374f49ed099b9018d910a79d775` | 2 |
| acd0 | `backup/20260923-step1-mini-r3-bom` | `48455ffd9489ce3ec9becac9fecf990cabf56cc0` | 1 |
| mini-goal77 | `backup/20260923-step1-legacy-f405-goal` | `fea5449aac14329eb62e8c68de1469d4665c34d5` | 1 |

currentの8コミットは、r3調整、r2加速度＋経路fingerprint、USB復旧、既存診断記録、classic CAD、r3 CAD、r2メカ、旧探索simを分離した。調整値やCAD/READMEの不整合も採否を決めずそのまま保存した。テーブルfingerprintは両profileのスナップショットに対応する。params/tune versionの更新、再調整、生成物の再生成はしていない。

BOMの内容はそのまま保存し、backup branch内の置き場だけを`hardware/mini_r3_0/bom/HM_Nightfall-mini-3a_v0_BOM_20260823.xlsx`とした。元の`outputs/nightfall-mini-3a-bom-2026-08-23/HM_Nightfall-mini-3a_v0_BOM.xlsx`は元worktreeと完全コピーに残る。

## stash・旧履歴

- `backup/20260923-step1-stash-control` → `d691dcbcbf67711d6e3ace581b2f8ea1e2e58455`
- `backup/20260923-step1-stash-params` → `99ec8a9e81e014fda7572b326a430bc440aeb559`
- `backup/20260923-step1-migration-history` → `7f84f59eb9c5b4fec98d6a018ed478e616d732d9`

2 stashはapply/pop/dropせず元の順序で保持。旧移行6コミットのsquash前履歴もtagで固定した。
全refとこれらのtagを含むGit bundleをローカル完全コピーと一緒に保存する。

## 復元検証

Git bundleから独立checkoutを作り、元HEAD＋index/working差分＋未追跡ファイルを復元した。変更がある5 worktreeで、全204ファイルのSHA-256/permissionとGit statusが保存時に一致した。復元試験ディレクトリも保存済み。

`restore.py --worktree <ID> --destination <存在しないパス>`で再実行できる。`--all-files`を指定するとignored動画・ログ・build等もコピーする。入れ子worktreeは`runtime-scaffold`として別に復元する。既存パスへの上書きは拒否する。

Git bundleの検証、元ワークツリーのHEAD/status不変確認、checkpointの内容hash確認を実施。今回は保全だけのためfirmware build、走行試験、CAD DRC/ERCは行わない。ST-LINK/UART/flash/reset/motor/fan/NVM操作は一切ない。

保全branchは後続の採否整理用であり、既存の機能PRへ混入させない。今回は新規の採用PRやマージを行わない。
