# classic r1 時間ベース最短経路・実走成功保存点

2026-10-10、時間ベース経路導出の適用後、ユーザから「上手くいきました」と報告を受け、ここまでを保存した。
保存タグは `stable/classic/unit001/s20261010-classic-r1-time-planner`。元のFWソースは `5cf9f7d0e5e037833df84eb5a3c150fc720eebb7` で、今回の保存ではコード・調整値を変更していない。

## 保存内容

- 探索の吸引ファン duty は `80 / 110`。
- 最短は直進・小回り90°・大回り90°/180°の時間ベース経路導出。斜めは生成しない。
- 走行制御、加速度、旋回値は大会基準版から維持。保存済み迷路を使い、経路なしでは走行前に停止。
- 調整値版は実走時の `t1.0` を維持し、commitとmanifestのハッシュで大会版と区別する。
- ビルド済みBIN/ELF/HEX、build_info、SHA256を作業ツリー外の `../classic-r1-backups/s20261010-classic-r1-time-planner/` に退避した。ソースとmanifestはGitHubに保存し、バイナリはローカルに保持する。

## 確認範囲

実走成功はユーザ報告であり、mode/case、迷路、回数、電源条件は指定されていない。今回、実機ROMとの照合や追加の走行操作は行っていない。
保存したローカルBINは185,488 B、SHA256 `d32887b86e683894baa95f2b68b21b0725a3316127013a427f98658fafee412f`、埋込みSHAは `5cf9f7d`、dirty=0。
先行作業のPC検証・Releaseビルド結果は [準備記録](../../../../docs/CLASSIC_R1_2026_PREPARATION.md) を参照。今回は保存のみのため再ビルドせず、この既存成果物を保持した。

公称時間モデルと旧runnerの速度指令には差があり、実時間最小性の保証はない。2015迷路の既知19条件は経路なしとなる。実機計算時間とスタック最大使用量の測定値は未取得。

校正値は実機Flashに残る。大会ROMの全Flashバックアップは `../classic-r1-backups/20261010-140208/` の一致確認済み2ファイルに保持されている。今回はNVMの再読出し・書込みをしていないため、この保存点は現在の迷路まで含めた実機全体のスナップショットではない。

## ソースから復元

```sh
git worktree add --detach ../nightfall-classic-r1-saved stable/classic/unit001/s20261010-classic-r1-time-planner
cd ../nightfall-classic-r1-saved
cmake --preset Release -DNIGHTFALL_CLASSIC_TIME_PLANNER=ON
cmake --build --preset Release --target nightfall_classic_r1_0
```

`runtime_settings.yaml` はコンパイル時のゴール等を記録した参照情報で、自動適用されない。書込みは従来のUSB-UART手順を使い、校正・迷路領域を消去しない。
