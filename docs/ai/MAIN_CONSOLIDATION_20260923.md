# 2026-09-23 開発本流のmain統合

## 範囲と基準

ユーザ指定の手順2（開発本流をmainへ統合する）に対応する。mainを今後の開発基準線へ戻すため、旧main `b55bf0a441435c1fd4b1d7d8ea0f63692f2b46ed` から現在のコミット済み本流 `a461d4f96447c74d897f820079c95d59ffb1e3bf` までの290コミットを、履歴を保持するmerge commitで取り込む。

- PRのなかった `codex/f413-search-port-complete` (`06b22180455c64d80874d14fbcce11117d454a79`) まで261コミット、続くPR #22〜#41に29コミット。全20PRのheadは本流tipの祖先である。
- 既存差分は333ファイル、87,741追加/1,935削除。積み上がったplatform/board/params/nvm/common/tools/docs/stableの履歴をまとめて採用する統合作業であり、通常の新規開発PRの分割単位とは異なる。中間版を順次mainに載せず、最終状態を検証する。
- 統合時の追加変更は、吸引モード再構成 `d395d13` に追従していなかった `tools/tuning/test_turn_limits.py` の期待値修正と、この統合記録のみ。mini_r3 mode3〜7の既存3000 deg/sを正としてテストを修正した。firmware/paramsの値は追加変更していない。
- MCU/機体: F405 `mini_r1_0` / `classic_r1_0`、F413 `mini_r2_0` / `mini_r3_0`。classic新機体のF413対応は未実装のまま。
- `.ioc`変更なし。既存履歴にmini_r2/r3のparams・LUT・gain・吸引モード・runtime profile対応を含む。F405の機体別paramsは旧mainと同一。
- NVM identity/layout/trace schemaは旧mainから変更なし（trace v6）。F413の古い距離校正の適用拒否条件、既存traceの再利用と複数走行保持は動作変更を含む。
- `stable/mini/unit002/s20260921-mini-r3-2s-fan-off/` のmanifest/notes/runtime settings追加を含む。main統合を新しい安定版の認定とは扱わない。

## 手順1に追加した保全

前回保存後に変わったのは `params/mini_r3_0/shortest_run_params_split.c` の7項目。mode3の小回りalpha 64000→48000/出口8→3、大90 alpha24500→20500/出口13→3.5、大180 alpha18500→18000/出口8→3、mode4小回りalpha118000→100000を追加保存した。

- GitHub branch: `backup/20260923-step1-additional-tuning`。
- 調整ソースcommit: `cceae900906f4121855fa4809630210b6c905688`、保全記録込みtip: `1d20e5e1eb8f5dab2d6e5814f8bb4d83b37c258f`。前回保全branchのtip `94b29c8871558d1ad2f9e34847f891149136762c` に接続済み。
- 全13ワークツリー、ignoredデータ、Gitメタデータを `/Users/xfa273/workspace/micromouse/nightfall-fw-preservation-20260923-161036` に追加保存し、SHA-256と権限を照合。前回15:01の保全も保持。
- bundle検証と通常作業ツリーの185変更ファイルの独立復元試験（HEAD/status/内容/権限）に合格。`README.md` と `restore.py` に復元方法を記録。
- ローカル全量保全は同じMac内。ソース/CAD/BOMのcheckpointはGitHubへpush済み。未コミットの調整、USB復旧、CAD、別ワークツリーの差分は保全branchに置き、今回のmain統合には含めない。

## 検証

通常作業ツリーのdirty値や既存buildを使わず、独立cloneのコミット済み本流で実行。実機コマンド列は **なし**。UART/ST-LINK/flash/reset/motor/fan/NVM操作を実施していない。

| 対象 | 結果 |
|---|---|
| `cmake --preset Debug`、`Debug-stm32f413`、`Debug-stm32f405` | PASS。F405はmini/classic両targetをbuild |
| F413 params coverage | PASS: 2 profiles、160 scalars、実52 translation units |
| `tools/hil/run_f413_{machine,motor_pwm,nvm_guard,nvm_params,stop_approach,search_distance,suction}_tests.sh` と `run_f413_runtime_goal_tests.py` | 8入口PASS。mockによるhost検証。制御/探索の近傍試験はASan/UBSanを含む |
| shared path/legacy codec/validator、route/planner/clearance、slalom、F413 path/params/mode2/4/6/preview、solver CLI | `tools/solver_host/run_*_tests.sh` の16入口PASS |
| `tools/route_precompute/run_tests.sh` | PASS: generated table一致、14,210 checks/0 failures |
| `tools/solver_host/run_solver_host.sh --explore-sim --max-steps 512` | PASS: open16x16、3 stepsでgoal到達 |
| F405 sensor differential | 旧mainと本流のソースを同じhost compilerで比較。mini/classic両params、標準LUT/差替LUT/warpの3状態、9 API・各65,536入力、3,538,944結果がfloat bit単位で一致（front_sumは第2入力を変化させた組合せ） |
| Python `tools/flashing` | 17 tests PASS（identity等はfixtureのみ） |
| Python `tools/logging` | 17 tests中14 PASS/3 fixture待ち。続く `tools/logging/test_f413_trace_retention.sh` でC ASan/UBSanおよび7 Python testsがPASS、上記3件も実行済み |
| Python `tools/tuning` | 32 tests PASS。旧期待値1件を前述の通り修正後再実行 |
| Python `tools/vision/tests` | 既存 `.venv-vision` で138 tests PASS。Wi-Fi/撮影制御はfake/local serverによる試験 |
| 差分 | `git diff --check` PASS（統合時の追加変更） |

F413 RAM 274,264 B/320 KiB、Flash 377,060 B/1 MiB。F405 mini RAM 120,376 B/128 KiB、classic RAM 84,152 B/128 KiB、両方CCMRAM 60,024 B/64 KiB。F405 miniのRAM/CCMRAM余裕は小さい。F405のpath変換差分はlegacy codec/path pipeline試験で検証し、距離変換の既存動作は上記比較で確認した。実機F405の走行回帰は今回未実施。

今回のbuildは本流 `a461d4f` のclean状態（その後はテストと文書だけ変更）。新たなF413 HILは未実施なので、探索730mm区間の壁非接触停止、吸引・高速・斜めの追従/壁余裕、新しいtrace保持の実FRAMでの動作は既存の未確認事項として残す。mini_r3のprecomputed route互換制限、unit002の極性確認待ちなどは `STATE.md` と個別HIL記録に従う。

## 統合に含まれるPR

統合前のbase/headはローカル監査の `open-prs-before-integration.json` に保存する。下記PRはheadの包含を確認してbaseをmainへ揃え、統合結果のMERGED状態を確認する。branchは削除しない。

| PR | Head | 内容 |
|---|---|---|
| [#22](https://github.com/xfa273/nightfall-fw/pull/22) | `89e4b888d7b2` | fix(flashing): use native CubeProgrammer on Apple Silicon |
| [#23](https://github.com/xfa273/nightfall-fw/pull/23) | `1e99fbb3dcb0` | fix(f413): restore mini r3 IMU-assisted velocity for floor tuning |
| [#24](https://github.com/xfa273/nightfall-fw/pull/24) | `928ac194a02f` | tune(f413): save mini r3 2S fan-off baseline and front-wall audit |
| [#25](https://github.com/xfa273/nightfall-fw/pull/25) | `dae0c4eb722f` | calibrate(f413): extend mini r3 front-wall LUT to 110 mm |
| [#26](https://github.com/xfa273/nightfall-fw/pull/26) | `c09d7fbf2e94` | fix(f413): safe stop approach before front alignment |
| [#27](https://github.com/xfa273/nightfall-fw/pull/27) | `9596816f7016` | feat(f413): mini_r3 mode4吸引50%とcase0ターン調整を準備 |
| [#28](https://github.com/xfa273/nightfall-fw/pull/28) | `f8b3a2fdfda7` | feat(f413): prepare mode6 suction turns and FAN_ON gain tuning |
| [#29](https://github.com/xfa273/nightfall-fw/pull/29) | `48f8bd23b348` | fix(logging): correct trace scales and standardize display units |
| [#30](https://github.com/xfa273/nightfall-fw/pull/30) | `8425d4715d1d` | fix(f413): 固定速度制限と調整用上書きを撤去して走行paramsを反映 |
| [#31](https://github.com/xfa273/nightfall-fw/pull/31) | `982038469131` | tune(mini-r3): 吸引走行の並進FFと方位追従を調整 |
| [#32](https://github.com/xfa273/nightfall-fw/pull/32) | `4e1902bd35bd` | fix(f413): 吸引発進時の積分持ち越しとcase0停止待ちを修正 |
| [#33](https://github.com/xfa273/nightfall-fw/pull/33) | `ef769dbac992` | mini_r3: mode7吸引ターン追加とcase0速度指令修正 |
| [#34](https://github.com/xfa273/nightfall-fw/pull/34) | `d395d1384124` | mini_r3: mode3〜7の吸引速度段階と最小接続加速度を設定 |
| [#35](https://github.com/xfa273/nightfall-fw/pull/35) | `fae343e7372a` | F413: 複数走行ログをFRAMに保持し全件書き出す |
| [#36](https://github.com/xfa273/nightfall-fw/pull/36) | `2faf46a8df36` | F413: case0大回り180°の助走を90 mm延長 |
| [#37](https://github.com/xfa273/nightfall-fw/pull/37) | `97c6e46ed1eb` | F413: params.hで自動撮影用LED信号と待機を切替可能にする |
| [#38](https://github.com/xfa273/nightfall-fw/pull/38) | `064411bbd693` | Logging: 走行別CSV保存の確認と旧キャプチャの更新忘れ検知 |
| [#39](https://github.com/xfa273/nightfall-fw/pull/39) | `270419e067c9` | F413: mini_r3ゴール座標の反映漏れを診断記録 |
| [#40](https://github.com/xfa273/nightfall-fw/pull/40) | `eade285bd331` | fix(f413): 機体別paramsの参照漏れを修正 |
| [#41](https://github.com/xfa273/nightfall-fw/pull/41) | `a461d4f96447` | fix(f413): 探索の処理中移動による距離累積を補正 |

## 次に残る整理

| 対象 | 次の判断 |
|---|---|
| #1 | 旧CSV UI。後発実装との重複と採否 |
| #16 | WeAct adapter設計。独立して採用判断 |
| #17/#18/#19 | 探索sim/C探索/runtime hooks。現行本流と競合する実装の比較、段階的採用 |
| #20 | mini_r2 encoder pad/取付穴。CAD正本と整合して採用判断 |
| #21 | 計測データturn simulator。現行tuningツールとの役割整理 |
| 未コミットparams/USB/CAD/BOM/旧sim・別worktreeの調整 | 保全済みcheckpointから目的ごとに採否を決める。mini_r3 CAD READMEの4層/未配線という旧記載と実PCB2層の不一致も残る |
| 通常作業checkout | dirty変更を保持した `fix/f413/search-distance-accounting` のまま。次の開発をmain起点へ移す際に保全済み調整を選んで持ち越す |

main参照だけを更新し、既存13ワークツリーのHEAD/ファイル/status、stash2件、旧branchは保持する。ローカル容量削減やワークツリー撤去は行わない。
