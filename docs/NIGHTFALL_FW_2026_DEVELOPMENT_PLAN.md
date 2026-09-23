# Nightfall FW 2026シーズン開発計画

このドキュメントは、2026シーズンに `nightfall-fw` で進める開発項目を、優先順位と依存関係が分かる形でまとめたものです。
開発運用上の前提は `docs/NIGHTFALL_FW_DEV_POLICY.md` に従います。

---

## 0. 進捗サマリ（2026-05-06）

2026-09-06追記: F413機種・個体設定のruntime選択を実装。
`mini_r2_0_unit001` / `mini_r3_0_unit001` を同一binaryで扱い、ハード設定・走行profileを分離する。
mini/classic namespace、未登録SAFE、UID検証、初回登録運用は `F413_MACHINE_CONFIG.md` を参照。
classic実機のpin adapterと調整profile、機種別KERI事前計算テーブルは実機追加時の作業。

- Phase 0: 完了
- Phase 1: 完了（既存機種の実機動作確認まで完了）
- Phase 2: 完了（`trace_printf()` 導入と `STM32F413` 起動トレース経路を整備）
- Phase 3: 完了（F405/F413起動時の識別判定、Step A/B/C、identity write path、F413 FRAM無し暫定運用チェックリストのStep2〜5実機検証を完了）
- Phase 4: 進行中（F413で `distance/sensor/maze/trace` のFRAM backend実機検証、trace_log schema v4、CSV dump、run hook連動ログ経路を整備）
- Phase 4.5: 進行中（`z/j` 実行入口を solver path + closed-loop control へ接続。調整用UARTテスト `1`〜`9` / `F` を追加し、実機ゲイン調整待ち）

---

## 1. 今シーズンの前提

- `STM32F405` 系既存機体と `STM32F413` 系新機体を並行開発する
- 機体の大別は `mini` / `classic` とする
- 正式な機体識別名は `<family>_r<major>_<minor>` とする
- 現行F413 bring-up対象は `mini_r2_0`（個体名: `mini_r2_0_unit001`）
- HW Rev の `major` は機体系世代、`minor` は基板版とする
- 共通バイナリの範囲は **MCU系列ごとに1個**とする
- `FW_TARGET`（例: `nightfall_stm32f413`）と `machine_name`（例: `mini_r2_0`）は別概念として扱う
- 機体識別は **MCU内蔵Flash予約領域の機体識別ブロック**を一次情報とする
- `STM32F413` 系の操作補助トレースは **`SWO`** を標準とする
- `STM32F413` 系のデバッグ・調整ログは **`FRAM` 保存 + `SWO` ダンプ** を標準とする
- 安定バージョンは `main` と別に、`unit_serial` ごとに管理する
- `STM32F405` 系既存機体は現行機体で完了とし、`F405 + FRAM` 派生は作らない
- `STM32F405` 系のトレース/ログは `RAM + UART` の範囲で新世代とできるだけ共通化する
- 新機体の部品到着前に、ソフトウェア側の構造整理と準備を進める

---

## 2. 今シーズンの最上位目標

- `STM32F413` 系新機体を追加しても破綻しないファーム構成へ移行する
- `STM32F405` 系既存機体の開発継続性を維持する
- 調整・デバッグに必要なログ取得効率を大きく改善する
- 機体個体ごとの安定バージョンを後から簡単に選択・再適用できる運用形を作る
- ゴール座標などの運用設定を再ビルドなしで更新できる構造へ寄せる
- 今後の機体追加や教育用派生に耐えられる構造へ寄せる

---

## 3. 優先順位

### 3.1 最優先

- `STM32F413` 系 platform 立ち上げ
- 機体識別ブロックの導入
- `NVM` 抽象化の土台作成
- `SWO` トレースと `FRAM` ログの方針確定・実装開始

### 3.2 高優先

- ディレクトリ構成整理
- Git運用、PR確認項目、実験ログ運用の徹底
- 安定バージョン manifest / runtime settings 運用の整備
- ログ可視化ツールの新形式対応
- `STM32F405` 系既存コードからハード依存部を切り出し、`RAM + UART` の範囲でトレース/ログAPIを揃える

### 3.3 中優先

- 制御とターン軌道生成/追従手法の見直し
- 教育用基本機能の切り分け準備
- 機能フラグ管理の整理

---

## 4. フェーズ別計画

## Phase 0. 方針確定と棚卸し

### 目的

- 今シーズンの設計判断を先に固定し、後戻りを減らす

### 実施項目

- 開発ポリシー更新
- 今シーズン開発計画作成
- 命名規則、HW Rev、安定バージョン lifecycle の確定
- 現行コードのハード依存箇所、保存データ、ログ経路の棚卸し
- `_sandbox/` 内の新機体関連成果物の正式置き場を決める

### 完了条件

- ポリシー文書と開発計画文書が存在する
- 新しい命名規則と安定バージョンの扱いが文章で固定されている
- 新機体対応で必要な構造変更の論点が一覧化されている

---

## Phase 1. リポジトリ構造の再編

### 目的

- `STM32F405` 系と `STM32F413` 系を並行開発できる構造へ移行する

### 実施項目

- `platform/<mcu_family>/` 前提の構成へ整理
- `board/<board_name>/` 導入
- `nvm/` 導入
- `stable/<family>/<unit_serial>/` 導入
- `tools/flashing/`, `tools/logging/` への整理
- `hardware/<board_name>/` の正式化
- 既存名から新しい `<family>_r<major>_<minor>` への移行表を作る
- `CMake` ターゲットを MCU系列基準へ再設計

### 完了条件

- ディレクトリ責務が文書と実体で一致している
- `STM32F405` 系と `STM32F413` 系でビルドエントリを分けられる
- 新規に追加する識別名が `<family>_r<major>_<minor>` へ統一されている
- `_sandbox/` を見に行かなくても必要な設計情報へ辿れる

### 注意点

- 一度に全面移行しない
- 既存機体が壊れないよう、段階的に移す

---

## Phase 2. `STM32F413` 系 platform 立ち上げ

### 目的

- 新機体向けの最小起動・周辺初期化・ビルドを成立させる

### 実施項目

- `platform/stm32f413/` の整備
- CubeMXプロジェクトの整理と再生成ルール確立
- クロック、GPIO、ADC、SPI、TIM、エンコーダ、割り込み設定の確認
- `main` 起動までの最小構成を安定化
- `SWO` 出力の疎通確認

### 完了条件

- `STM32F413` 系ターゲットがビルドできる
- 起動時に `SWO` へトレースを出せる
- 最低限のセンサ/モータ関連周辺機能の初期化方針が固まっている

### 依存関係

- Phase 1 のディレクトリ責務整理

---

## Phase 3. 機体識別ブロックと `NVM` 基盤

### 目的

- MCU系列内で共通バイナリを使い分けるための基盤を作る

### 実施項目

- MCU内蔵Flash予約領域に機体識別ブロックを定義
- `family`, `board_id`, `hw_rev_major`, `hw_rev_minor`, `unit_serial`, `default_param_profile`, `capability_flags`, `mcu_uid`, `crc` を持つ構造体を設計
- 起動初期に機体識別ブロックを読む処理を導入
- `NVM` API を定義し、アプリ層から直接 Flash sector を触らない構造へ寄せる
- 既存の `distance_params`, `flash_params`, `eeprom` を新しい `NVM` 方針へ寄せる設計を始める
- 機体識別ブロック、ログメタ情報、安定バージョン manifest で同じ識別項目を使う

### 完了条件

- `STM32F405` 系と `STM32F413` 系の両方で機体識別ブロックを読める見通しがある
- 機体識別失敗時のセーフモード方針が決まっている
- 永続化データの責務分離が整理されている

### 注意点

- MCU固有UIDは補助情報として使い、主キーにはしない
- 機体識別ブロックは通常のアプリ書き込みで消えないようにする

---

## Phase 4. `FRAM` 組み込みとログ基盤再設計

### 目的

- 調整時間短縮に効くログ取得基盤を作る

### 実施項目

- `SPI FRAM` ドライバ実装
- ログヘッダ、ログレコード、メタ情報形式の設計
- 高頻度ログをFRAMへバイナリ保存する仕組みの実装
- 走行後にダンプする専用モードの設計
- `stable_id` を含めたログメタ情報設計
- `SWO` ダンプ受信ツール、CSV変換ツールの整備
- 既存可視化ツールの新形式対応

### 完了条件

- `STM32F413` 系でトレースとログが分離されている
- 走行後にログをまとめて取り出せる
- ログにビルド識別子と機体識別情報が含まれる
- PC側で可視化まで一連の流れが成立している

### 注意点

- 制御中に高頻度ログを文字列化しない
- ライブトレースと大容量ログを混同しない

---

## Phase 5. `STM32F405` 系既存機体との共存整理

### 目的

- 既存機体を今シーズン中も問題なく開発・運用できるようにしつつ、新規 `F405` 派生を増やさず保守対象を明確化する

### 実施項目

- `STM32F405` 系 existing platform の責務整理
- 新規 `F405 + FRAM` 派生を前提にしない設計境界を明確化
- 既存UARTトレースと新しい `trace_printf()` API の橋渡し
- 既存RAMログと新しいログヘッダ/メタ情報の橋渡し
- `params` と `board` の境界の見直し
- `mini` / `classic` の family 名と既存ディレクトリ名の移行方針整理

### 完了条件

- `STM32F405` 系既存機体が引き続きビルド・調整できる
- `RAM + UART` の範囲で新世代と概ね共通の trace/log API とメタ情報を使える
- 新しい構成方針に少しずつ移しても既存機体の運用が止まらない

---

## Phase 4.5. `STM32F413` 実走（探索/最短）統合

### 目的

- 既に成立した `x/y` run-trace 計測経路を使いながら、新機体で探索走行と最短走行を段階的に有効化する

### 実施項目

- 探索/最短の実行入口を `STM32F413` 側へ段階的に移植し、UARTから安全に起動できるようにする
- `run-start` / `run-stop` hook を探索走行・最短走行の実行経路へ接続する
- `serial_capture_csv.py` + `analyze_trace_csv.py` を使った検証手順を、`x/y` から探索/最短へ横展開する
- 実走前に「浮かせ試験」「低速短区間」「床上短区間」の3段階ゲートを設ける
- 走行中の事故防止として、異常時停止条件（壁センサ異常、エンコーダ異常、姿勢異常）を明文化する

### 完了条件

- 探索走行で開始〜停止までのtrace/CSV取得が再現可能
- 最短走行で開始〜停止までのtrace/CSV取得が再現可能
- 失敗時に、ログから「入口・位相・停止理由」を追える

### 進め方（現作業との接続）

1. 既存の `q -> x/y -> V -> analyze (--expect x/y)` を日次スモークとして維持する
2. 探索走行入口を追加したら同じ取得導線で「探索セッションCSV」を取得する
3. 最短走行入口を追加したら同様に「最短セッションCSV」を取得する
4. 探索/最短の各段階で、低速条件と停止条件を固定してから速度を上げる

### 進捗メモ（2026-05-06）

- Step1: `z` / `j` の安全run入口を追加（低速短区間・PUSHスイッチ即停止・trace flags記録）
- Step2: `z/j` 実測CSVで flags 位相とスイッチabortを確認し、`analyze_trace_csv.py --expect z/j` によるホスト検証導線を追加
- Step3: `z/j` 経路へ run guard（switch/wall/encoder/imu）を統合し、停止理由をtrace flags（bit8〜bit11）へ記録
- Step4: `z/j` を `search/shortest` 実行入口ラッパーへ接続（`NIGHTFALL_F413_REAL_RUN_PATH_ENABLED` ゲート追加、OFF時は safe fallback 実行）
- Step5: gate ON 時に `nvm_maze_load_map` + BFS で本経路ドラフト（step数/旋回回数/直進区間数）を算出し、entryラッパーから事前確認ログを出す。その後 Step6 で `solver_build_path()` へ置換済み
- Step6: F413ビルドへ solver/path/maze_grid/solver_params を統合。`params/f413_preorder/` 作成、`f413_solver_bridge.c`（MAIN_C_ globals + load_map_from_eeprom NVM ブリッジ）作成。z/j entry の BFS ドラフトを `solver_build_path()` (Dijkstra + simplify + L-turn) に置換。RAM 55KB/320KB, FLASH 81KB/1536KB
- Step7: gate ON 時に solver-path の `path[]` コード列をオープンループで走行実行。直進は forward duty × 時間、ターンは rotate duty × 時間。safe fallback の代替として path 全体を走破するセッション関数を実装
- Step8: `f413_control.c/h` を作成。TIM5 1kHz 割り込みで並進速度（エンコーダ）+ 角速度（IMU ISM330DHCX SPI2読取 + LPF + オフセット補正）の P+FF 制御を実装。solver path session をクローズドループに移行: 直進は距離目標（半区画45mm×N）到達で完了、ターンは角度目標（90/180°）到達で完了。RAM 55KB/320KB, FLASH 84KB/1536KB
- Step8補足: 現在の作業ツリーでは `NIGHTFALL_F413_REAL_RUN_PATH_ENABLED=1`。`z/j` は `solver_build_path()` 成功時に closed-loop solver path session を実行し、失敗時のみ safe fallback へ戻る
- Step9: 調整用UARTテストを追加。`1`=S3直進、`2`=S6直進、`3`=R90、`4`=L90、`5`=S3+R90+S3、`6`〜`9`=片側モータopen-loop+encoder、`F`=ボタンアーム実行
- Step10: F405 mini相当へのパス実行移植の第2ステップとして、F413 solver path runner の直進/斜め直進でIMU方位保持を有効化し、旋回到達判定を符号付き角度へ変更した
- Step11: 実機迷路走行に依存せず経路導出を確認するため、`tools/solver_host/run_solver_host.sh` を追加。F413向け `solver_build_path()` をホスト上でコンパイルし、内蔵サンプル迷路またはKeriLabのC配列形式サンプルで `path[]` コード列を確認できる
- Step12: KeriLabの `.maze` テキスト形式を `tools/solver_host/run_solver_host.sh --maze <file>` で直接読み込めるようにし、`tools/solver_host/run_kerilab_samples.sh` で代表32x32過去大会迷路を取得・一括検証できる導線を追加した
- Step13: 小回りターン（`300/400`）と大回りターン（`501` など）を含む `path[]` 導出・F413 runner解釈まで確認できたため、F405 mini相当へのF413 solver/path移植は完了扱いとする。斜め経路導出・パス変換と斜め走行調整は既知課題があり、移植後の別フェーズ課題として扱う
- Step14: UARTケーブルの抜き差しを減らすため、F413側へF405風の操作UI基盤を追加。PUSHで0〜7のモードを進め、LED1〜3で2進表示し、前右壁センサ反応かつ前左非反応をenterとして実行する。現時点の割当は `0=idle`, `1=wall-sensor-test`, `2=S3`, `3=R90`, `4=L90`, `5=S3+R90+S3`, `6=search-entry`, `7=shortest-entry`
- Step14補足: mode1の実機確認で、壁なし時のセンサ差分は概ね0〜50、壁あり時は1000以上と確認。操作時に壁ではなく手をかざしてもenterしやすいよう、F413操作UIのenter閾値を `FR>=150` / `FL<=250` に調整した
- Step15: F413操作UIをF405の `select_mode()` 体系へ近づけ、トップ `mode 0..9`、`mode1..7` 内部の `case 0..9`、さらに `case0` の `sub 0..9` を選べる3階層ステートへ変更した。PUSH時のブザー周期はF405と同じ `(11 - selected) * 400`、enter音は `900` で短く2回に合わせた。未移植のmode/case/subは安全のため実行せず、UARTへ内容と no-op 理由を表示する
- Step16: F413起動時に既存F405の下降音と区別できる機体識別用ブザー音（上昇3音＋間＋下降2音）を追加した。またテスト動作移植の第一段として、`mode2..7 case0 sub0..9` をF413の低速path-codeテストへ接続し、小回り/大回り/180/斜め45/V90/135/低速直進/高速直進を速度違いで試せるようにした。`mode9 case2` はencoder check、`mode9 case4` はfan PWM checkへ接続した
- Step17: FRAMログを全テレメトリ保存形式へ拡張し、走行・調整用動作の開始時に自動format、実行中に全テレメトリを定周期保存、終了後は手かざしなしで `mode9 case5` から直近実行ぶんを一括CSV出力できるようにした。現行CSVは `nightfall_trace_csv_v4` とし、距離・角度・目標/実速度・加速度補助速度・目標/実角速度・IMU Z角速度の生値/複数LPF候補・目標角・前後加速度・encoder・motor・壁ADC・vbat・mode/case/sub/test_id・予備フィールドを含む
- Step18: F413壁センサsnapshot APIと壁切れ状態APIを追加し、UART `w` / `W` の非モータ確認で壁なし誤検出なしと左壁あり→なし検出を確認した。さらにFRAM v2 schemaを変更せず、`reserved_i32_0..3` に壁delta（FR/R/FL/L）、`reserved_u16_0/1` に壁有無・壁切れ状態と検出距離を記録する観測接続を追加した。`NIGHTFALL_F413_DISABLE_WALL_TRACE_OBSERVE` で後から無効化できる。
- Step19: F413壁制御を直進待機ループへ弱い角度目標補正として接続した。直進中・壁検出中のみ `f413_ctrl_set_angle_target()` に±3deg以内の補正を入れ、`NIGHTFALL_F413_DISABLE_WALL_CONTROL` で後から無効化できる。壁制御active状態は `reserved_u16_0` bit9へ記録する。
- Step20: 実探索ループ接続の前段として、F413で壁snapshotからF405互換 `map[][]` へ現在区画の壁情報を書き込み、FRAMへ1024セル保存・再読込検証できる非モータUART `O` を追加した。`store_map_in_eeprom()` もF413 FRAM保存へ接続した。
- Step21: 実迷路を通信ケーブル付きで走らせる前に探索移植を進めるため、`solver_host` に仮想壁入力探索シミュレーション `--explore-sim` を追加した。KeriLab代表迷路を含む仮想壁で `map[][]` 更新→歩数マップ→次方向決定→仮想移動を検証できる。F413実機側にも非モータUART `G=search-preview` を追加し、現在の壁snapshotから次方向決定とmap保存まで確認できるようにした。
- Step22: F413実探索1ステップ実行の最小入口としてUART `N=search-step-fwd` を追加した。`G` と同じ壁snapshot→map更新→次方向決定を行い、次方向が直進の場合だけ90mm低速直進をFRAM trace付きで実行する。右左折/Uターンが必要な場合はこの段階では走らずpreview-onlyで停止する。
- Step23: UART `N` を状態保持型の探索1ステップ実行へ拡張した。初回だけ `map[][]` と `mouse` を初期化し、以後は `mouse.x/y/dir` を保持して壁snapshot→map更新→次方向決定→1動作を進める。直進は90mm、右/左/Uターンは既存角度目標制御でその場旋回し、成功後に `mouse` 状態を更新する。UART `B=search-reset` で探索ステップセッションをリセットできる。
- Step24: 実迷路なしで探索移植の状態を確認するため、F413へ非走行UART `[`/`]`/`@` を追加した。`[` は探索状態とFRAM map整合状態を表示、`]` は空探索mapをFRAM保存してリセット、`@` はFRAM上のmapを32行HEXでdumpする。
- Step25: F405互換操作UIの非走行メンテナンス移植として、F413 `mode9 case6..9` を接続した。case6は壁閾値/壁ADC確認、case7は非破壊NVMステータス表示、case8はidentity表示、case9はsensor params表示のみを行い、実パラメータ保存は安全のため行わない。
- Step26: F413 UART `@` の `[SEARCH-DUMP]` をPC側で確認しやすくするため、`tools/logging/render_search_dump.py` を追加した。FRAM map dumpをASCII迷路へ変換し、境界壁欠落や隣接セル壁不一致を要約表示できる。通常モードではF405互換の開始セル強制東壁だけ片側表現として許容し、`--strict-consistency` で厳密検出できる。
- Step27: 実機探索後に保存されたFRAM mapを実走なしで最短経路生成へ接続確認できるよう、`tools/solver_host/run_solver_host.sh --search-dump <log>` を追加した。F413 UART `@` の `[SEARCH-DUMP]` を読み込み、`map[y][x] >> 4` を壁情報として `solver_build_path()` を実行する。未探索セルが `0xF0` の空mapでは最短経路生成が失敗するのが正常で、探索後dumpが最短走行可能なmapかをPC上で判定できる。
- Step28: 制御ゲイン・各モーション調整へ入る直前の確認手順として、`docs/F413_CONTROL_TUNING_PRECHECK.md` を追加した。浮かせ固定状態での書き込み、NVM/identity/sensor params、壁センサ、IMU、エンコーダ、片側モータ、traceログ導線、調整開始順序、中止条件を一つのチェックリストへまとめた。
- Step29: F413制御をF405従来機相当のカスケード構成へ移植した。従来のF413並進FF+Pを、距離外側PID→速度内側PIDへ置き換え、角度外側PID→角速度内側PID、左右モータ合成、FRAM v2ログgetter、既存OP UI/test/path入口を維持した。壁制御も角度目標の直接上書きではなく、F405と同じく角速度指令側の補正として入る経路へ変更した。
- Step30: F405従来機のinner tune相当をF413へ移植し、速度・角速度・距離・角度の各制御ループを単独で調整できるモードを追加した。`f413_ctrl_tune_start()` でaxis/set/patternを指定し、1kHz制御tick内で通常目標生成をバイパスしてstep/triangle/trapezoid参照を与える。OP UIは `mode9 case0 sub0..9`、UARTは `!/" /#/$/%/^/&/*/(/)` のショートカットから代表プリセットを実行でき、FRAM v2ログには通常テレメトリに加えてtune参照値とaxis/set/patternメタ情報を残す。
- Step31: 斜めターン調整前の机上参照実装として、入口／出口速度を持つ二段階加速時間モデルと、小回り90・大回り90/180だけを使う拡張状態Dijkstraを `common/route/` に追加した。ゴール評価を最初のG区画境界通過時刻、ゴール後直進を停止可能性だけに使う契約へ固定し、大回り中の境界交差はF413と同じraised-cosine角速度を2次元積分して算出する。最大32x32の厳格KeriLab parser、型付き動作再生validator、既存斜め`path[]`文法validatorをhostへ追加し、固定commitの過去大会24迷路、全mode 2..7 / case 1..9、ASan/UBSan単体試験で検証した。公称軌跡と論理アンカーの残差および掃引クリアランスは未解決なので、実機コードへの接続はターン再調整・斜め対応後まで保留する。
- Step32: Step31の参照実装をhalf-grid anchor・8方位・17速度classへ拡張し、45度in/out、斜めV90、135度in/outと斜め直線を含む有限状態Dijkstraおよび型付き動作変換を追加した。評価値は引き続き最初のG区画境界通過時刻で、停止可能なtailを別に検証し、直線でGへ入った後は新しいturnを開始しない。F413 mode2をprimary、F405 mini mode2〜5をcomparisonとしてcase8/9を実parameter sourceから読む。直交turnは現調整値を時間源、PC専用exact-closure seedを幾何源とし、未調整の斜めturnは重心速度を保持したPC仮値を時間・幾何の両方に使う。全8種を固定0.001 mm closure gateで検査する。既存`convertDiagonal()`は方位とhalf-grid終点を保存するtransactional codecへ置換し、typed actionから現runnerへの変換は斜め終端と速度profile非等価を明示するgeometry gateに限定した。同一configを使うforward replay validator、別作成のprimitive期待値fixture、required-open軌跡、基板外形だけの任意turn clearance診断を追加し、固定過去大会24迷路×5profile×case8/9の240/240構成をPC上で検証した。新plannerは実機入口へ未接続であり、斜め実測調整、完成機包絡＋直進掃引、typed executorまたはlegacy runner更新、F413向け固定メモリ化を実機反映gateとする。

- Step33: Step32経路の可視化で、斜め方位の`connector_steps=0`を介して45度in/out・V90・135度turnが同じ壁中心anchorから直接連結され、区画中心外に不要な折れ返しを作れることを確認した。壁中心anchor自体は正規の斜め状態として維持し、斜め方位から次turnへ入る場合だけ最低1 diagonal half-stepを要求する不変条件をplanner列挙・turn edge生成・forward validatorへ追加した。F413 mode2 case8の専用fixtureとmatrixの明示的な0件gateを追加し、固定24迷路×5profile×case8/9は240/240 validator通過、0-step斜めturn 0件、斜め厳密短縮225、直交同値15、legacy互換90、斜め終端非対応150となった。実機入口とfirmware parameterは変更していない。
- Step34: Step33の一律1 half-step制約は、KERI #1〜#5で合法なturn直結まで除外していたため撤回した。固定commitの`StepMapSlalom`と同じprimitive固有guardをNightfallのhalf-gridへ移植し、添付で問題になった壁越しV90を拒否しながら合法な0-step 45/V90/135接続を保持した。斜めplannerの候補は#1〜#5だけとし、#0小回りは直交baseline専用に分離した。開始run-up不足には同一中心線を時空相似でLOW/CRAWL化し、小回りへfallbackしない。固定24迷路×5profile×case8/9では230構成が停止可能な経路を持ち、全230で斜め採用・直交baselineより短縮、小回り0、合法0-step 1303、LOW 219、CRAWL 49を確認した。32MM2009HXの10構成はゴール進入後の停止tailが成立しないため期待どおり`no-feasible-terminal`とした。F413にはmode2/case8固定、motor/fan/run非接続のUART `K` previewを追加し、走行用goalと独立したdiagnostic中央2x2 goalに対する#1〜#5 action列と最初のゴール進入時刻を実機上で確認できる入口を用意した。FRAM maze readに成功すれば従来どおりそれを優先し、失敗時だけprogram Flash内の固定revision 16MM2014CX fixtureへread-only fallbackするため、FRAMへ迷路を書かずに実機導出を再現できる。
- Step35: 調整済みmode2斜めturnを時間源として、F413固定メモリKERI #1〜#5 plannerを`mode2 case6..9`の実走行経路生成へ接続した。実行入口は保存済みFRAM迷路とcompiled `GOAL1..9`を必須とし、fixture fallbackを禁止する。現legacy runnerで正確に表現できるNOMINAL turnとcardinal停止tailに候補を限定し、型付きplanをtransactionalに既存`path[]`文法へ変換・検証してからrun sessionへ渡す。選択caseの直交／斜め直線制限を使い、斜めを強制せず時間最短が直交なら直交を選ぶ。route planner/clearance/motion、legacy codec/pipeline、固定メモリFRAM失敗・scratch cleanup・斜め生成propertyをhostで検証し、F413/F405両build後に既知迷路のcase6から段階的に実走確認する。
- Step36: Step35で実機経路導出に約115秒かかり、低加速caseでは最初のnominal turnへ到達できず`no-path`になったため、mode2 case6〜9のturn geometry/pose、速度別turn時間、直交・斜めconnector時間、wall-end approach時間をPCで生成するconst Flash tableへ移した。case6/7/8を低→中→高の`1000/800mm/s @ 1000mm/s^2`、`1250/900 @ 3000`、`1500/1000 @ 4000`へ段階化し、nominal大回りのrun-up不足または停止tail不足時だけ調整済みsmall90へ落とすLOW専用wall stateとrecovery passを追加した。runnerは連続S/DSを合算し、実制御と同じ単調加減速を含む全path preflightをモータ投入前に行う。保存迷路fixtureではcase6がsmall90 recovery、case7/8がnominal 45-inから始まり、全caseで斜めturn/straightを含む有効なlegacy列へ変換できた。非走行UART `+`でも保存FRAM迷路・compiled `G(1,0)`からcase6〜9の全てを4.3〜7.2秒で導出・legacy変換し、斜めturn/straight、scratch解放、motor off、NVM read-onlyを確認した。残作業はcase6→7→8の順でユーザ実走すること。
- Step37: 終点だけを一致させる従来の動画調整では旋回途中の車体接触を判定できなかったため、実runnerのraised-cosine軌道へ完成機の前後左右矩形、独立した6mm柱、壁パネルを重ね、全掃引中の最小距離を評価するhost `turn_clearance.py`を追加した。幾何gap、機械公差・モデル/動画位置誤差・方位誤差・補間誤差、要求free marginを分離し、adaptive interpolation、最初の接触と最大侵入、最接近部位、PNG/JSONを出力する。PC側では角加速度を粗探索しながらKERI標準終点を満たすin/outを0.5mm単位で解き、同一判定器を240fps動画軌跡にも適用する。angle-accumを使うcase0/case4+は`--angle-policy runtime`で45/90/135/180degを明示し、case1/2を含む`turn_tune.py`一般用途はparams角度を既定とする。動画はnominal角速度profileへ時間合わせして低角速度の旋回冒頭も保持し、到達角・出口距離・pre/post-roll・追跡gap・終点・方位をhard gateにした。normalized動画は形状/再現性だけ、実壁との最終判定はboard座標を保持するabsolute登録＋同一座標のscene JSONだけに限定する。既存27本のR135-in非接触データと最新接触動画/1ms traceをdigest付きmanifestへ索引し、現行`10250/3.5/22.0`のzero-slipモデルは標準70x39mm仮外形で内側柱へ初接触する方位`-63.3deg`（実動画/trace約`-67deg`）を再現した。最終3本からR135-in専用の速度方位遅れ係数`K=0.03303s^2/m`を導き、旋回core終点残差を0.52mmまで縮小した。この非zero係数は校正artifactを明示して右D135・500mm/s・alpha10250・正の入口距離というscope内にある場合だけ安全判定へ使い、範囲外外挿は診断値に限定する。zero-slip/empirical両軌跡と既定不確かさ・3mm marginを同時に通る候補は仮外形では得られなかったため、診断上のbest値もC代入値として出さず、実寸未測定のfirmware paramsへは未反映とした。青ラベル中心から完成機front/rear/left/rightを測定し、sceneを絶対board座標へ登録した後に粗探索を再実行し、安全候補だけを右/左各3〜5本の動画と少なくとも1本の1ms traceで最終確認する。mode2 paramsを変更した場合は実走前にroute precompute表の再生成/検査を必須とする。
- Step38: ユーザ確認の完成機外形70x39mm（青ラベル中心=機体中心=旋回中心、前後35mm・左右19.5mm）をprovenance付き既定値へ固定した。量子化in/out探索は単一の最近傍閉包点だけでなく、1ms離散実行後の終点誤差1mm内に入る全候補を評価するようにし、alpha10250では19候補から`in=24.5/out=33.0mm`が最良となったが、raw余裕4.781mmに対してeffective余裕はbootstrap 3mm位置誤差で0.682mm、既知接触を覆う推奨5mmでは-1.318mmとなり、3mm要求を満たさないため未適用とした。過去27本をtrajectory-start基準で再監査すると旧`K=0.03303`は正規化core終点専用で、補正後終点は`K=0.0261`、10〜130deg全経路は`K=0.0178`（RMSE 2.89mm）となり、単一係数が掃引軌跡を表さなかった。artifactをdiagnostic-onlyへ変更し、明示的に`safety_qualified=true`でないnon-zero modelに加え、実測掃引検証のないzero-slip理想モデルも診断候補に限定するfail-closed gateを追加した。次は青/赤ラベル面高さを使ってabsolute sceneの視差を補正し、500mm/s・exact135deg・正の固定entry/out・空きfixtureでalpha 8000/10000/12000/14000を各5本（各alpha最低1本は1ms trace）、その後左右各3〜5本の壁付き検証を行ってfull-pathモデルを再構築する。
- Step39: ユーザ実測の青ラベル面10mm・赤ラベル面2mm（迷路床基準、水平基線24mm）と、3D造形ArUco共通上面2mmをtracking/calibration artifactへ固定した。homography基準面と赤ラベル面は同じ2mmなので赤座標は不変、青だけを相対8mmの位置依存視差について補正する。固定カメラの中心XY・高さは走行軌跡から自己推定せず、盤面四隅相当の4静止姿勢でfitし中央1姿勢をheld-out検証する。fit span、凸包面積、静止時間/安定性、fit/held-out残差、物理的カメラ高さを全て通ったgeometryだけを許可し、既存CSVに残る青赤canonical座標から新しい補正CSVと入力hash付きsidecarを生成する。`turn_video_tune --propose-fit`と`turn_clearance video --registration-mode absolute`は全入力に検証済みsidecarが無ければfail closedとし、normalized未補正軌跡は診断専用に維持した。Pixel recorder 0.5.6は焦点距離・intrinsic・distortion・crop・zoom・active physical camera IDとstatic sensor geometryをadditiveに保存したが、absolute資格に必要な最終integrity/fixed-focus契約には不足する。0.5.7はAF off・固定focus 1.05Dとper-frame lens state、app version、raw video/Camera2 sidecarの同一run SHA/integrityを記録し、光信号を使わない手動one-shot 1080p/240録画をPixel UI・認証Wi-Fi・Mac dashboard/CLIへ追加した。capture fingerprintはreport、Camera2 rows、raw video、QA、trajectory、board calibrationをSHAで結び、校正動画と対象走行のphysical camera/lens/crop/zoom/fixed-focus setup一致を自動検査する。したがって0.5.6以前は診断専用である。次はcamera/marker/setupを動かさず、手動one-shot開始後1秒以上settleしてから4 fit姿勢＋1 held-out姿勢を各3秒以上保持した1本の0.5.7動画を取得し、qualified geometryが得られるまでabsolute再解析・firmware parameter変更を行わない。
- Step40: 固定rigの既知位置5点を取得したところ、定規で±2mm以内に置いた四隅4点が現座標では26--30mm内側へ一様に縮んだ。4点からの診断affineは約X 1.066倍/Y 1.071倍で、Camera2 intrinsic/distortionによる補正量は端でも約1mmに留まるため、主因を現`board_layout_8x8_60mm.json`の固定マーカ実配置との縮尺不整合と判定した。外周4マーカはframeごとのprojective registrationに限定し、盤面metricはセル中央のflushな8mm円64点などから独立にfitする`board_metric_geometry.py`へ分離した。`extract_board_metric_lattice.py`は4マーカで周期格子の番号を固定して白円を対応付け、`fit_board_metric_geometry.py`は全体32点以上・fit24点以上・held-out8点以上・凸包300000mm²以上を要求し、fit p95 0.75mm、held-out p95 1.0mm/max 1.5mmを越えるmapを拒否する。qualified dense mapは`markerless_trajectory.py`のmm位置/速度、label-plane fit、height-correctionへSHA付きで伝播し、map無しの新label-plane fitをfail closedにした。外周マーカ上面2mmはregistration専用とし、flush格子のmetric基準面を床0mmへ変更したため、label-plane段では赤2mm/青10mmをともに補正する。次は走行を乱さない薄い白円または測定済み平面のremovable dot/ChArUco targetを盤面全域へ設置し、空盤面校正clipからheld-out gateを通した後で5姿勢校正を再取得する。Pixelで1mm gateを通せない場合は低歪みglobal-shutter cameraを比較する。
- Step41: 盤面へ追加した90mm間隔の直交中心線について、ユーザがID6/4/5/7中心と四隅交点の一致を確認したため、固定マーカ中心を`(0,0)/(720,0)/(0,720)/(720,720)mm`へ修正し、旧780mm spanを廃止した。同じPixel 0.5.7空盤面clipを720mm基準で再投影すると、平均pitchはX=89.791mm（-0.232%）、Y=89.796mm（-0.227%）、best-affineからの最大非一様残差はX=1.014mm、Y=0.922mmとなった。従って以前の26--30mm収縮はPixelの端部歪みではなくlayout定義の約8%縮尺バグであり、残る光学・盤面・線引き・検出の合成誤差は約1mm級である。`probe_board_line_grid.py`を追加し、四隅マーカ中心を外周線anchorとして7本の内部直交線と画角各帯のlocal pitchを反復計測できるようにした。線probeは診断専用で、absolute safety qualificationには引き続き独立held-outを持つdense metric mapと、その後のlabel-plane 5姿勢校正を要求する。

### F405同等迷路走行までの残作業順序（2026-05-09）

1. F413壁センサ基盤をF405相当に近づける
   - 前壁・左右壁の有無判定、ヒステリシス、ADC異常検出、デバッグ表示を整理する。
   - FRAM v2ログの `adc_fr/adc_r/adc_fl/adc_l` で閾値妥当性を確認できる状態を維持する。
2. F413壁切れ検出を実装・検証する
   - F405 `detect_wall_end()` 相当の左右壁状態、微分立ち下がり検出、検出距離記録をF413実行系へ接続する。
   - まず浮かせ固定状態の手かざし/ログで検出イベントを確認し、床上短区間で距離を調整する。
3. F413壁制御を直進制御へ統合する
   - 両壁/片壁/壁なしのゲート、補正量上限、異常値除外を実装する。
   - IMU方位保持と競合しないよう、ログで `target_angle/real_angle/motor_out/adc_*` を確認しながら低速から有効化する。
4. 実探索ループをF413へ接続する
   - 1区画走行ごとに壁を読み、現在座標・方位・mapを更新し、探索アルゴリズムで次動作を決める。
   - 探索終了または停止時に `nvm_maze_save_map()` でmapを保存し、失敗理由をtrace/FRAMログから追えるようにする。
5. 探索後の最短走行フローをF405互換UIへ接続する
   - `mode1` を実探索、`mode2..7 case1+` を保存済みmapからの最短走行へ段階的に接続する。
   - 未移植のmode/case/subはno-opを維持し、実装済みになったものだけ実走可能にする。
6. 実走調整フェーズへ進む
   - 走行距離（タイヤ径/エンコーダ換算）→直進ゲイン→90/180旋回ゲイン→直進+旋回接続→壁判断→壁切れ→低速探索→低速最短→速度上げ→大回り/斜め系モーション、の順で調整する。
   - 各段階で `mode9 case5` または `V` によりFRAM v2 CSVを保存し、調整根拠を `docs/ai/WORKLOG.md` へ残す。

次ステップ: F413の単独ループ調整入口が揃ったため、速度・角速度の下位ループから順にFRAM v2ログを見ながらゲイン調整へ進む。実探索ループの実走確認は、速度/角速度/距離/角度の基本調整後に行う。

---

## Phase 6. 制御とターン軌道生成/追従の見直し

### 目的

- 実機調整の試行錯誤時間を減らす

### 実施項目

- 現状のターン軌道生成・追従手法の整理
- ログに基づく評価指標の定義
- ターン終了時の位置・角度誤差評価方法の標準化
- パラメータの物理意味と機体依存補正の分離
- CSV等でのパラメータ管理拡張検討

### 完了条件

- どのログを見れば何を評価できるかが明確になっている
- 少なくとも1つ以上、現行より調整効率の良い候補手法を比較できる

### 注意点

- 新手法の導入前に評価基準を先に決める
- 既存手法を完全に捨てず、比較可能な形で進める

---

## Phase 7. Git運用・CI・安定バージョン・補助ツール整備

### 目的

- 後から見て分かる履歴と、安全な更新手順、安定バージョンの再利用手順を作る

### 実施項目

- ブランチ命名規則の徹底
- `EXPERIMENT_LOG.md` 運用開始
- `.ioc` 更新確認の手順化
- PR確認項目のテンプレート化
- `CubeMX再生成 → 差分確認 → ビルド` の自動化検討
- 安定バージョンの `manifest.yaml` / `runtime_settings.yaml` の形式化
- 安定バージョン選択、書き込み、goal座標編集フローの整備
- `tools/flashing/`, `tools/logging/` の責務整理

### 完了条件

- 実験履歴を後から追える
- `.ioc` 変更の事故を減らせる
- 少なくとも1台分の安定バージョンを後から選び直せる
- ゴール座標などの運用設定を再ビルドなしで更新できる流れが見えている
- ツール変更とファーム変更の混線を減らせる

---

## Phase 8. 教育用基本機能の切り分け準備

### 目的

- 教育向け派生を作りやすい構造へ寄せる

### 実施項目

- 基本機能と高度機能の境界整理
- 教育用ビルドの対象機能を定義
- 競技向け機能の無効化方式を検討
- `feature flags` と `capability flags` の再整理

### 完了条件

- 教育用ビルドの要件が文章化されている
- 競技用と教育用を同じコードベースで分ける道筋が見えている

---

## 5. 具体的な実装順序

1. ポリシーと計画を文書化する
2. ディレクトリ責務と命名を整理する
3. `STM32F413` 系 platform の最小起動を成立させる
4. 機体識別ブロックと `NVM` API を導入する
5. `SWO` トレースを整備する
6. `FRAM` ログ保存とダンプを実装する
7. 安定バージョンの manifest / runtime_settings と適用フローを整備する
8. PC側ログツールを対応させる
9. `STM32F405` 系既存機体を新構成へ少しずつ追従させる
10. 制御・軌道生成の見直しをログベースで進める
11. 教育用切り分けとフラグ整理を進める

---

## 6. 今シーズン中に最低限達成したい状態

- `STM32F413` 系ターゲットが安定してビルドできる
- 新機体で `SWO` トレースが使える
- 新機体で `FRAM` ログを保存し、走行後に取り出せる
- 機体識別ブロックによって MCU系列内共通バイナリを切り替えられる
- `STM32F405` 系既存機体の開発を止めずに移行を進められる
- 少なくとも1個体分の安定バージョン manifest と再適用手順が定義されている

---

## 7. 今シーズン中にできれば達成したい状態

- `NVM` API へ既存保存処理を集約できている
- `STM32F405` 系も新しいトレース/ログAPIへ寄せ始めている
- 安定バージョンを選んで書き込み、goal座標などを編集できるツール導線がある
- ターン調整の評価指標が固まり、ログから改善判断ができる
- 教育用基本機能の切り分け方針が実装可能な粒度で固まっている

---

## 8. 保留・継続検討事項

- `.ioc` 変更検出や再生成確認のCI自動化
- PRテンプレート整備
- ログダンプ形式を CSV 主体にするか、バイナリ主体 + 変換にするかの詳細
- `runtime_settings.yaml` をどのNVM形式へ変換して書き込むかの詳細
- 教育用ビルドの公開範囲
- `STM32F405` 系既存機体への新ログ基盤の適用範囲

---

## 9. この計画の見直しタイミング

- `STM32F413` 系最小起動完了時
- `機体識別ブロック + NVM API` の仕様確定時
- `FRAM` ログダンプが一通り成立した時
- 制御手法の見直しに着手する前

見直し時は、完了項目、未着手項目、依存関係の変化を反映して更新する。

---

## 10. AI運用（Codex直接実行）

- この運用は、**ユーザーは Codex に依頼し、Codex が全作業を直接実行する** 前提で行う
- 多ファイル変更、調査→修正→検証ループが長い作業、ビルドやテストの反復が多い作業も Codex が直接担当する
- 実行時は、依頼範囲外の変更を避け、対象に近いビルド/テストで妥当性を確認する
- 運用上の重要イベントは `docs/ai/WORKLOG.md` に追記し、監査可能性を維持する
- ST-LINK/UART接続時のF413実機操作は `docs/ai/HIL_SAFETY.md` に従う
- Codex移行時点の入口資料は `AGENTS.md`、`docs/ai/CODEX_ONBOARDING.md`、`docs/ai/F413_PORTING_STATE.md`、`docs/ai/HIL_SAFETY.md` を正とする
- 旧委譲関連の互換資産は `docs/ai/archive/delegation-legacy/` に集約して保持する
