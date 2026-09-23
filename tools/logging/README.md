# tools/logging

ログ収集・変換・可視化のホスト側ツールを配置します。

## 現在の内容

- `serial_capture_csv.py`: シリアル受信をCSVへ保存
- `analyze_trace_csv.py`: trace CSVをflags位相で要約（idle/forward/coast/reverse/smoke）
- `analyze_turn_csv.py`: F413ターン調整用に角速度積分・最終角度誤差・オーバーシュート等を要約
- `analyze_wall_end_csv.py`: F413壁切れの微分値・検出位置・次ターンまでの距離と90度ターン前後オフセットを要約
- `analyze_front_match_csv.py`: 前壁位置合わせテストの状態遷移・距離誤差・指令追従を要約
- `analyze_search_event_log.py`: 探索の区画判断・動作結果・壁切れ・前壁位置合わせ結果を要約
- `export_plotjuggler_csv.py`: FRAM trace CSVをPlotJugglerで読みやすいCSVへ変換
- `run_plotjuggler.sh`: CSV変換後、固定テンプレート付きでPlotJugglerを起動
- `render_search_dump.py`: F413 UART `@` の `[SEARCH-DUMP]` をASCII迷路へ変換
- `serial_terminal.py`: ST-LINK VCPなどで使うシンプルな対話式UART端末
- `serial_capture_csv.sh`: シェル版CSVキャプチャ
- `serial_minicom.sh`: minicomベース受信
- `serial_monitor.sh`: シンプル受信
- `visualizer/`: 可視化ツール一式（`run_visualizer.sh`, `web_visualizer.py`）

## デフォルト保存先

- `serial_capture_csv.py` / `serial_capture_csv.sh` の既定保存先は `tools/logging/logs` です。
- `web_visualizer.py` の既定ログフォルダも `tools/logging/logs` です。
- `MICROMOUSE_LOG_DIR` 環境変数を設定すると、そのパスを優先します。

## `serial_capture_csv.py` のコマンド送信

- スクリプト更新後は、受信中のdumpが完了してからCtrl+Cで終了し、
  `python3 tools/logging/serial_capture_csv.py` を再実行してください。
  起動中プロセスの処理はファイル更新だけでは切り替わりません。
  更新版は起動時に `CSV output : one file per run` と表示し、実行中のソース更新も警告します。
  自動再起動や起動時コマンドの再送信はしません。
- キャプチャ中に同じポートへUARTコマンドを送信できます（別シリアルモニタ不要）。
- 起動時に自動送信する場合: `python3 tools/logging/serial_capture_csv.py --send q,y,V`
- 起動後に手入力する場合: 実行中ターミナルで `q,y,V` と入力して Enter
- コマンドは半角ASCIIで入力してください（全角文字は送信せず警告表示します）。

## ST-LINK V3 MINIE VCPでのUART通信

F413の現行 `Debug-stm32f413` は `USART1` を `921600 8N1` で使います。ST-LINK V3 MINIEの `TX/RX/GND` を機体側UARTへ接続している場合、次でシリアル端末を開けます。

`NIGHTFALL_F413_UART_BAUD_RATE` をCMake cacheで変更した場合は、その値に合わせてください。

```bash
python3 tools/logging/serial_terminal.py
```

ポート候補を確認する場合:

```bash
python3 tools/logging/serial_terminal.py --list
```

ポートを明示する場合:

```bash
python3 tools/logging/serial_terminal.py --port /dev/cu.usbmodem112202
```

終了は `Ctrl-]` または `Ctrl-C` です。`tools/logging/serial_monitor.sh` と `tools/serial_monitor.sh` からも同じ端末を呼び出せます。

## `analyze_trace_csv.py` のディレクトリ指定

- `--expect x` / `--expect y` / `--expect z` / `--expect j` を付けると、ディレクトリ指定時は期待位相を満たす最新CSVを優先して選択します。
- `--expect z` は `search-safe` セッション（`motor_forward` + `motor_coast`）を検証します。
- `--expect j` は `shortest-safe` セッション（`motor_forward` + `motor_coast` + `motor_reverse`）を検証します。
- `z/j` が solver path + closed-loop session として動作した場合は、CSVの `flags` に bit12 (`0x1000`) が含まれます。
- `1`〜`5` の closed-loop 調整用テストも同じ `V` dump とCSV解析の流れで確認できます。
- 要約出力では `abort_count[...]`（bit8〜bit11）を表示し、停止理由フラグの有無を確認できます。
- `segments` の `d_enc=(L,R)` は 16bit符号付き差分で表示されるため、エンコーダwrap時の見かけ上の大値を抑制できます。
- 例:
  - `python3 tools/logging/analyze_trace_csv.py tools/logging/logs --expect y`
  - `python3 tools/logging/analyze_trace_csv.py tools/logging/logs --expect z`
  - `python3 tools/logging/analyze_trace_csv.py tools/logging/logs --expect j`

## `analyze_turn_csv.py` のターン評価

- `omega_z_mdps` を時間積分し、旋回角度の推定値を出します。
- 通常は `motor_reverse` 位相だけを評価します。全レコードを見る場合は `--all-records` を付けます。
- UART `3` / `4` の `#last_test_id` がCSVメタにある場合は目標角を自動推定します。
- 例:
  - `python3 tools/logging/analyze_turn_csv.py tools/logging/logs --target-angle -90`
  - `python3 tools/logging/analyze_turn_csv.py tools/logging/logs --target-angle 90 --tolerance 3`

## `analyze_wall_end_csv.py` の壁切れ・前後オフセット評価

- `wall_trace_observe=2` のログから、F405互換窓微分、検出側、検出距離、検出時センサ値を表示します。
- 壁切れ検出から次の角速度区間開始までの距離を `event_to_turn` として表示します。
- 完了した90度ターンの角速度区間を速度・IMU角速度で積分し、90mm x 90mm軌道に必要な入り・出オフセット候補を表示します。
- 180度指定では、ログ欠落のない完走区間だけからレーン間隔、入り/出オフセット差、角加速度候補を表示します。
- 現在値を指定すると、旋回本体と前後オフセットを合計した移動量も表示します。
- 例:
  - `python3 tools/logging/analyze_wall_end_csv.py tools/logging/logs --dist-in 3 --dist-out 5`
  - `python3 tools/logging/analyze_wall_end_csv.py tools/logging/logs/trace_bin_YYYYMMDD_HHMMSS.csv --target-axis-mm 90`
  - `python3 tools/logging/analyze_wall_end_csv.py tools/logging/logs --target-angle 180 --dist-in 12 --dist-out 19 --alpha 4697`

## `analyze_front_match_csv.py` の前壁位置合わせ評価

- `mode1 case0 sub4` の専用テレメトリを抽出します。
- 各状態の継続時間、左右前センサの換算距離、位置・姿勢誤差、速度・角速度の指令値と実測値を表示します。
- `paused-wall-lost` / `paused-too-close` により安全停止の理由も確認できます。
- 例:
  - `python3 tools/logging/analyze_front_match_csv.py tools/logging/logs`
  - `python3 tools/logging/analyze_front_match_csv.py tools/logging/logs/trace_bin_YYYYMMDD_HHMMSS.csv`

## `analyze_search_event_log.py` の探索要約

- `motion_end` には動作時間と終了状態を表示します。
- 探索中に前壁位置合わせを行った動作では、`fm1` / `fm2` に完了種別、所要時間、最終位置・姿勢誤差を表示します。
- 完了種別は `complete`（通常完了）、`relaxed`（時間経過後の緩和完了）、`timeout`、`wall_lost`、`aborted`、`not_run` です。
- 前壁位置合わせ結果は既存の動作レコード内に格納するため、長時間探索のレコード消費量は増えません。
- 例:
  - `python3 tools/logging/analyze_search_event_log.py tools/logging/logs/trace_bin_YYYYMMDD_HHMMSS.csv`

## F413の複数走行ログ保持

走行開始時に前回ログを初期化せず、FRAMのリングへ追記します。
自動サンプル（最短・調整走行など）と探索イベントログが対象です。
走行を終了して保存されたログは再起動後も残ります。既存のv6ログもそのまま追記でき、
FRAMの領域配置・記録形式・保存単位は変えません。

- ログ領域は640 KiB、104 bytes/recordで**6301レコード**。1 ms周期では**全走行合計約6.3秒**です。
  例えば3.5秒の走行を2回保存すると、最初の走行の先頭約0.7秒は残りません。
  探索イベントは1 ms周期ではないため、保持時間はイベント数によります。
- 空きがある間は前回分を保持し、満杯になると従来のリング方式で**最古のレコードから上書き**します。
  最古の走行が途中から残った場合は `trace_run_partial_start=1` を付け、Webビューアでも表示します。
- `mode9 case5` / UART `>` は保持中の**全走行を一つのbinary frame**で古い順に出力します。
  UART `V` は同じ範囲を走行ごとのヘッダ付きCSVで出力します。書き出しでログは消去されません。
  `v` / `<` の末尾256レコード、`R` の末尾8レコードという診断用抜粋は維持します。
- 更新後の `serial_capture_csv.py` はbinaryを一つの `.raw` と、走行別の
  `trace_bin_YYYYMMDD_HHMMSS_run001.csv` / `_run002.csv` …へ保存します。
  1走行なら従来のファイル名です。`V` でも走行別CSVになります。
  `seq=0` を走行の区切りとし、再起動による時刻リセットにも対応します。
  連番はそのダンプ内の古い順で、再ダンプした同じ走行の重複排除は行いません。
- 起動中のキャプチャとビューアは更新後に再起動してください。取得済みrawも
  `python3 tools/logging/trace_bin_dump.py path/to/trace.raw` で走行別に再変換できます。
  `--csv-out result.csv` は複数走行の場合 `result_run001.csv` などの出力先になります。
  standalone受信の既定待機は、満杯時の約655 kB出力を考慮して30秒です。

旧版キャプチャで保存した結合CSVを再変換した場合は、分割CSVの内容を確認後、
元の結合CSVを `logs/combined_originals/` などへ移して保管してください。
ビューアは従来どおり各 `_runNNN.csv` を選択して使います。
`Refresh log list` で一覧を更新すればよく、今回の分割に表示側の変更は不要です。

`op_mode/case/sub/test_id` は各走行レコードから取得します。
走行時のFW SHA・dirty状態・ゲイン値は既存v6形式には保存されません。
`V` の `fw_git_sha` / `fw_git_dirty` や追加調整情報は `fw_metadata_scope=dump_time` を付け、
書き出し時点の値と明示します。WebでもFW SHAに `(dump)` を表示します。
binaryからのCSVには取得時FWを推測して補いません（`fw_capture_metadata=not_stored`）。
ターン解析は現在の `last_test_id` を過去走行の目標角として使わず、記録済みtest IDまたは
`--target-angle` を使います。

空白ヘッダ（全0x00または全0xFF）のみ新規初期化します。
未知の旧スキーマや破損ヘッダ、読み込みエラーでは既存領域を消さず、ログ開始エラーを表示します。
旧スキーマの場合は対応FWで先に書き出してから、明示的なメンテナンス手順で初期化してください。
既存のRAMステージング上限4092サンプル・モータ動作中のFRAM書き込み抑止・8件ごとのヘッダ確定は維持します。
電源断前の未保存RAMデータや未確定ヘッダの復旧は対象外です。

ホスト上の実シリアライザ・ロガー・dumpと疑似UARTによる試験（実機に接続しません）:

```bash
tools/logging/test_f413_trace_retention.sh
sh tools/hil/run_f413_nvm_guard_tests.sh
```

## FRAM trace CSV表示（調整時の標準運用）

調整作業ではWeb版ビューアを使います。Web UI起動後は、左サイドバーのCSVメニューでログを選ぶだけで、FRAM traceログ用の固定グラフが表示されます。

```bash
tools/logging/visualizer/run_visualizer.sh
```

表示対象:

- `*.plotjuggler.csv` は一覧から除外します。
- 物理量は基本的にSI単位へ換算し、ユーザ指定により角度・角速度はdeg・deg/sで表示します。時間軸・X min/max・Durationは秒です。
- 距離m、速度m/s、加速度m/s²、角度deg、角速度deg/s。各グラフの軸・系列名にも単位を付けます。
- モータ指令は符号付き比率（1=100%、元ログの1000に相当）。ADC/エンコーダはカウント、flags/予約欄はrawとして明示し、校正情報なしに電圧や距離へ換算しません。
- `tune_ref_m_s` / `tune_error_dps` などの派生列は、TUNEフラグと軸番号があるログのみ生成して対応する物理量のグラフに配置します。壁観測の予約欄は調整指令とみなしません。
- 元のmdps/mdegと換算値を同じ軸に重ねません。元CSV・binary・firmwareの保存単位/スキーマは維持し、読み込み時に換算します。
- アップロードしたCSVは `tools/logging/logs/uploaded/` に保存されます。
- 旧8列CSVはプリセットや列名指定で表示できます。時刻は秒へ換算し、単位情報のない任意のparam列はrawのまま表示します。

新しいログを取得した後は、Web UI左側の `Refresh log list` を押してからCSVを選びます。`Auto-refresh log list` を有効にすると、数秒ごとに一覧が更新されます。PlotJugglerのCSV読み込みポップアップは使わないため、調整作業中は毎回同じ操作で確認できます。

表示モジュールを更新した際はビューアを再起動してください。古いimport済みモジュールが
プロセス内に残る場合があります。表示単位の単体/図生成試験は次で実行できます。

```bash
python3 -m unittest discover -s tools/logging -p 'test_trace_units.py' -v
```

2026-09-21修正: 23:04:42のmode6/case0/sub1ログ（3866行）で、実角速度
`-1684200 mdps`を未換算のまま`deg/s`軸に描画していた。正しくは
`-1684.2 deg/s`。角度にも同じ1000倍混在があった。
修正後の時間幅は3.986 s、実角度最小-90.359 deg。

## PlotJugglerでのFRAM trace CSV表示（詳細確認用）

PlotJugglerで見る場合は `run_plotjuggler.sh` を使います。Nightfall標準テンプレート `tools/logging/plotjuggler/nightfall_f413_tune.xml` を指定して起動します。

macOSで未インストールの場合:

```bash
brew install --cask plotjuggler
```

最新ログを開く:

```bash
tools/logging/run_plotjuggler.sh
```

CSVを指定して開く:

```bash
tools/logging/run_plotjuggler.sh tools/logging/logs/stm32_log_20260510_153411.csv
```

ファイル選択ダイアログでCSVを選んで開く:

```bash
tools/logging/run_plotjuggler.sh --pick
```

内部では、元CSVを相対秒 `time` 列付き・SI単位（角度deg・角速度deg/s）の `*.plotjuggler.csv` に変換してから、次の形で起動します。標準テンプレートも同じ単位の列を使い、角度と角速度は別のグラフに表示します。

```bash
plotjuggler --nosplash --datafile path/to/log.plotjuggler.csv --layout tools/logging/plotjuggler/nightfall_f413_tune.xml
```

別テンプレートを試す場合:

```bash
tools/logging/run_plotjuggler.sh --layout path/to/custom.xml tools/logging/logs/stm32_log_20260510_153411.csv
```

手動で読み込む場合:

1. PlotJugglerを起動する。
2. `python3 tools/logging/export_plotjuggler_csv.py tools/logging/logs` で変換後の `*.plotjuggler.csv` を作る。
3. `File` → `Load data...` で変換後の `*.plotjuggler.csv` を選ぶ。
4. CSV読み込みダイアログで区切り文字は `,`、時刻列は `time` を選ぶ。
5. `File` → `Load layout...` で `tools/logging/plotjuggler/nightfall_f413_tune.xml` を読む。
6. 左側の一覧から追加で見たい系列があれば右側へドラッグする。
7. 自分用に調整したら `File` → `Save layout...` でXMLレイアウトとして保存する。
8. 次回以降は先に同じ形式の `*.plotjuggler.csv` を読み込み、`File` → `Load layout...` で保存したXMLを読む。

まず見ると便利な系列:

- `target_distance_m` と `distance_m`: 距離の目標と実測
- `target_velocity_m_s` と `real_velocity_m_s`: 速度追従
- `velocity_error_m_s`: 速度誤差
- `motor_duty_l` と `motor_duty_r`: 左右モータ出力比率
- `angle_deg` と `target_angle_deg`: 角度追従
- `target_omega_dps` と `real_omega_dps`: 角速度追従
- `flag_motor_forward` / `flag_motor_coast` / `flag_motor_reverse`: 走行位相

調整用ログの場合、軸に応じて `tune_ref_m` / `tune_ref_m_s` / `tune_ref_deg` /
`tune_ref_dps` と対応する `tune_error_*` を追加します。`reserved_i32_0` は生値を保持。
前壁合わせのyaw errorはFR-FL距離差なのでmで表示し、角度とは区別します。

旧単位の列名を使う独自レイアウトには、変換時に `--legacy-units` を指定できます。
標準テンプレートは、このオプションなしで新たに変換したCSVと組み合わせてください。
`--no-derived` は派生列を省略しますが、既定の単位換算（s/m/m/s/m/s²/deg/deg/s）は適用します。

## `render_search_dump.py` の探索map表示

- F413 UART `@` の `[SEARCH-DUMP] y=..:` 行を含むログを、壁付きのASCII迷路へ変換します。
- 下位4bitを `W/S/E/N = 1/2/4/8` として解釈します。
- 通常チェックでは、F405互換の開始セル強制東壁だけは片側表現として許容します。
- 例:
  - `python3 tools/logging/serial_capture_csv.py --show-noncsv --send @ tools/logging/logs /dev/cu.usbmodem112202 921600 > /tmp/search_dump.log`
  - `python3 tools/logging/render_search_dump.py /tmp/search_dump.log`
  - `python3 tools/logging/render_search_dump.py --summary-only /tmp/search_dump.log`
  - `python3 tools/logging/render_search_dump.py --summary-only --strict-consistency /tmp/search_dump.log`

## 互換パス

既存運用向けに以下はラッパーとして維持しています。

- `tools/serial_capture_csv.py`
- `tools/serial_capture_csv.sh`
- `tools/serial_minicom.sh`
- `tools/serial_monitor.sh`
- `tools/log_visualizer/*`

新規運用では `tools/logging/` 配下を利用してください。
