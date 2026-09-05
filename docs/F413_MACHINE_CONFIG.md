# F413 共通ファームと機種・個体設定

2026-09-06 実装。方針の正本は `NIGHTFALL_FW_DEV_POLICY.md` §7。

## 選んだ方式

`nightfall_stm32f413.elf/.bin` は **全F413機で同じもの**を使用する。
内蔵Flash sector 15のidentityを起動時に検証し、
`(family, board_id, unit_serial)` から設定を一度だけ選択する。
走行中の再選択・設定変更はしない。F405の既存mini/classic別ビルドは変更しない。

| 層 | キー／保存場所 | 内容 |
| --- | --- | --- |
| MCU | `nightfall_stm32f413` | Clock、HAL、固定バッファ上限、割り込み周期 |
| 機種 | `family + board_id`、`board/f413/f413_registry.c` | ピン構成の互換ID、搭載機能、標準ハード設定、既定profile |
| 個体 | 上記 + `unit_serial`、同registry | 配線変更等のハード上書き、個体専用profileへの差し替え |
| 走行profile | `params/<profile>/profile.c`等、共通binary内 | 寸法・PID/FF・フィルタ・壁閾値、探索2組、最短mode2..7各9case、センサLUT |
| 校正 | 各機のFRAM、既存NVM API | 壁オフセット・基準値・距離warp。今回フォーマット変更／保存なし |

`board_id` は `0x00MMNNVV`（major/minor/variant）。miniとclassicは別namespaceで、
同じboard_idでも衝突しない。`unit_serial` は機種内連番であり、F413全体の台数ではない。
従って今回の2機目は `mini_r3_0_unit001` とする。

## 現在の登録

| 個体 | family / board_id / unit | 走行profile | 左前進IN2 | Fan |
| --- | --- | --- | --- | --- |
| mini_r2_0_unit001 | mini / `0x00020000` / 1 | `0x00020001` / `f413pre-t0.1` | Low（元の配線） | なし |
| mini_r3_0_unit001 | mini / `0x00030000` / 1 | `0x00030001` / `mini-r3-seed-t0.1` | High（左配線を反転済み） | あり |

両機とも現時点は右前進IN2 High、encoder符号 L=+1/R=-1、TIM2 PSC=0/ARR=1000。
r3の左配線反転は**機種標準ではなくunit001の上書き**。
将来unit002を追加しても、この反転を無条件に引き継がない。
旧 `NIGHTFALL_F413_MOTOR_LEFT_FORWARD_IN2_HIGH` ビルド指定は廃止し、指定時はコンパイルエラー。

r2の既存数値は維持した。r3は今回直前の数値を独立コピーした**未調整seed**であり、
タイヤ径・トレッド等も初期値を継承しているだけで、新機体の実測／走行検証済みを意味しない。
r3の制御ゲイン、壁距離LUT/FRAM warp、旋回値は別途調整する。
fan搭載判定と非駆動時のgateは接続済みだが、F413の通常走行中fan制御・fan-onゲイン選択は
従来どおり未接続（fan testのみ、走行値のfan設定は0）。この移行では有効化しない。

## 起動と保護

1. `HAL_Init()`直後にidentityを読み、checksum/schema/revision整合性を検証する。
2. 機種・個体が登録済みか、対応するpin layoutか、profileのfamily/boardが一致するかを検証する。
3. ID内UIDが非zeroなら実MCU UIDとの一致を必須とする。r3等の新規個体はzero UIDを拒否する。
   r2 unit001のみ既存のzero UIDを互換許可する。
4. `default_param_profile=0` はその個体のregistry指定を使う。非zeroは選択profile IDとの一致を要求する。
   別機種profileへの任意切替スイッチではない。`capability_flags=0` は機種定義を継承し、非zeroなら一致を要求する。
5. 選択完了後にboard GPIO・ADC・SPI・TIM等を初期化する。実効値は外部から書換不能なgetterで使用する。

IDなし、破損、未知機種、未知個体、UID不一致、未対応pin layout、profile不一致はすべて停止する。
その場合はF413共通の診断UART（USART1 PA9/PA10）だけで `[SAFE] machine=...` とUIDを表示し、
board GPIO・motor/fan timer・制御IRQ・OP UIは初期化しない。復旧はSWDで行う。
さらにopen-loop motor、closed-loop start、fan testにもidentity/capability gateを置く。
診断UARTのピン対は将来F413基板でも予約すること。

## 設定の変更方法

- r2の数値: `params/f413_preorder/`（既存調整ツール互換のため旧directory名を維持）。
- r3の数値: `params/mini_r3_0/`。ここを変えてもr2の値は変わらない。
- 配線極性、encoder符号/CPR、トレッド、PWM prescaler、IMU前後軸: registryの機種定義または個体上書き。
- 同じ機種の個体ごとに走行値を変える場合: 独立profileを作成し、対象unitの`profile_override`で指定する。
  profile IDは非zeroの未使用値を割り当て、family/board_idを元機種に合わせる。
  mode/case配列とLUTを含むprofile全体が切り替わる。個体が増えてもbinaryは増やさない。
- 数値変更時は `PARAMS_TUNE_VERSION` を上げ、共通ファームを再ビルド・書き込む。
  本実装は「IDはNVM、設定値はversion管理した共通binary内」の方式。UARTでの任意値更新や
  走行parameter blobのFRAM保存は追加していない。通常の調整にidentity再書き込みは不要。

`f413_param_fields.def` がruntime scalarの型一覧、`f413_runtime_aliases.h` が既存コード向けの読取facade。
新しいscalarを加える際は両方と全profileの定義・host試験を更新する。
maze配列サイズ・goal/start座標・診断刺激の時間・MCU共通の安全上限は現時点では共通ビルド設定のまま。
区画寸法（mini half-cell=45mm、classic=90mm）と走行距離はruntime profile値を使う。

## classic F413を追加するとき

classicをmini扱いするfallbackは存在しない。まだclassic実機のピン／寸法／調整値を登録していないため、
現在のbinaryはclassic IDを安全停止させる。将来は以下で同じ仕組みに追加できる。

1. `family=classic` と固有の機種board_idを登録する。
2. 現在のCubeMX pin/peripheral contractと同一なら `F413_LAYOUT_MINI_R2` を使用可能。
   異なる場合は新layoutとplatform初期化分岐を実装する。MCUが同じという理由だけで既存layoutを指定しない。
3. classic固有のprofileを作り、half-cell=90mm、メカ寸法、極性、センサ校正、各走行値を設定する。
4. unitを登録し、下記のID登録と段階的HILを実施する。

型・resolver・profileの仕組みはclassicでも共通。host試験ではminiと同じboard_idのclassic、
異なる寸法／ゲインの選択、mini profile混入の拒否を確認する。

### 事前計算済み経路テーブルの注意

現在のKERI mode2 case6..9用テーブルはr2専用である。
`route_precomputed_compatible` はr2のみtrue、r3はfalse。
別profileでUART `K`、保存迷路からのKERI経路生成を要求すると拒否し、r2用テーブルを流用しない。
mode2 case6には従来の固定検証path割当が残るため、このgateを「全mode2を禁止」と解釈しないこと。
新機種へのKERI対応はprofile別のgeometry/timeテーブル生成・選択と実測確認が必要。
r2値を変更した場合も `tools/route_precompute/generate.py` / `--check` を必ず実行する。

## 初回ID登録（現在接続機を必ず確認）

通常のapp更新ではidentityを保持する。新機体は先にcommon appを書き込み、未登録SAFEを確認する。

```sh
python3 tools/flashing/f413_identity.py inspect --sn <STLINK_SN>
python3 tools/flashing/f413_identity.py provision-empty \
  --sn <STLINK_SN> --machine mini_r3_0 --unit-serial 1 \
  --expected-uid <inspectで確認したXXXXXXXX-XXXXXXXX-XXXXXXXX> \
  --backup-dir build/identity/<新しい一意なbackup名> --allow-identity-write
```

このツールはMCU IDとUIDを確認し、sector15全128KiBを退避してから**全域が空の場合だけ**
68byteのidentityを `--skipErase` で書く。eraseコマンドは持たない。
書き込み後は全sectorを再読し、identity以外のbyteが変化していないことも検証する。
既存identity・非blank sector・UID不一致は書き込まない。backupディレクトリは再利用不可。
生成identity.binとmanifestは再登録用に別媒体にも保管する。既存IDの改名／置換はこのツールで行わず、
退避・対象・復旧方法を確認して保護領域の明示的更新手順を別途実施する。

リセット後に `ID machine=... unit_name=...` と `[MACHINE] profile=...`、
極性・encoder符号・half-cell・fan表示を確認する。CSVにはprofile ID、tune version、実効極性を併記する。
設定metaはdump時の設定なので、旧ファームのFRAMログは更新前にdumpして保管する。
調整記録・安定版manifestは `FW_TARGET + family + board_id + unit_serial + FW SHA + tune version` を一組で残す。

## 検証

```sh
sh tools/hil/run_f413_machine_tests.sh
sh tools/hil/run_f413_motor_pwm_tests.sh
python3 -m unittest discover -s tools/flashing -p test_f413_identity.py
python3 tools/route_precompute/generate.py --check
cmake --build --preset Debug-stm32f413
cmake --build --preset Debug-stm32f405
git diff --check
```

実機では未登録SAFE→r3登録→boot表示→非モータIMU/ADC→出力OFFを確認する。
r2の接続確認は別途必要。モータや床上走行は `docs/ai/HIL_SAFETY.md` に従う。
