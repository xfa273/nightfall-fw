# NVM Trace Log Format Spec (v1)

この文書は `NVM_AREA_TRACE_LOG` に保存するトレースログの形式を定義します。

---

## 1. 対象

- 対象エリア: `NVM_AREA_TRACE_LOG`
- 主用途: `STM32F413` + FRAM backend での走行後ログ解析
- schema version: `0x00010000` (`NVM_TRACE_LOG_SCHEMA_VERSION`)

---

## 2. データレイアウト

エリア先頭から以下を配置する。

1. `nvm_trace_log_header_t`（固定長）
2. `nvm_trace_log_record_t` のリングバッファ

`record_capacity` は以下で決まる。

```text
(area_size_bytes - sizeof(nvm_trace_log_header_t)) / sizeof(nvm_trace_log_record_t)
```

---

## 3. Header 定義

`nvm/nvm_trace_log.h`

```c
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    uint32_t length;
    uint32_t crc;
    uint32_t record_size;
    uint32_t record_capacity;
    uint32_t write_index;
    uint32_t total_records;
} nvm_trace_log_header_t;
```

### 各フィールド

- `magic`: `0x544C4F47` (`"TLOG"`)
- `version`: `0x00010000`
- `length`: `sizeof(nvm_trace_log_header_t)`
- `crc`: ヘッダpayloadの加算チェックサム
  - 対象: `record_size` 以降（先頭16byteを除く）
- `record_size`: `sizeof(nvm_trace_log_record_t)`
- `record_capacity`: 格納可能レコード数
- `write_index`: 次に書く位置（リング）
- `total_records`: 起動後累積で追記した総レコード数

---

## 4. Record 定義

`nvm/nvm_trace_log.h`

```c
typedef struct __attribute__((packed)) {
    uint32_t seq;
    uint32_t timestamp_ms;
    int16_t encoder_l;
    int16_t encoder_r;
    int16_t motor_out_l;
    int16_t motor_out_r;
    int16_t omega_z_mdps;
    uint16_t flags;
} nvm_trace_log_record_t;
```

### 各フィールド

- `seq`: 記録番号（単調増加を想定）
- `timestamp_ms`: 記録時刻（ms）
- `encoder_l/r`: 左右エンコーダ値
- `motor_out_l/r`: 左右モータ出力
- `omega_z_mdps`: z軸角速度（mdps）
- `flags`: 状態フラグ（v1運用ビット割り当て）
  - bit0 (`0x0001`): スイッチ押下状態（押下時1）
  - bit1 (`0x0002`): idle run session（`x`）
  - bit2 (`0x0004`): motor forward 区間（`y`）
  - bit3 (`0x0008`): motor coast 区間（`y`）
  - bit4 (`0x0010`): motor reverse 区間（`y`）
  - bit5 (`0x0020`): smoke+trace 区間（`g`）
  - bit6 (`0x0040`): search entry / search-safe 区間（`z`）
  - bit7 (`0x0080`): shortest entry / shortest-safe 区間（`j`）
  - bit8 (`0x0100`): run abort reason = switch
  - bit9 (`0x0200`): run abort reason = wall sensor fault
  - bit10 (`0x0400`): run abort reason = encoder fault
  - bit11 (`0x0800`): run abort reason = imu fault
  - bit12 (`0x1000`): solver path / closed-loop test session
  - bit15 (`0x8000`): 自動収集モードで追加されたレコード

---

## 5. API

`nvm/nvm_trace_log.h` で以下を提供する。

- `nvm_trace_log_format()`
- `nvm_trace_log_get_header()`
- `nvm_trace_log_append()`
- `nvm_trace_log_read_latest()`

---

## 6. UART検証コマンド（F413）

`platform/stm32f413/.../main.c` で以下を提供。

- `q`: format
- `r`: sample append
- `R`: latest dump
- `v`: latest CSV dump（oldest→newest, 最大256件）
- `V`: latest CSV dump（oldest→newest, 全件）
- `k`: selftest（format→固定パターンappend→readback検証）
- `u`: run start hook（現在は formatして定周期append開始）
- `U`: run stop hook（現在は定周期append停止）
- `g`: hardware smoke + trace session（run start/stop hookを内部実行）
- `x`: idle run session 1000ms（run start/stop hookを内部実行、モータ駆動なし）
- `y`: motor run session short（run start/stop hookを内部実行、短時間の正転/逆転を含む）
- `z`: search entry（`NIGHTFALL_F413_REAL_RUN_PATH_ENABLED=1` では solver path + closed-loop session、solver失敗時またはgate OFF時は search-safe fallback）
- `j`: shortest entry（`NIGHTFALL_F413_REAL_RUN_PATH_ENABLED=1` では solver path + closed-loop session、solver失敗時またはgate OFF時は shortest-safe fallback）
- `1`: closed-loop straight S3（270mm）
- `2`: closed-loop straight S6（540mm）
- `3`: closed-loop right 90deg turn
- `4`: closed-loop left 90deg turn
- `5`: closed-loop S3 + R90 + S3
- `6`: open-loop left motor forward + encoder check
- `7`: open-loop right motor forward + encoder check
- `8`: open-loop left motor reverse + encoder check
- `9`: open-loop right motor reverse + encoder check
- `F`: button-armed test execution（直前に選んだ `1`〜`5`、未選択時は `1`）

補足（Phase4.5 Step8/Step9）:

- `NIGHTFALL_F413_REAL_RUN_PATH_ENABLED=1` の場合、`z/j` は実行入口で `solver_build_path()` を呼び、成功時は生成された `path[]` を closed-loop session で実行する。
- closed-loop session は `f413_control.c/h` の TIM5 1kHz 制御を使い、直進は距離目標（半区画45mm×N）、ターンは角度目標（90/180deg）で完了判定する。
- solver失敗時または `NIGHTFALL_F413_REAL_RUN_PATH_ENABLED=0` の場合は、従来の低速短区間 safe fallback を実行する。
- `1`〜`5` は `z/j` 前の調整用に、同じ run-start/stop hook と FRAM trace CSV 経路で距離・角度・位相を確認する。

想定手順:

1. `q`
2. `r` を複数回
3. `R`
4. `v`
5. `k`
6. `u` で数秒待機して `U`
7. `v` で auto capture のCSVを確認（簡易確認）
8. 機体を浮かせて `6`〜`9` で片側モータとエンコーダを確認
9. `1`〜`5` で closed-loop 直進/旋回/複合動作を確認
10. 必要に応じて `z` / `j` で solver path session を確認
11. 必要に応じて `V` で全件CSVを取得

`header` / `rec[...]` / CSV行 / `SELFTEST PASS` が表示されれば、
v1形式での保存・読出し・CSVダンプ経路まで成立している。

---

## 7. ホスト側CSV取得

- `v` / `V` コマンド出力は `#mm_columns=...` とCSV行を出力する
- `tools/logging/serial_capture_csv.py` が `#mm_columns=` を認識してCSV保存できる
- `tools/logging/analyze_trace_csv.py` が `flags` 位相（idle/forward/coast/reverse/smoke）を自動要約できる
  - `--expect x/y/z/j` で run session 別の位相セット検証ができる
- `v` / `V` コマンドは `#fw_*` メタ情報行（target/version/build/git/schema/identity）も出力する
  - `#fw_family_name`, `#fw_machine_name`, `#fw_machine_unit`, `#fw_board_id_hex` を含む
- `z/j` の solver path 実行では `flags` に bit12 (`0x1000`) が含まれるため、safe fallback と closed-loop solver path をCSV上で区別できる

例:

1. ボード起動後に `q` / `r` を複数回実行
2. `v` を実行
3. PC側で `tools/logging/serial_capture_csv.py` で受信・保存
4. `python3 tools/logging/analyze_trace_csv.py <csv_path> --expect y` で位相区間を確認
5. `z` / `j` 検証時は `--expect z` / `--expect j` を使う
