# NVM Trace Log Format Spec (v3)

この文書は `NVM_AREA_TRACE_LOG` に保存するトレースログの形式を定義します。

---

## 1. 対象

- 対象エリア: `NVM_AREA_TRACE_LOG`
- 主用途: `STM32F413` + FRAM backend での走行後ログ解析
- schema version: `0x00030000` (`NVM_TRACE_LOG_SCHEMA_VERSION`)
- CSV format name: `nightfall_trace_csv_v3`

---

## 2. データレイアウト

エリア先頭から以下を配置する。

1. `nvm_trace_log_header_t`（固定長）
2. `nvm_trace_log_record_t` のリングバッファ

`record_capacity` は以下で決まる。

```text
(area_size_bytes - sizeof(nvm_trace_log_header_t)) / sizeof(nvm_trace_log_record_t)
```

F413では走行・調整用動作の開始時にこの領域をformatし、直近1回分の実行ログをFRAMへ自動保存する。

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
- `version`: `0x00030000`
- `length`: `sizeof(nvm_trace_log_header_t)`
- `crc`: ヘッダpayloadの加算チェックサム
  - 対象: `record_size` 以降（先頭16byteを除く）
- `record_size`: `sizeof(nvm_trace_log_record_t)`
- `record_capacity`: 格納可能レコード数
- `write_index`: 次に書く位置（リング）
- `total_records`: format後に追記した総レコード数

---

## 4. Record 定義

`nvm/nvm_trace_log.h`

```c
typedef struct __attribute__((packed)) {
    uint32_t seq;
    uint32_t timestamp_ms;
    int32_t target_distance_x1000;
    int32_t distance_mm;
    int32_t angle_mdeg;
    int32_t target_velocity_mm_s;
    int32_t real_velocity_mm_s;
    int32_t accel_velocity_mm_s;
    int32_t target_omega_mdps;
    int32_t real_omega_mdps;
    int32_t target_angle_mdeg;
    int32_t accel_forward_mm_s2;
    int32_t reserved_i32_0;
    int32_t reserved_i32_1;
    int32_t reserved_i32_2;
    int32_t reserved_i32_3;
    int16_t encoder_l;
    int16_t encoder_r;
    int16_t motor_out_l;
    int16_t motor_out_r;
    uint16_t adc_fr;
    uint16_t adc_r;
    uint16_t adc_fl;
    uint16_t adc_l;
    uint16_t adc_vbat;
    uint16_t flags;
    uint8_t op_mode;
    uint8_t op_case;
    uint8_t op_sub;
    uint8_t test_id;
    uint16_t reserved_u16_0;
    uint16_t reserved_u16_1;
} nvm_trace_log_record_t;
```

### 各フィールド

- `seq`: 記録番号（format後0始まり）
- `timestamp_ms`: 記録時刻（ms）
- `target_distance_x1000`: 距離目標（mm * 1000）
- `distance_mm`: 制御側累積距離（mm）
- `angle_mdeg`: 制御側累積角度（mdeg）
- `target_velocity_mm_s`: 並進目標速度（mm/s）
- `real_velocity_mm_s`: 制御用推定並進速度（mm/s）
- `accel_velocity_mm_s`: 加速度補助速度推定（mm/s）
- `target_omega_mdps`: 角速度目標（mdps）
- `real_omega_mdps`: IMU Z角速度（mdps）
- `target_angle_mdeg`: 角度目標（mdeg）
- `accel_forward_mm_s2`: 前後加速度（mm/s^2）
- `reserved_i32_0..3`: `#wall_trace_observe=1` の場合は壁センサdelta（FR, R, FL, L）、それ以外は将来拡張用32bit符号付き予備
- `encoder_l/r`: 左右エンコーダカウンタ値
- `motor_out_l/r`: 左右モータ制御出力（符号付きPWM相当）
- `adc_fr/r/fl/l`: 壁センサADC raw
- `adc_vbat`: バッテリADC raw
- `flags`: 状態フラグ
- `op_mode`: 操作UI mode
- `op_case`: 操作UI case
- `op_sub`: 操作UI sub
- `test_id`: UART/テスト識別子
- `reserved_u16_0..1`: `#wall_trace_observe=1` の場合は壁観測flags/壁切れ距離圧縮値、それ以外は将来拡張用16bit予備

### flags

- bit0 (`0x0001`): スイッチ押下状態（押下時1）
- bit1 (`0x0002`): idle run session（`x`）
- bit2 (`0x0004`): motor forward 区間
- bit3 (`0x0008`): motor coast 区間
- bit4 (`0x0010`): motor reverse / turn 区間
- bit5 (`0x0020`): smoke+trace 区間（`g`）
- bit6 (`0x0040`): search entry / search-safe 区間（`z`）
- bit7 (`0x0080`): shortest entry / shortest-safe 区間（`j`）
- bit8 (`0x0100`): run abort reason = switch
- bit9 (`0x0200`): run abort reason = wall sensor fault
- bit10 (`0x0400`): run abort reason = encoder fault
- bit11 (`0x0800`): run abort reason = imu fault
- bit12 (`0x1000`): solver path / closed-loop test session
- bit13 (`0x2000`): angle target enabled
- bit15 (`0x8000`): 自動収集で追加されたレコード

---

## 5. CSV出力形式

`v` / `V` / `mode9 case5` は以下のメタ行を出力する。

```text
#log_format=nightfall_trace_csv_v3
#fw_target=...
#fw_version=...
#fw_build_type=...
#fw_git_sha=...
#fw_git_dirty=...
#fw_log_schema=0x00030000
#wall_trace_observe=1
#wall_trace_reserved_i32=delta_fr,delta_r,delta_fl,delta_l
#wall_trace_reserved_u16_0=flags
#wall_trace_reserved_u16_1=dist_q4_lr
#mm_columns=timestamp_ms,seq,op_mode,op_case,op_sub,test_id,target_distance_mm,distance_mm,angle_mdeg,target_velocity_mm_s,real_velocity_mm_s,accel_velocity_mm_s,target_omega_mdps,real_omega_mdps,target_angle_mdeg,accel_forward_mm_s2,encoder_l,encoder_r,motor_out_l,motor_out_r,adc_fr,adc_r,adc_fl,adc_l,adc_vbat,flags,reserved_i32_0,reserved_i32_1,reserved_i32_2,reserved_i32_3,reserved_u16_0,reserved_u16_1
```

CSV行は `#mm_columns` と同じ順序で、oldest→newest に出力する。

`#wall_trace_observe=1` の場合、`reserved_u16_0` は以下のbitを持つ。

- bit0 (`0x0001`): front wall
- bit1 (`0x0002`): right wall
- bit2 (`0x0004`): left wall
- bit3 (`0x0008`): wall ADC saturation
- bit4 (`0x0010`): wall-end state right wall
- bit5 (`0x0020`): wall-end state left wall
- bit6 (`0x0040`): right wall-end detected
- bit7 (`0x0080`): left wall-end detected
- bit8 (`0x0100`): wall-end detection gate enabled
- bit9 (`0x0200`): wall control active
- bit15 (`0x8000`): wall trace observe enabled

`reserved_u16_1` は下位8bitに右壁切れ検出距離、上位8bitに左壁切れ検出距離を `distance_mm / 4` で格納する。

---

## 6. F413操作

- `q`: trace log format
- `r`: sample append
- `R`: latest dump（人間確認用）
- `v`: CSV dump（最大256件）
- `V`: CSV dump（保存済み全件）
- `k`: selftest（format→固定パターンappend→readback検証）
- `u`: run start hook
- `U`: run stop hook
- `mode9 case5`: test_mode内の直近全ログCSV出力

走行・調整用動作はrun start hookでFRAMログをformatし、実行中に自動appendする。実行後は `mode9 case5` を選ぶだけで、直近実行の全レコードを一括出力できる。

---

## 7. ホスト側CSV取得

- `tools/logging/serial_capture_csv.py` は `#mm_columns=` を認識してCSV保存できる
- `tools/logging/analyze_trace_csv.py` と `tools/logging/analyze_turn_csv.py` はv1の `omega_z_mdps` とv2の `real_omega_mdps` の両方を読める
- `flags` により、直進・旋回・coast・abort理由を識別できる
