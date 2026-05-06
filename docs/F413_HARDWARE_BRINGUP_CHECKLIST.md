# STM32F413 実機ハードウェア動作確認チェックリスト（Phase4前段）

このチェックリストは、FRAM/NVM確認後に、
`STM32F413` 新機体でハードウェア各要素の健全性を順番に確認するための手順です。

対象要素:

- 壁センサ（IR LED + ADC）
- 操作用スイッチ
- IMU（SPI2）
- ブザー
- モータドライバ（STBY/DIR/PWM）
- エンコーダ
- LED（写真撮影用30秒点灯）

---

## 0. 事前準備

- [ ] `cmake --build --preset Debug-stm32f413` が成功している
- [ ] 新しい `nightfall_stm32f413.bin` を基板へ書き込み済み
- [ ] シリアルターミナルを `115200 8N1` で接続済み
- [ ] 起動時に以下が表示される
  - [ ] `[NVM-TEST] UART command mode ready`
  - [ ] `[RUN-TEST]  x=idle-run-session(1000ms), y=motor-run-session(short), z=search-entry(safe fallback), j=shortest-entry(safe fallback)`
  - [ ] `[HW-TEST]  w=wall, p=switch, i=imu, b=buzzer, o=motor, e=encoder, l=led30s, g=smoke+trace`

安全注意:

- モータ試験前に機体を必ず浮かせる（タイヤが空転できる状態）。

---

## 1. 操作用スイッチ（`p`）

### 手順

1. ターミナルで `p` を送信
2. スイッチを押していない状態で1回確認
3. スイッチを押した状態で1回確認

### 合格基準

- [ ] 押下前後で `raw` が変化する
- [ ] 例: `released or high` ↔ `pressed or low`

---

## 2. 壁センサ（`w`）

### 手順

1. ターミナルで `w` を送信
2. センサ前に壁がない状態で1回確認
3. センサ前に壁を近づけた状態で1回確認

### 合格基準

- [ ] `off` と `on` の値が取得できる
- [ ] 壁接近時に対象チャネル（R/L/FR/FL）の `on` 値が明確に増加する
- [ ] `[HW-TEST][Wall] PASS(measure done)` が出る

---

## 3. IMU（`i`）

### 手順

1. ターミナルで `i` を送信

### 合格基準

- [ ] `WHO_AM_I=0x6B expected=0x6B => PASS` が出る
- [ ] `FAIL(spi)` が出ない

---

## 4. ブザー（`b`）

### 手順

1. ターミナルで `b` を送信
2. 150ms程度のビープ音を耳で確認

### 合格基準

- [ ] 実音が出る
- [ ] `[HW-TEST][Buzzer] PASS(beep)` が出る

---

## 5. モータドライバ（`o`）

### 手順

1. 機体を浮かせる
2. ターミナルで `o` を送信
3. 正転パルス→停止→逆転パルスが出ることを目視確認

### 合格基準

- [ ] 左右モータが短時間で正逆転する
- [ ] 異音・過熱・停止不能がない
- [ ] `[HW-TEST][Motor] PASS(pulse done)` が出る

---

## 6. エンコーダ（`e`）

### 手順

1. ターミナルで `e` を送信
2. `measuring 3000 ms` の表示後、約3秒の計測中にタイヤを手で回す

### 合格基準

- [ ] `L:... d=...` と `R:... d=...` が出る
- [ ] 回した側の `d` が0以外になる

---

## 7. LED 30秒点灯（`l`）

### 手順

1. ターミナルで `l` を送信
2. 全LEDが約30秒点灯することを確認（写真撮影）

### 合格基準

- [ ] 全LEDが30秒点灯して自動消灯する
- [ ] `[HW-TEST][LED] PASS(all LEDs were on)` が出る

---

## 8. スモーク一括（`g`）

### 手順

1. ターミナルで `g` を送信

### 合格基準

- [ ] `switch / wall / imu / buzzer` のログが連続して出る
- [ ] `smoke end` まで到達する

補足:

- `g` はモータとエンコーダを自動実行しません（安全のため個別実行）。

---

## 9. 判定

以下を満たせば、ハードウェア前段確認は合格とする。

- [ ] スイッチOK
- [ ] 壁センサOK
- [ ] IMU OK
- [ ] ブザーOK
- [ ] モータドライバOK
- [ ] エンコーダOK
- [ ] LED OK

---

## 10. run-traceセッション確認（`x` / `y`）

### 手順

1. ターミナルで `x` を送信
2. 続けて `V` を送信し、`flags=32770`（`auto + idle`）の行が複数件出ることを確認
3. 機体を浮かせた状態で `y` を送信
4. 再度 `V` を送信し、`motor_out_l/r` と `encoder_l/r` が0以外の区間を含むことを確認
5. ホスト側で `python3 tools/logging/analyze_trace_csv.py tools/logging/logs --expect x` を実行
6. 続けて `python3 tools/logging/analyze_trace_csv.py tools/logging/logs --expect y` を実行

### 合格基準

- [ ] `x` 実行で `auto: STOP total=...` が0以外になる
- [ ] `x` 後のCSV `flags` に `32770`（idle）が含まれる
- [ ] `y` 実行で短時間の正逆転パルスが目視確認できる
- [ ] `y` 後のCSVに `motor_out_l/r` の変化が残る
- [ ] `y` 後のCSV `flags` に `32772`(forward) / `32776`(coast) / `32784`(reverse) が含まれる
- [ ] `analyze_trace_csv.py --expect x` / `--expect y` がともに成功する

未達項目がある場合:

- 配線/はんだを優先再確認
- 再現する失敗ログを残す
- 必要なら基板再設計候補として課題化する

---

## 11. Phase4.5 安全runセッション確認（`z` / `j`）

### 手順

1. 機体を浮かせるか、床上で十分に短い安全区間を確保する
2. `z` を送信（search-safe、低速短区間）
3. 続けて `V` を送信し、`flags` に `search-safe(bit6)` を含む区間（例: `32836` / `32840`）があることを確認
4. `j` を送信（shortest-safe、低速短区間）
5. 続けて `V` を送信し、`flags` に `shortest-safe(bit7)` を含む区間（例: `32900` / `32904` / `32912`）があることを確認
6. ホスト側で `python3 tools/logging/analyze_trace_csv.py tools/logging/logs --expect z` を実行
7. 続けて `python3 tools/logging/analyze_trace_csv.py tools/logging/logs --expect j` を実行
8. `z` または `j` 実行中にPUSHスイッチを押し、`aborted by switch` が表示されることを確認
9. `z` / `j` 実行開始時に `real-path draft ready ... steps=... turns=... straights=...` が出ることを確認（maze未保存時は `draft unavailable` でも可）
10. `real-path preview codes(...)` が出る場合、`Sx/R/L` の並びが不自然でないことを確認

### 合格基準

- [ ] `z` 実行で `search-safe start` → `search-safe end`（または `aborted by switch`）が出る
- [ ] `j` 実行で `shortest-safe start` → `shortest-safe end`（または `aborted by switch`）が出る
- [ ] `z` 後のCSV `flags` に `bit6 (0x0040)` を含む行がある
- [ ] `j` 後のCSV `flags` に `bit7 (0x0080)` を含む行がある
- [ ] guard停止時は `bit8~bit11`（switch/wall/encoder/imu）いずれかの停止理由フラグが残る
- [ ] gate ON 時は `real-path draft ready` ログに `steps/turns/straights` が出る（maze未保存時は `draft unavailable` で許容）
- [ ] gate ON 時は `real-path preview codes(...)` が出る（経路が組めた場合）
- [ ] `analyze_trace_csv.py --expect z` / `--expect j` がともに成功する
- [ ] PUSHスイッチ押下で即停止ログ（`aborted by switch`）が出る

---

## 記録テンプレート

```text
[F413 Hardware Bring-up Verification]
Date:
Board/Unit:
FW Binary:

[Switch]
- result: pass / fail
- notes:

[Wall Sensor]
- result: pass / fail
- notes:

[IMU]
- result: pass / fail
- notes:

[Buzzer]
- result: pass / fail
- notes:

[Motor Driver]
- result: pass / fail
- notes:

[Encoder]
- result: pass / fail
- notes:

[LED]
- result: pass / fail
- notes:

[Overall]
- result: PASS / FAIL
- next action:
```
