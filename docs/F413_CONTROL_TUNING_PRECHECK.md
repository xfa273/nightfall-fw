# F413 制御ゲイン・モーション調整前チェックリスト

このチェックリストは、F413で制御ゲインや各モーションの調整を始める直前に、浮かせ固定状態で安全確認とログ導線確認を行うための手順です。

前提:

- 機体は浮かせて固定し、タイヤが自由に空転できる状態にする。
- 実迷路内の走行はまだ行わない。
- 最初は直進・旋回を床に置かず、ログと挙動の方向だけ確認する。
- 書き込みはST-LINK SWD、UARTはST-LINK VCPを使う。

---

## 1. PC側で行う確認

### 1.1 ビルド

```sh
cmake --build --preset Debug-stm32f413
cmake --build --preset Debug-stm32f405
```

合格基準:

- [ ] F413ビルドが成功する
- [ ] F405ビルドが成功する

### 1.2 ホストツール確認

```sh
tools/solver_host/run_solver_host.sh --explore-sim --max-steps 512
```

合格基準:

- [ ] `[explore] result=ok` が出る

---

## 2. F413へ書き込み

```sh
python3 tools/flashing/flash_stlink --build
```

複数ST-LINKがある場合:

```sh
python3 tools/flashing/flash_stlink --build --sn 003B00273234511537333934
```

合格基準:

- [ ] `SUCCESS` が出る
- [ ] 書き込み後にsoftware resetされる

---

## 3. UART接続確認

ポート候補確認:

```sh
python3 tools/logging/serial_terminal.py --list
```

端末を開く:

```sh
python3 tools/logging/serial_terminal.py --port /dev/cu.usbmodem112202
```

合格基準:

- [ ] 起動ログが読める
- [ ] `[NVM-TEST] UART command mode ready` が出る
- [ ] helpに `P=PUSH increment, E=FR enter` が出る

---

## 4. 非走行の健全性確認

### 4.1 NVM状態

操作UIで確認する場合:

```text
PPPPPPPPPEPPPPPPPE
```

意味:

- `P` × 9: mode9へ移動
- `E`: mode9へ入る
- `P` × 7: case7へ移動
- `E`: NVM status実行

合格基準:

- [ ] `[NVM-STATUS] distance=OK` または意図した状態が表示される
- [ ] `[NVM-STATUS] sensor=OK` または意図した状態が表示される
- [ ] `[NVM-STATUS] maze=OK` または意図した状態が表示される
- [ ] `[NVM-STATUS] trace=OK` または意図した状態が表示される

### 4.2 identity確認

ST-LINK reset後に以下を送る:

```text
PPPPPPPPPEPPPPPPPPE
```

合格基準:

- [ ] `[IDENTITY] status=OK` が出る
- [ ] family / board / rev / unit が意図した値である

### 4.3 sensor params確認

ST-LINK reset後に以下を送る:

```text
PPPPPPPPPEPPPPPPPPPE
```

合格基準:

- [ ] `[SENSOR-PARAM] source=NVM` または `source=default` が出る
- [ ] `save is intentionally not performed` が出る
- [ ] この操作では保存・上書きが行われない

---

## 5. センサ・入力確認

### 5.1 壁センサ1回測定

```text
w
```

合格基準:

- [ ] `[HW-TEST][Wall] PASS(measure done)` が出る
- [ ] 壁なしで `front=0 right=0 left=0` になる
- [ ] `sat=0` になる

### 5.2 壁切れモニタ

```text
W
```

合格基準:

- [ ] 壁なし固定で `endR=0 endL=0` になる
- [ ] 手や壁を近づけて外す場合、対象側の `endR` または `endL` が1になる

### 5.3 IMU疎通

```text
i
```

合格基準:

- [ ] `WHO_AM_I=0x6B expected=0x6B => PASS` が出る

### 5.4 IMU手動角度

```text
I
```

手順:

1. 開始直後は静止させる。
2. 手で機体を少し左右に回す。
3. CCW/左回しが正、CW/右回しが負になることを確認する。

合格基準:

- [ ] offset samplingが完了する
- [ ] 左回しで角度が正方向へ増える
- [ ] 右回しで角度が負方向へ増える

### 5.5 IMU加速度

```text
c
```

合格基準:

- [ ] 静止時の値が極端に暴れない
- [ ] 前後に軽く動かすと前進軸の値が反応する

### 5.6 エンコーダ

```text
e
```

手順:

1. 左右車輪を手で前進方向へ回す。
2. 左右車輪を手で逆方向へ回す。

合格基準:

- [ ] カウントが変化する
- [ ] 前進方向と逆方向で符号が反転する

---

## 6. 浮かせ固定状態でのモータ系最小確認

この章はモータが回る。必ず機体を浮かせて固定してから行う。

### 6.1 片側モータ・エンコーダ方向確認

UARTで順番に実行する:

```text
6
7
8
9
```

意味:

- `6`: 左モータ forward
- `7`: 右モータ forward
- `8`: 左モータ reverse
- `9`: 右モータ reverse

合格基準:

- [ ] `6` で左モータだけ前進方向に回る
- [ ] `7` で右モータだけ前進方向に回る
- [ ] `8` で左モータだけ逆方向に回る
- [ ] `9` で右モータだけ逆方向に回る
- [ ] 異音・過熱・停止不能がない

---

## 7. ログ導線確認

### 7.1 idleログ

```sh
python3 tools/logging/serial_capture_csv.py --show-noncsv --send q,x,V --send-interval-ms 2500 tools/logging/logs /dev/cu.usbmodem112202 115200
python3 tools/logging/analyze_trace_csv.py tools/logging/logs --expect x
```

合格基準:

- [ ] `q` でformat成功
- [ ] `x` でidle sessionが完了
- [ ] `V` でCSVが保存される
- [ ] analyzerが `--expect x` で成功する

### 7.2 短時間モータログ

この確認はモータが回る。必ず浮かせ固定状態で行う。

```sh
python3 tools/logging/serial_capture_csv.py --show-noncsv --send q,y,V --send-interval-ms 5000 tools/logging/logs /dev/cu.usbmodem112202 115200
python3 tools/logging/analyze_trace_csv.py tools/logging/logs --expect y
```

合格基準:

- [ ] `y` が完了する
- [ ] CSVに `motor_forward` / `motor_reverse` / `coast` 相当の位相が出る
- [ ] analyzerが `--expect y` で成功する
- [ ] 停止不能・異音・過熱がない

---

## 8. 調整開始の入口

上記が通ったら、制御ゲイン・モーション調整は次の順で始める。

1. 浮かせ固定で `1` S3直進ログ取得
2. 浮かせ固定で `3` R90ログ取得
3. 浮かせ固定で `4` L90ログ取得
4. 床上低速S3直進
5. 床上低速R90/L90
6. S3+R90+S3
7. 壁なし直進で壁制御が暴れないことを確認
8. 壁あり低速直進で壁制御方向を確認

ログ取得例:

```sh
python3 tools/logging/serial_capture_csv.py --show-noncsv --send q,1,V --send-interval-ms 8000 tools/logging/logs /dev/cu.usbmodem112202 115200
python3 tools/logging/analyze_trace_csv.py tools/logging/logs
```

旋回ログ解析例:

```sh
python3 tools/logging/serial_capture_csv.py --show-noncsv --send q,3,V --send-interval-ms 8000 tools/logging/logs /dev/cu.usbmodem112202 115200
python3 tools/logging/analyze_turn_csv.py tools/logging/logs --target-angle -90
```

---

## 9. 中止条件

以下が出た場合は調整を止め、ログを保存して原因確認する。

- [ ] モータが止まらない
- [ ] 片側だけ異常に強く回る
- [ ] エンコーダ符号が期待と逆
- [ ] IMU角度符号が期待と逆
- [ ] `NVM_STATUS` 異常が出る
- [ ] trace log append失敗が出る
- [ ] `abort(...)` が繰り返し出る
- [ ] 発熱・異音・焦げ臭さがある

---

## 10. 記録テンプレート

```text
[F413 Control Tuning Precheck]
Date:
FW git sha:
Machine:
Battery voltage:
Fixture: lifted/fixed

[Build]
- Debug-stm32f413: pass / fail
- Debug-stm32f405: pass / fail

[Flash]
- ST-LINK flash: pass / fail

[Non-motor]
- NVM status: pass / fail
- identity: pass / fail
- sensor params: pass / fail
- wall w: pass / fail
- wall-end W: pass / fail
- IMU i: pass / fail
- IMU angle I: pass / fail
- IMU accel c: pass / fail
- encoder e: pass / fail

[Motor lifted]
- 6 L fwd: pass / fail
- 7 R fwd: pass / fail
- 8 L rev: pass / fail
- 9 R rev: pass / fail
- q,x,V analyze: pass / fail
- q,y,V analyze: pass / fail

[Decision]
- Ready for gain tuning: yes / no
- Notes:
```
