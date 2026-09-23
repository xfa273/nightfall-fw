# WeAct-Adapter-5V

WeAct Studio MiniDebugger から `mini_r2_0` / F413 機体へ SWD、SWO、UART、5 V を接続する変換基板です。
既存の `STLink-Adapter` を基に、デバッガの 5 V 出力を手動で遮断できるスライドスイッチ `SW1` を追加しています。

## 電源スイッチ

- `5V` 側: MiniDebugger の 5 V を機体コネクタ K1 pin 1 へ接続します。
- `OFF` 側: MiniDebugger の 5 V を機体から切り離します。SWD/UART/GND は接続されたままです。
- `SW1` は UART-Adapter と同じ E-Switch `EG1218`（SPDT, On-On）を使用します。中央端子を機体 5 V、`5V` 側端子をデバッガ 5 V、`OFF` 側端子を未接続としています。

> [!CAUTION]
> この基板はダイオードORやロードスイッチによる自動逆流防止回路ではありません。バッテリを接続する前に必ず `OFF` にしてください。`5V` のままバッテリとMiniDebuggerを同時接続しないでください。

`EG1218` の定格は 200 mA / 30 VDC です。このアダプタからの給電はファームウェア書き込み、ログ確認、ブザー等の低消費電力なベンチ作業向けです。モータやファンを動かす用途には使用しません。

## MiniDebugger側の配線

| MiniDebugger信号 | J2 pin | アダプタ | 機体側 |
| --- | --- | --- | --- |
| `5V` | 1 | TP1 pad 1 | K1 pin 1 `+5V`（SW1経由） |
| `TXD` | 4 | TP1 pad 2 | K1 pin 9 `USART1_RX` |
| `RXD` | 3 | TP1 pad 3 | K1 pin 8 `USART1_TX` |
| `GND` | 2 | TP1 pad 4 | K1 pin 2 `GND` |
| `SWDIO` | 9 | TP2 pad 1 | K1 pin 3 `SWDIO` |
| `SWCLK` | 8 | TP2 pad 2 | K1 pin 4 `SWCLK` |
| `SWO` | 6 | TP2 pad 3 | K1 pin 10 `SWO` |

信号名とMiniDebuggerのJ2ピン番号を基準に配線し、コネクタを見たときの物理的な並びだけで判断しないでください。TXD/RXDやSWD信号を5 V/GNDへ誤接続すると、MiniDebuggerのUSB認識が消えるほどの過電流になる場合があります。

MiniDebugger の `3V3`（J2 pin 10）と `NRST`（J2 pin 5）はこのアダプタでは未接続です。余った線は互いに接触しないよう個別に絶縁します。MiniDebugger の2本目の `GND`（J2 pin 7）は未使用でも構いません。

## 初回組立後の確認

最初に機体を接続せず、テスターで次を確認します。

1. `OFF` 側で TP1 pad 1 と K1 pin 1 が導通しないこと。
2. `5V` 側で TP1 pad 1 と K1 pin 1 が導通すること。
3. `5V` 側、`OFF` 側のどちらでも 5 V と GND が短絡していないこと。
4. MiniDebuggerだけを接続し、`5V` 側で K1 pin 1 が約 5 V、`OFF` 側で無給電になること。
5. すべての電源を外して機体側ケーブルを接続し、TP1 pad 4－機体GNDがほぼ0 Ωであること。
6. 同じ状態でTP1 pad 2－機体K1 pin 9、TP2 pad 3－機体K1 pin 10がそれぞれ導通し、5 V/GNDとは短絡していないこと。
7. 最後に機体を接続し、モータ・ファンを動かさずSWD、UART、ブザーを確認すること。

## 検証結果

KiCad 10.0.3 のCLIで確認しています。

- PCB DRC: errors 0、unconnected 0
- DRC warnings: 26（元のSTLink-Adapterと同数。K1の狭ピッチ端子に対する既知のソルダーマスク警告）
- ERC: errors 3、warnings 6（元設計の電源駆動未定義3件とフットプリントライブラリ未登録等。今回追加した配線の未接続・短絡エラーなし）
- 上面・下面3Dレンダリング: `preview/`
- ERC/DRCレポート: `reports/`

### 実機検証（2026-07-27）

初回の外部配線誤りを修正した後、製造した基板とmini_r2_0 / STM32F413機体で次を確認しました。

- `5V` 側で意図どおり5 Vを給電し、`OFF` 側でMiniDebuggerの5 V出力を機体から切り離せること。
- macOSからWeAct MiniDebuggerのST-LINK/V2-1（V2J43M28）を認識できること。
- SWD 4000 kHzでターゲット電圧3.22 V、Device ID `0x463`（STM32F413/F423）を読み取り、software resetが成功すること。
- Virtual COM Portを921600 baud / 8N1で開き、安全なUART診断 `i` に対して `[HW-TEST][IMU] WHO_AM_I=0x6B expected=0x6B => PASS` が返ること。

この確認ではモータ・ファンを動作させず、ファームウェア書き込みおよびNVM書き込みも実施していません。

## 発注データ

`fabrication/WeAct-Adapter-5V-gerbers.zip` を基板メーカーへ投入します。

推奨指定:

- 2 layer
- FR-4, 1.6 mm
- 1 oz copper
- HASL lead-free（または通常HASL）
- Solder mask / silkscreen: 任意色

`fabrication/generate_fab.sh` でGerber、Excellon drill、ZIPを再生成できます。
