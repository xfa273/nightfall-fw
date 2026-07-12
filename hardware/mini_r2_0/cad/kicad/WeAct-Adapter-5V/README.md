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

| MiniDebugger | アダプタ | 機体側 |
| --- | --- | --- |
| `5V` | TP1 pad 1 | K1 pin 1 `+5V`（SW1経由） |
| `TXD` | TP1 pad 2 | K1 pin 9 `USART1_RX` |
| `RXD` | TP1 pad 3 | K1 pin 8 `USART1_TX` |
| `GND` | TP1 pad 4 | K1 pin 2 `GND` |
| `SWDIO` | TP2 pad 1 | K1 pin 3 `SWDIO` |
| `SWCLK` | TP2 pad 2 | K1 pin 4 `SWCLK` |
| `SWO` | TP2 pad 3 | K1 pin 10 `SWO` |

MiniDebugger の `3V3` と `NRST` はこのアダプタでは未接続です。余った線は互いに接触しないよう個別に絶縁します。MiniDebugger の2本目の `GND` は未使用でも構いません。

## 初回組立後の確認

機体を接続する前にテスターで次を確認します。

1. `OFF` 側で TP1 pad 1 と K1 pin 1 が導通しないこと。
2. `5V` 側で TP1 pad 1 と K1 pin 1 が導通すること。
3. `5V` 側、`OFF` 側のどちらでも 5 V と GND が短絡していないこと。
4. MiniDebuggerだけを接続し、`5V` 側で K1 pin 1 が約 5 V、`OFF` 側で無給電になること。
5. 最後に機体を接続し、モータ・ファンを動かさず書き込み、UART、ブザーを確認すること。

## 検証結果

KiCad 10.0.3 のCLIで確認しています。

- PCB DRC: errors 0、unconnected 0
- DRC warnings: 26（元のSTLink-Adapterと同数。K1の狭ピッチ端子に対する既知のソルダーマスク警告）
- ERC: errors 3、warnings 6（元設計の電源駆動未定義3件とフットプリントライブラリ未登録等。今回追加した配線の未接続・短絡エラーなし）
- 上面・下面3Dレンダリング: `preview/`
- ERC/DRCレポート: `reports/`

## 発注データ

`fabrication/WeAct-Adapter-5V-gerbers.zip` を基板メーカーへ投入します。

推奨指定:

- 2 layer
- FR-4, 1.6 mm
- 1 oz copper
- HASL lead-free（または通常HASL）
- Solder mask / silkscreen: 任意色

`fabrication/generate_fab.sh` でGerber、Excellon drill、ZIPを再生成できます。
