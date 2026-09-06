# F413 exploration CPU benchmark

モータを動かさず、接続した STM32F413 の SRAM 上で計算時間を測る。
アプリケーション Flash、identity、FRAM、保存迷路には書き込まない。
実行後はソフトウェアリセットで元のアプリケーションに戻るため、
保存されていない RAM のセッション状態は失われる。

```sh
tools/hil/exploration_bench/build.sh smoke
python3 tools/hil/exploration_bench/run_ram.py \
  --elf build/exploration_mcu/smoke.elf \
  --serial <ST-LINK serial> --output build/exploration_mcu/smoke-result
# 上記はビルドと入力の確認のみ。実行には --run を付ける。
```

対象は HSI → PLL (M=16, N=200, P=2)、AHB /1 の 100 MHz F413。
起動前にモータ左右とファンの PWM compare=0、PB2 standby=0 を SWD で確認し、
一致しなければ機体の状態を変更せず終了する。小さな SRAM の第一段プログラムが
割り込み・DMA・タイマ等を停止してから計測プログラムをロードする。
CubeProgrammer への書き込み先は SRAM のみ。周辺レジスタの停止操作は
レビュー可能な `prepare.c` に限定する。

計測は DWT CYCCNT を使用する。通常ファームウェアの割り込み、センサ処理、
Flash 命令フェッチの影響を含まないため、走行中の CPU 余裕や制御周期ジッタの
保証には使わない。`result.json` に ELF SHA-256、クロックレジスタ、実測値、
Flash 全 1.5 MiB の実行前後 SHA-256 を記録し、一致を確認する。
失敗時もリセットを試み、Flash の確認結果を残す。

`policy` は C 探索ポリシー単体を、完全な依存壁を持つ小さな仮想経路で測る。
その結果を実時間経路プランナの速度と混同しない。
`planner` は大会迷路から生成した部分地図の楽観・保守投影で実プランナを測る。
入力生成は `tools/exploration_sim/mcu_benchmark_cases.py` を使用する。
32×32 の SRAM 計測では第4引数でケースを1つ選び、定数データの容量を抑える。
すべての計測プログラムは実機への走行指示を持たない。

```sh
tools/hil/exploration_bench/build.sh policy build/exploration_mcu/policy.elf
tools/hil/exploration_bench/build.sh adapter build/exploration_mcu/adapter.elf
```

`adapter` は実際の `f413_exploration.c` を同じ `-O2` で組み込み、実 oracle・
探索ポリシー・到着予測を結合する。HAL/CMSIS は型宣言のためにのみ参照し、
モータ・HAL 実行関数・NVM はリンクしない。置換するのはログ出力、メモリ貸出、
迷路/姿勢のグローバル変数だけである。現行 `f413_preorder` の 16×16、
goal=(1,0) と、既知の開放迷路を使う。

アダプタ計測の row 4000 は初期化、4001 は観測コピーと足立法採用、
4002 は予測ジョブ準備、4003 は実 oracle を含むポーリング、4004 は到着時の
完了証明適用、4005 は加速中の完了延期と直進減速、4006 は壁訂正による中止、
4007 はメモリ解放、4008 は容量不足、4009 は共有壁矛盾による中止を検査する。
4010 は実際のゴール到達を記録してから再開始で位置が変わるケース、4011 は
同じ観測状態で計算が間に合わないときの足立法保持、4012 は保持中でも厳密な
完了証明を採用できることを検査する。
status の bit31 が立てば期待結果と一致しない。4001/4002/4004 の合計時間は
16 回分、それ以外は記録された slices の回数分である。

SRAM 実行では通常 Flash に置くコードとテーブルも RAM を消費するため、
アダプタ計測だけ貸出領域を `NF_MCU_SLALOM_WORKSPACE_BYTES - 8 KiB`
（現行 192 KiB）に制限する。16×16 計算器が必要とする領域を上回り、
計算器が実際に初期化・参照するバイト数は同じである。実ファームウェアの
200 KiB 貸出を変更しない。実際の貸出容量は mailbox の reserved に記録する。

## Trace lease exclusion test

`build.sh trace` compiles the actual trace-buffer lease implementation. The only
NVM symbols in that ELF are local counter stubs: no storage driver is linked.
It checks eight conditions, including exclusive/aligned borrowing, rejection
while capture is enabled, rejection of capture before any format/header call
while borrowed, wrong-pointer release, and reuse after release/abort. A passing
mailbox row 5000 has status 0 and checksum 255. This is an exclusion test, not a
persistent trace/NVM integration test.
