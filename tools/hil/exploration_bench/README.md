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
