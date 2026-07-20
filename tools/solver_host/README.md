# solver_host

F413実機で迷路探索を行う前に、PC上で共通 `solver_build_path()` の経路導出を確認するためのホストツールです。

## 内蔵サンプルで確認

```sh
tools/solver_host/run_solver_host.sh
```

## KeriLab `.maze` 形式を直接読む

```sh
tools/solver_host/run_solver_host.sh --maze build/solver_host/kerilab_data/32MM2023HX.maze --mode 2 --case 1
```

## KeriLab代表32x32サンプルを取得して一括確認

```sh
tools/solver_host/run_kerilab_samples.sh
```

個別の迷路を指定する場合:

```sh
tools/solver_host/run_kerilab_samples.sh data/32MM2023HX.maze data/32_test_01.maze
```

`MODE` / `CASE` 環境変数でF413側の最短走行モード・ケースを変更できます。

```sh
MODE=2 CASE=8 tools/solver_host/run_kerilab_samples.sh
```

F413の大会用32x32設定と同じ配列境界で確認する場合は、`MAZE_SIZE` を
コンパイル時に上書きします。

```sh
CPPFLAGS=-DMAZE_SIZE=32 tools/solver_host/run_solver_host.sh \
  --maze build/solver_host/kerilab_data/32MM2023HX.maze --mode 2 --case 1
```

## C配列形式を読む

KeriLabビューアなどでC配列形式に変換したファイルも読み込めます。

```sh
tools/solver_host/run_solver_host.sh --maze-c-array path/to/maze.c --origin top-left --mode 2 --case 1
```

## F413 UART `@` の探索map dumpを読む

F413実機の `[SEARCH-DUMP]` ログを読み込み、保存済み探索mapから `solver_build_path()` が最短経路を作れるか確認できます。`map[y][x] >> 4` を壁情報として使うため、未探索セルが `0xF0` のままの空mapでは最短経路生成に失敗するのが正常です。

```sh
python3 tools/logging/serial_capture_csv.py --show-noncsv --send @ tools/logging/logs /dev/cu.usbmodem112202 115200 > /tmp/search_dump.log
tools/solver_host/run_solver_host.sh --search-dump /tmp/search_dump.log --mode 2 --case 1
```

## 詳細表示

通常は `solver.c` 内部の巨大なASCII迷路表示を抑制し、`path[]` の要約だけ出します。内部表示も見たい場合は `--verbose-solver` を付けます。

```sh
tools/solver_host/run_solver_host.sh --maze path/to/maze.maze --verbose-solver
```

## 仮想壁入力による探索シミュレーション

F413実機を迷路内で走らせずに、PC上で仮想迷路の壁を1区画ずつセンサ入力として与え、`map[][]` 更新と次方向決定を確認できます。

```sh
tools/solver_host/run_solver_host.sh --explore-sim
tools/solver_host/run_solver_host.sh --maze build/solver_host/kerilab_data/32MM2023HX.maze --explore-sim --max-steps 2048
```

各ステップの位置・相対壁・mapセル値を表示する場合:

```sh
tools/solver_host/run_solver_host.sh --maze path/to/maze.maze --explore-sim --explore-verbose
```
