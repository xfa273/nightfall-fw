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

## C配列形式を読む

KeriLabビューアなどでC配列形式に変換したファイルも読み込めます。

```sh
tools/solver_host/run_solver_host.sh --maze-c-array path/to/maze.c --origin top-left --mode 2 --case 1
```

## 詳細表示

通常は `solver.c` 内部の巨大なASCII迷路表示を抑制し、`path[]` の要約だけ出します。内部表示も見たい場合は `--verbose-solver` を付けます。

```sh
tools/solver_host/run_solver_host.sh --maze path/to/maze.maze --verbose-solver
```
