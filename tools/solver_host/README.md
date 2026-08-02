# solver_host

F413実機で迷路探索を行う前に、PC上で共通 `solver_build_path()` の経路導出を確認するためのホストツールです。

## 内蔵サンプルで確認

```sh
tools/solver_host/run_solver_host.sh
```

## KeriLab `.maze` 形式を直接読む

```sh
tools/solver_host/run_solver_host.sh \
  --maze build/solver_host/kerilab_data/762ed2b68735ea29148c6a1251a90ed0651ff26b/32MM2023HX.maze \
  --mode 2 --case 2 --time-plan --assert-valid
```

`--time-plan` は実機コードを変更せず、F413の指定mode/caseの速度・加速度・
小回り90／大回り90・180パラメータを使って、直交動作だけの時間最短経路を
Dijkstraで求めます。評価値は停止完了ではなく、いずれかのゴール区画へ最初に
入る境界の通過時刻です。ゴール後の開放直線を停止軌道として延長するため、
`goal_entry_us` と `stop_us` は分けて表示します。

`geometry=` 行には、raised-cosine角速度と一定並進速度から2次元積分したターンの
公称変位、`path[]` が要求する論理アンカー、および両者の残差を表示します。
大回り中のゴール進入時刻もこの2次元軌跡とゴール境界の最後の交点から求めます。
残差は現在のパラメータを机上で評価するための指標であり、この段階では自動的に
ターンを無効化する閾値にはしていません。

ターン集合は次のように切り替えられます。

```sh
# case 1は小回りのみ、case 2以降は小回り＋大回り（現行profile準拠）
tools/solver_host/run_solver_host.sh --maze path/to/maze.maze \
  --mode 2 --case 2 --time-plan --turn-set profile --assert-valid

# 比較用の明示指定
tools/solver_host/run_solver_host.sh --maze path/to/maze.maze \
  --mode 2 --case 2 --time-plan --turn-set small
tools/solver_host/run_solver_host.sh --maze path/to/maze.maze \
  --mode 2 --case 2 --time-plan --turn-set all
```

時間プランナの `.maze` readerはファイルのSと全Gを使用し、最大32x32まで読む
厳格版です。未知壁 `.` は壁ありへ読み替えずエラーにします。

## 斜め対応・時間最短経路を確認する

`--slalom-time-plan` は直交3種に45度in/out、V90、135度in/outを加えた
時間最短経路を導出し、half-grid anchor・8方位・境界速度classを持つ型付き動作列へ
変換します。F413は調整済みのmode2をprimaryとし、F405 mini mode2〜5を比較に
使用します。caseは斜め直線値を持つ8または9だけを受理します。

```sh
tools/solver_host/run_solver_host.sh \
  --maze path/to/maze.maze \
  --slalom-time-plan \
  --slalom-profile f413-preorder-mode2 \
  --case 8 --compare-orthogonal --assert-valid

tools/solver_host/run_solver_host.sh \
  --maze path/to/maze.maze \
  --slalom-time-plan \
  --slalom-profile f405-mini-mode4 \
  --case 9 --compare-orthogonal --assert-valid --summary-only
```

`--compare-orthogonal` は同じconfigの斜め5種だけを無効にした候補と比較し、斜めを
許した結果が遅くならないことを検査します。評価値は従来と同じく最初のG区画への
進入時刻です。斜めturnはPC専用exact-closure仮値であり、firmware parameterは
変更しません。

過去大会24迷路×5profile×case 8/9の240構成を一括確認する場合:

```sh
tools/solver_host/run_slalom_kerilab_matrix.sh
```

結果は実行ごとの一時ファイルを経て、全gate通過時だけ
`build/solver_host/slalom_kerilab_matrix.tsv` へ原子的に保存されます。
`legacy_geometry` は現runnerへ論理距離を写せるかのgateで、
`terminal-diagonal-unsupported` は現runnerの固定直交停止では斜め終端を表せない
ことを示します。現runnerは直線codeごとに速度を再計画するため、互換経路でも
`legacy_time_equivalent=no`です。

固定データcommitでの基準結果は240/240成功、斜め採用240、直交限定に対する
厳密短縮240、`legacy_geometry=ok` 81、斜め終端非対応159です。

## KeriLab過去大会迷路を取得して一括確認

過去大会迷路は
[`kerikun11/micromouse-maze-data`](https://github.com/kerikun11/micromouse-maze-data)
のMITライセンスデータを利用します。再現性のため、取得元をcommit
`762ed2b68735ea29148c6a1251a90ed0651ff26b` に固定し、取得したファイルは
`build/solver_host/kerilab_data/<commit>/` にキャッシュします。

通常は16x16と32x32から、単一・2x2・3x3ゴールを含む代表5迷路を実行します。
各迷路について時間ベースの直交経路を導出し、`--assert-valid` で動作文法と
論理壁通過、ゴール進入を検証します。ターンの掃引領域と機体クリアランスの検証は
まだ含みません。

```sh
tools/solver_host/run_kerilab_samples.sh
```

16x16の9迷路と32x32の15迷路、合計24件を実行する場合:

```sh
ALL=1 tools/solver_host/run_kerilab_samples.sh
```

固定commitに存在する個別の迷路を指定する場合:

```sh
tools/solver_host/run_kerilab_samples.sh data/16MM2019CX.maze data/32MM2023HX.maze
```

F413の大会用32x32設定と同じ配列境界で確認する場合は、`MAZE_SIZE` を
コンパイル時に上書きします。

```sh
CPPFLAGS=-DMAZE_SIZE=32 tools/solver_host/run_solver_host.sh \
  --maze build/solver_host/kerilab_data/32MM2023HX.maze --mode 2 --case 1
```

`MODE` / `CASE` / `TURN_SET` 環境変数でF413側の最短走行モード・ケースと
ターン集合を変更できます。

```sh
MODE=2 CASE=8 TURN_SET=all tools/solver_host/run_kerilab_samples.sh
```

既定値は `MODE=2 CASE=2` で、小回りと大回りの両方を評価します。

## ホスト単体試験

```sh
tools/solver_host/run_route_motion_tests.sh
tools/solver_host/run_route_planner_tests.sh
tools/solver_host/run_route_clearance_tests.sh
tools/solver_host/run_slalom_profile_audit.sh
tools/solver_host/run_slalom_time_planner_tests.sh
tools/solver_host/run_legacy_path_codec_tests.sh
tools/solver_host/run_path_pipeline_tests.sh
tools/solver_host/run_slalom_plan_legacy_codec_tests.sh
tools/solver_host/run_slalom_time_plan_host_tests.sh
tools/solver_host/run_legacy_path_validator_tests.sh
tools/solver_host/run_solver_host_cli_tests.sh
```

既存の斜め経路→動作変換は共通のtransactional codecへ置換され、
`--assert-valid` で直交／斜め状態遷移を検証します。不完全な小回り列は無理に
斜め入りへ変換せず、論理方位とhalf-grid終点が一致する完全な列だけを変換します。

```sh
tools/solver_host/run_solver_host.sh --mode 2 --case 8 --assert-valid
```

設計と距離所有規則は
[`docs/ai/ORTHOGONAL_TIME_PLANNER.md`](../../docs/ai/ORTHOGONAL_TIME_PLANNER.md)
および
[`docs/ai/SLALOM_TIME_PLANNER.md`](../../docs/ai/SLALOM_TIME_PLANNER.md)
を参照してください。

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
この旧 `solver_build_path()` 用シミュレータはfirmwareの `MAZE_SIZE=16` 固定です。
32x32迷路は黙って切り詰めずエラーにし、32x32の検証には `--time-plan` を使います。

```sh
tools/solver_host/run_solver_host.sh --explore-sim
tools/solver_host/run_solver_host.sh \
  --maze build/solver_host/kerilab_data/762ed2b68735ea29148c6a1251a90ed0651ff26b/16MM2020CX.maze \
  --explore-sim --max-steps 2048
```

各ステップの位置・相対壁・mapセル値を表示する場合:

```sh
tools/solver_host/run_solver_host.sh --maze path/to/maze.maze --explore-sim --explore-verbose
```
