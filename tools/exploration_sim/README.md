# exploration_sim - 探索アルゴリズム机上検討ツール

マイクロマウスの**探索走行**を、実機なしで「どの順に探索し、どれだけの時間が
かかるか」を調べる Python ツール（標準ライブラリのみ・依存なし）。

## セットアップ

```sh
git clone --depth 1 https://github.com/kerikun11/micromouse-maze-data.git \
    ../maze-data                      # 過去大会の迷路 80 件（./mazes 代替）
export NIGHTFALL_MAZE_DATA=../maze-data/data
```

迷路データはこの順番で探索する:
`--maze-dir` / `$NIGHTFALL_MAZE_DATA` / `<repo>/mazes` /
`~/workspace/micromouse/maze-data[/data]` / `~/workspace/micromouse/micromouse-maze-data/data`

## 使い方

```sh
cd tools/exploration_sim

# 1 迷路を詳しく（探索順マップ・タイムライン・HTML レポート）
./run_exploration_sim.py --maze ../../../../maze-data/data/16MM2019CX.maze \
    --print-order --out ../../out/exploration_sim

# 全日本決勝迷路で戦略・センサ・初期化の方針比較
./run_exploration_sim.py --maze-dir ../../../maze-data/data \
    --maze-glob '16MM20[12][0-9]CX.maze' \
    --strategy fw,dijkstra,gain --sensing ideal4_ahead --compare

# 整合性の自己テスト（迷路 80 件の構文検査＋センサ変換の仕様検査）
./run_exploration_sim.py --selftest
```

## 再現しているファーム挙動 (`platform/stm32f405/Core/Src/search.c`)

| 実機 | シミュレータ |
|---|---|
| `get_wall_info()` F/R/L の 3 センサ（後方なし） | `--sensing fw` |
| `write_map()` `m_temp = (wall_info >> dir) & 0x0f`、現マス**上書き**、4 近傍へ壁あり/なしを両方向に伝搬 | `--sensing fw --map-write overwrite --propagate fw` |
| `make_smap()` 全未探索セルを 0 とする多起点 BFS（1 セル=1 コスト） | `--strategy fw` |
| `make_route()` 直進→右→左→後退 の優先順で厳密降下 | `--strategy fw` |
| `conf_route()` 1 セルごとに再計画 | 毎ステップ再計算 |
| `map_Init()` 下位 4bit=壁なし で初期化 | `--unknown-walls open` |

## 切り替えて比較できるもの

| フラグ | 意味 |
|---|---|
| `--strategy fw\|dijkstra\|gain` | 現行 BFS / 時間コスト Dijkstra / ゴール利得ベース目標選択 |
| `--sensing fw\|fw_fixed\|ideal3\|ideal4\|ideal4_ahead` | 現行変換 / 回転修正 / 3 センサ完動 / 4 センサ / 4 センサ＋先読み |
| `--map-write overwrite\|accumulate` | 現マスの壁を上書き（現行）/ OR 蓄積 |
| `--propagate fw\|safe` | 「壁なし」を近傍へ伝搬（現行）/ 壁ありのみ |
| `--unknown-walls open\|closed` | 未観測を開（現行 `map_Init`）/ 閉 で初期化 |
| `--goal-policy full_first\|dash_when_reachable` | 全探索優先（現行相当）/ 到達可能になったら即ゴール |

時間モデルは `params/<board>/search_run_params_split.c` 等を実測値として読み、
直線は台形積分（区間をまたいで速度を引き継ぐ）、旋回は角速度台形＋安定化待ち。
`--time-model cells` で 1 セル=0.16s の簡易モデルにも切り替えられる。

## 出力

`--out` に `summary.csv`（比較表）、`<maze>__<strategy>....timeline.csv`（1 手ごと）、
`.json`（メトリクス）、`.html`（迷路＋探索順＋ラン別の自己完結レポート）を書きます。

指標: `t_first_goal`（初ゴールまでの合計走行時間）/ `t_full_known`（全セル既知に
なった時刻）/ `best_shortest` / `cells_moved` / `turns_90` / `turns_180` /
`phantom_hits`（認識上は開口・実際は壁への進入）/ `wrong_walls` `missing_walls`
（最終認識マップと実地図の差分）。

## マトリクス比較（sweep.py）

```sh
# 戦略比較（初ゴールまでの時間の平均を出す）
./sweep.py --mazes 16MM2012CX,16MM2017CX,16MM2019CX,16MM2020CX \
    --strategy fw,dijkstra,gain --sensing ideal4_ahead --pivot t_first_goal

# センサ変換・マップ書き込み方針の比較
./sweep.py --mazes 16MM2012CX,16MM2017CX --strategy fw \
    --sensing fw,fw_fixed,ideal4_ahead --map-write overwrite,accumulate \
    --pivot phantom_hits
```

## 指標の読み方

- `t_first_goal` … 最初のゴール到達までの**累計走行時間**。探索性能の主指標。
- `t_full_known` … 全セルに一度も入らないと埋まらない（ゴール到達でランが
  終わるため、通常は空欄になる。踏破率は `known%` を見る）。
- `known%` … **踏破したセルの割合**（着座したことがあるか）。壁情報の確定とは
  別物なので、地図の正確性は `wrong_walls` / `missing_walls` / `phantom_hits` で見る。
- `phantom_hits` … 認識上は開口・実際は壁、という状態への進入回数。
  0 なら「走る前に壁を読めている」ことを意味する。

## 可視化 UI（viewer.py）

```sh
./viewer.py --maze 16MM2017CX            # out/exploration_sim/viewer_16MM2017CX.html
./viewer.py --all-classic                # 全日本決勝迷路分をまとめて生成
open ../../out/exploration_sim/viewer_16MM2017CX.html
```

左ペイン＝現状ソフト、右ペイン＝改善版（プリセットはドロップダウンで差し替え可）を
**同じ時間軸で同期再生**します。壁は 4 状態に色分けし、「機体がどう進むか」と
「未知の壁が既知に変わっていく（変わらない）様子」をそのまま見られます。

| 色 | 意味 |
|---|---|
| 白の太線 | 既知の壁（正しい） |
| 赤の太線 | 誤って「壁あり」と信じている（実在しない壁） |
| 青の破線 | 実在するが見落としている壁 |
| 灰色の細線 | 真の迷路（参考表示、トグル可） |
| 水色 | 走行軌跡／赤リング = phantom（未知の壁への進入） |

操作: `space` 再生/停止、`←→` ±0.2 s、`↑↓` 再生速度、スライダーでシーク、
`次の phantom` ボタンで問題箇所へジャンプ。`訪問順` / `軌跡` / `真の迷路` はトグル。
外部依存ゼロの単一 HTML（canvas + JS）なので、そのまま共有できます。

### ラン進行（機体実装と同じ流れ）

- **ラン 1 = 全面探索**: ゴール区画を通過しても止まりません（機体と同じ。
  `MF.FLAG.GOALED` を立てて地図保存の機会を記録するだけ）。未探索へ向かう経路が
  無くなるまで走ります。
- **ラン 2 以降 = ゴールモード**: 開始位置がゴール側なら**スタートを**、そうでなければ
  ゴールを目的地にします（`g_goal_is_start` 相当）。
- ランの切れ目は係員の再配置として `action='P'` フレームで表します（機体は破線枠の
  `P` 表示、軌跡は分断、タイムラインに白線）。壁を跨いだテレポートは見えません。
- 比較用に `end_run_at_goal="always"`（ルール上ゴールで停止する運用）も選べます
  （`run_preset(maze, motion, key, {"end_run_at_goal": "always"})`）。

### 入っているプリセット（`trace_sim.PRESETS`）

| key | 中身 |
|---|---|
| `current` | 現状同等（`(wall_info >> dir)` 変換／現マス上書き／壁なしも伝搬／最近傍未探索を目標） |
| `sensor_only` | 変換だけ正しい絶対方位に修正（未観測は従来のまま「壁なし」扱い） |
| `write_only` | 地図更新だけ修正（期待値と一致するスロットのみ既知扱い、OR 蓄積） |
| `fixed_map` | 変換・地図更新を両方修正、目標選択は現状のまま |
| `fixed_gain` | `fixed_map` ＋ ゴール利得ベースの目標選択 |
| `topmouse` | 上位機相当（4 センタ＋先読み）＋ 利得目標 |

## 検証用ハーネス（Node）

JS の描画が時間軸に反応しているか、ブラウザを開かずに確認できる。

```sh
python3 viewer.py --maze 16MM2017CX         # まず生成
python3 - <<'PY'                            # v2.js を作る（boot() を無効化して module 化）
import re
h=open("../out/exploration_sim/viewer_16MM2017CX.html",encoding="utf-8").read()
js=re.search(r"<script>\n(.*)</script>",h,re.S).group(1).replace("boot();","/*off*/",1)
open("/tmp/v2.js","w").write(js+"\nmodule.exports={get state(){return state},boot,loadPane,draw,render,setCELL:v=>{CELL=v},get DATA(){return DATA}};")
PY
# /tmp/harness2.js のような DOM スタブ (getElementById / getContext / getComputedStyle) から
# boot() → draw(pane, t) を呼び、stroke 色別本数を数える
```

期待値（16MM2017CX / 実測）: `t=0` で既知壁 0 本 → `t=222 s` で既知壁 485 本・
見落とし（青破線）が 500 本超 → 3 本に減少。左ペイン `current` は既知壁 37 本で凍結。
