# 直交・時間最短経路プランナ（机上参照実装）

## 目的と範囲

本実装は、斜めターンの調整完了前に経路探索と時間評価の意味を固定するための
ホスト用参照実装である。現時点で扱う動作は次の直交プリミティブだけとする。

後続の斜め対応参照実装は
[`SLALOM_TIME_PLANNER.md`](SLALOM_TIME_PLANNER.md)へ分離した。本書は直交版の
設計根拠と回帰仕様として残す。

- 直進
- 小回り90度（左右）
- 大回り90度（左右）
- 大回り180度（左右）

斜め直進、45/135度、斜めV90は探索グラフにも動作enumにも含めない。
また、この段階ではF405/F413の実機走行コードを変更しない。

## 評価時間の定義

経路コストは、スタート動作開始から、非ゴール区画からいずれかのゴール区画へ
初めて入る境界を通過するまでの公称時間 `goal_entry_us` とする。

ゴール後は、進行方向に壁まで直進を延長し、停止可能な軌道を同時に計画する。
停止完了時刻 `stop_us` は妥当性検証と表示には使うが、Dijkstraの最小化対象には
加えない。これにより、ゴール後へ減速区間を移してゴール進入速度を上げられる。

最後の接続点からゴール入口までの距離を `P`、ゴール入口から停止点までを `B`
とすると、直進終端はまず `P+B` 全体を入口速度から0まで計画し、`P` の位置の
時刻だけを探索コストに使う。

```text
B = half_cell + post_goal_extension_cells * full_cell
goal_entry_us = time_at_distance(full_stop_profile, P)
stop_us       = full_stop_profile.total_time
```

ここで先頭の `half_cell` は、現行runnerが `path[]` 完了後に必ず走る停止用45 mm
そのものであり、別の終端半区画を加えるものではない。例えば直進1セルでGへ入る
`[201, 0]` は、`201` の45 mm終端でG境界を通過し、その後の固定45 mmで停止する。
ゴール後に1セル開放されていれば、直進コードを90 mm延ばしたうえで同じ固定45 mm
を使うため、`B=135 mm` となる。

ゴール集合は単一座標へ縮約しない。探索中に最初に踏む任意のG区画を終端候補とし、
2x2/3x3ゴールの内部を通過して奥のGを選ぶことはない。

## 動作アンカーと距離の所有権

現行 `path[]` の区画契約に合わせ、ターン入口／出口オフセットと角速度プロファイル
中の並進をターン側が所有する。隣接する直進へ同じ距離を重複計上しない。

探索状態は次の組である。

```text
(x, y, heading, previous_boundary_class)
```

`previous_boundary_class` は `START / SMALL90 / LARGE90 / LARGE180` で、直前動作の
出口速度と、次の直進から既に所有済みの半区画数を決める。次のターンまでに
`n`区画直進するときのconnector距離は次式である。

この有限状態化は、各ターンが `path[]` の論理始終端アンカーを許容誤差内で閉じる
校正済みprimitiveであることを前提とする。残差を実位置誤差として累積させる場合、
同じ論理状態でも到達poseが履歴に依存し、Dijkstraの状態として不足する。

```text
connector_half_sections = 2*n - previous_out_half - next_turn_in_half
```

STARTと大回りターンは出口側1半区画、大回りターンは入口側1半区画を所有する。
そのため、連続する大回り間では、論理的に1区画進みながら物理connector距離が
0になる場合がある。壁通過は論理区画列で、時間と速度は物理距離で検証する。

ゴールへ直接入る小回りは、現行 `path[]` 契約どおりターン終端をゴール境界通過
時刻とする。大回りはターン終端をG区画の中心アンカーとみなし、終端から弧長
45 mm手前とはみなさない。raised-cosine角速度と一定並進速度を2次元積分し、
出口方位軸でG中心から45 mm手前の境界線を外から内へ横切る交点のうち、時刻が
最後のものを採る。交点の横位置が区画幅内であることも確認する。180度では出口軸
射影が全区間で単調とは限らないため、この「最後の有効交点」が必要になる。

大回りでGへ入るには `convertLTurn()` の成立に後続直進が必要なので、ゴール後の
開放区画を少なくとも1つ要求する。例えば `[..., 501, 201, 0]` では、G境界後に
大回り残区間、`201` の45 mm、runner固定45 mmが続く。ターン後に速度0まで減速
できる直線距離は後二者の90 mmである。

大回りの境界交差時には機体角がまだ終端方位へ達していないことがある。出力の
`goal_heading` は境界瞬間の連続角ではなく、ターン完了後の論理方位を表す。
現在のprofileには公称積分変位と論理アンカーの残差があり、hostは `geometry=` 行に
これを表示する。この残差が調整で十分小さくなるまでは、小回り／大回りとも
`goal_entry_us` はアンカーclosureを仮定した机上の公称値であり、実機へ接続しない。

## 時間モデル

内部単位は距離mm、速度mm/s、並進加速度mm/s^2、角度deg、角加速度deg/s^2、
探索コストは丸めた `uint64_t` マイクロ秒とする。

直進は入口速度と出口速度を持つ1本のconnector単位で評価する。低速／高速の
二段階加速度を速度しきい値で分割し、最高速度へ届かなければ、必要距離が一致する
ピーク速度を二分探索する。セルごとに停止・再加速する評価は行わない。

ターン時間はF413の公称角速度生成式に合わせる。

```text
omega_base = sqrt(2 * alpha * angle / 3)
omega_peak = min(omega_base, omega_cap)
t_acc      = rounding_scale * omega_peak / alpha
t_cruise   = max(0, angle / omega_peak - t_acc)
t_turn     = dist_in / velocity + 2*t_acc + t_cruise + dist_out / velocity
```

角速度区間の軌跡計算には、F413制御と同じraised-cosine形状を使う。

```text
omega_acc(t) = 0.5 * omega_peak * (1 - cos(pi*t/t_acc))
omega_dec(t) = 0.5 * omega_peak * (1 + cos(pi*t/t_acc))
dx/dt, dy/dt = velocity * (cos(theta(t)), sin(theta(t)))
```

連続時間で固定分割Simpson積分し、境界交差区間内を二分探索する。単体試験では
大回り90/180の交差時刻、横位置、直線出口内の解析解、境界へ届かない軌跡の拒否を
固定している。実機の1 ms離散サンプリングとの差は、今後ログと比較する公称誤差で
あり、現段階の時刻には含めない。

F413最短走行で角度積算が有効な契約に合わせ、90/180度は調整値ではなく理論角を
用いる。壁切れ補正などセンサ依存の時間は公称値0として含めない。

## 探索と決定性

優先度付きキューを使う前向きDijkstraで、あるターン出口から次のターン出口までを
1本の辺として生成する。辺コストはconnector時間とターン時間の和である。
同じ状態・同じ時刻ではターン数、状態番号の順で決定し、同一入力の動作列を
再現可能にする。

出力は型付き動作列で、各動作に姿勢、論理区画数、物理距離、入口／出口速度、
時間、ゴール境界マーカを持たせる。独立validatorが動作列を再生し、次を確認する。

- 全ての論理移動が壁を越えない
- 動作間の姿勢と速度が連続する
- 最初のG進入だけにゴールマーカがある
- 再生した `goal_entry_us` と `stop_us` が探索結果に一致する
- 最終速度が0である

これは論理壁の通過検査であり、ターン軌跡の掃引領域と機体外形の壁クリアランス
検査ではない。また現在の境界交差検査は指定した進入境界と横位置を確認するが、
軌跡がそれ以前にG正方形の別の辺をclipしないことまでは証明しない。したがって
掃引領域／全セル境界交差を実装するまでは「最初のG進入」も校正済み軌跡を前提と
した値である。

## 迷路fixture

KeriLabの
[`micromouse-maze-data`](https://github.com/kerikun11/micromouse-maze-data/tree/762ed2b68735ea29148c6a1251a90ed0651ff26b)
（MIT）をcommit `762ed2b68735ea29148c6a1251a90ed0651ff26b` に固定して使う。
ASCII parserは最大32x32を動的に判定し、ファイル内のSと全Gを使う。未知壁 `.`、
Sが1個でないデータ、Gがないデータ、不正な壁文字は受理しない。

```sh
tools/solver_host/run_route_motion_tests.sh
tools/solver_host/run_route_planner_tests.sh
tools/solver_host/run_legacy_path_validator_tests.sh
ALL=1 tools/solver_host/run_kerilab_samples.sh
```

## 既存斜め変換の回帰fixture

既存変換は共通`legacy_path_codec`へ置換した。以前再現した
`[201, R, R, 0] -> [201, 135-in, 0]`の不正終端は、不完全なturn runを小回りのまま
保持することで解消した。R/L turn word、前後S1/S2/S3、長いDS、容量不足、未終端を
property testと実`path.c` pipeline testで固定している。

```sh
tools/solver_host/run_solver_host.sh --mode 2 --case 8 --assert-valid
```

直交プランナには斜めcodeを流入させない。斜めプランナは旧converterへ戻さず、
型付きactionを直接生成する。

## 後続実装へ引き継いだ事項

- 45/135/V90と斜め直線はhalf-grid・8方位グラフへ追加済み
- ターン掃引の矩形SAT診断とrequired-open軌跡再生は追加済み。ただし完成機包絡、
  直進掃引、衝突時の代替経路再探索は未完了
- 公称残差をprofile別に監査し、F413 mode2は斜めを含む現調整値を時間源、PC専用
  exact-closure seedを幾何源とした。F405 comparisonの斜めはPC仮値を時間・幾何の
  両方に使い、全8種へ固定0.001 mm gateを適用済み
- 連続raised-cosineモデルとF413の1 ms離散プロファイルの時刻差を定量化する
- 公称時間と動画・ログから得た実測時間の誤差を比較する
- 小迷路のstraight/turn oracleと全8primitive配置検査は追加済み。より大きい独立oracleは継続する
- F413の固定メモリKERI #1〜#5実装とmode2 case6〜9のlegacy runner接続は完了済み
- 完成機包絡・直進掃引とLOW/CRAWL速度sidecar、斜め停止tail対応は継続する

構造上の参考は、KERI氏の
[`StepMapSlalom`](https://github.com/kerikun11/micromouse-maze-library/blob/3170f7d50be544328257ab63180f2d279793c00a/src/MazeLib/StepMapSlalom.cpp)
（姿勢状態、複数ゴール、直線macro辺、Dijkstra）と、同氏の
[`走行時間ベースの過去迷路分析`](https://www.kerislab.jp/posts/2020-03-08-all-japan-32x32-maze/)
を参照した。ただし数値パラメータは流用せず、NightfallのF413 profileを使う。
