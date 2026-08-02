# 斜め対応・時間最短経路プランナ（PC参照実装）

## 到達点と適用範囲

PC参照実装として、直交・斜めを同じ時間軸で比較する経路導出と、経路から型付き
動作列への変換を `common/route/` に実装した。
選択経路のターンはKERI氏のターン一覧 #1〜#5だけに固定する。

- #1: 45度斜め入り／斜め出
- #2: 大回り90度
- #3: 135度斜め入り／斜め出
- #4: 大回り180度
- #5: 斜めV90度

#0の小回り90度は直交限定baselineとの比較用にだけ残し、斜めplannerの候補・出力
には含めない。PC参照実装は動的メモリを使い、斜めparameterの一部は机上の仮値で
ある。F413では同じトポロジと時間契約を16x16固定メモリへ縮小した非走行previewを
用意し、typed actionの実走行executorへの接続は引き続き行わない。

## グラフとアンカー

論理座標は90 mmセルを45 mm単位のhalf-gridで表す。

| アンカー | half-gridの形 | 許す方位 |
| --- | --- | --- |
| セル中心 | `(odd, odd)` | 東西南北 |
| 内部の縦壁中心 | `(even, odd)` | 斜め4方位、および小回り接続用の東西 |
| 内部の横壁中心 | `(odd, even)` | 斜め4方位、および小回り接続用の南北 |

状態は次の有限組である。

```text
(anchor, heading_8, boundary_speed_class_11)
```

速度classはSTART、各primitiveの公称境界速度、全#1〜#5に共通のLOW、開始境界速度と
同じCRAWLである。左右は同じ速度なので状態を分けない。次のconnector時間が直前
turnの出口速度に依存しても、全履歴ではなく現在状態だけで辺コストが決まるため、
前向きDijkstraを適用できる。辺は「同一方位connector + 1 turn」のmacro辺で、
connectorは直交・斜めとも0 half-stepから列挙する。終端だけは停止可能な直線を付ける。

壁中心anchorと0-step turn接続は、45度in/out、V90、135度in/outを組み合わせるために
必要な正規状態である。以前の「斜めturn前に最低1 half-step」という一律制約は、
合法な #1/#5 の連結まで壊すため撤回した。代わりに固定commitのKERI
`StepMapSlalom::Index::next()` / `update()`と同じ、primitive固有の前後guard開口を
候補生成とforward replayの両方で検査する。中心線が直接横切る壁だけでなく、turnの
内側／出口側に必要な開口も揃わなければedgeを生成しない。これにより、添付で問題に
なった壁を挟むV90は拒否し、合法なzero-step 45度/V90連結は保持する。

構造上の参考はKERI氏の `StepMapSlalom` である。同実装もセル中心の直交姿勢と
壁中心の斜め姿勢、複数ゴール、直線macro辺を用いる。Nightfall側は既存
`path[]`の小回り距離所有と、入口／出口速度を状態へ加えた独立実装である。

- [StepMapSlalom.h](https://github.com/kerikun11/micromouse-maze-library/blob/3170f7d50be544328257ab63180f2d279793c00a/include/MazeLib/StepMapSlalom.h)
- [StepMapSlalom.cpp](https://github.com/kerikun11/micromouse-maze-library/blob/3170f7d50be544328257ab63180f2d279793c00a/src/StepMapSlalom.cpp)

## プリミティブの論理終点

ローカル座標は進行方向を`forward`、左を`lateral`とし、右ターンではlateralを
反転する。`h=45 mm`である。

| Primitive | 始点→終点方位 | 論理終点 `(forward, lateral)` |
| --- | --- | --- |
| small 90 | cardinal→cardinal | `(h, h)` |
| large 90 | cardinal→cardinal | `(2h, 2h)` |
| large 180 | cardinal→opposite | `(0, 2h)` |
| 45 in | cardinal→diagonal | `(2h, h)` |
| 45 out | diagonal→cardinal | `(3h/sqrt(2), h/sqrt(2))` |
| V90 | diagonal→diagonal | `(h*sqrt(2), h*sqrt(2))` |
| 135 in | cardinal→diagonal | `(h, 2h)` |
| 135 out | diagonal→cardinal | `(h/sqrt(2), 3h/sqrt(2))` |

全8種について、左右のcanonical配置、終点anchor/方位、必要な壁開口、1壁を
追加したときの拒否を公開primitive checkerとfixtureで検証する。

## 時間モデル

直線は入口速度・出口速度を持つ1本のmotionとして計画し、セルごとに停止しない。
直交直線は現runnerと同じく`accel_switch_velocity`を境に低速／高速加速度を
切り替える。斜め直線については、現在のF405 `run.c`とF413
`f413_path_run.c`が`acceleration_d_straight_dash`だけで各DS codeを計画するため、
PC比較も`switch=0`の単一加速度とした。将来二段階斜め加速へ変更する場合は、
plannerだけでなく実行器も同じ契約へ更新する。

速度・加速度表は`float`なので、数学的に半区画停止を意図した値でも量子化後の
停止距離が数nmだけ45 mmを超える場合がある。linear plannerは絶対0.0001 mmかつ
相対1 ppm以内に限り加速度を微補正して運動phaseを自己整合させ、それを超える
停止距離不足は従来どおり`infeasible`とする。

ターン時間は `motion_time.c` のraised-cosine角速度モデルと現parameterから求める。
ただし調整済み直交parameterを同じ簡略モデルで積分すると、論理終点から最大
15.37 mmずれる。この軌跡を論理anchorへ突然接続すると位置が跳ぶため、全8種に
PC専用のexact-closure中心線を別途持たせる。

直交ターンでは現parameter由来の総時間`T`と境界速度`v0`を維持し、exact-closure
中心線の長さ`L`を次の正値速度profileで時間へ対応づける。

```text
v(t) = v0 + A*sin^2(pi*t/T)
A    = 2*(L/T - v0)
```

これにより両端速度は`v0`、距離積分は`L`、総時間は`T`となる。途中速度が正でない
configは拒否する。斜めターンは仮seedを時間・幾何の両方へ使うため補正は丸め誤差を
除き0である。これは不足している実機内周速度データを補う一貫したPC surrogateで
あり、現runnerの実軌跡を測定済みだという意味ではない。

中心線は1回の逐次Simpson積分でarc-length順にcacheし、上式の距離積分を逆算した
時刻も同時にcacheする。closure残差は最大0.001 mm、角度残差は最大1e-6 degに固定し、
gate通過後の最終sampleだけを論理終点へcanonicalizeする。許容値を大きくして位置・
角度warpを持ち込むconfigは拒否する。

開始offsetは任意の出口速度を最適化せず、現F405/F413 runnerの`first_sectionA`と同じく
その距離を最大加速した出口速度へ固定する。したがってSTART辺の時間契約も実行器と
一致する。

開始直後またはzero-step連結で公称turn速度まで加速できない場合に、#0へ戻さず
#1〜#5の同じ中心線を低速で実現する。LOWはそのprofileの調整済み小回り速度を保守的な
共通境界速度として借り、CRAWLは開始offset後の速度を使う。公称specに対する速度比を
`s`として、並進速度と角速度capを`s`倍、角加速度を`s^2`倍する。

```text
v' = s*v,  alpha' = s^2*alpha,  omega_cap' = s*omega_cap
```

この時空相似によりturn時間だけが`1/s`倍になり、中心線、終点anchor、終端方位は変わら
ない。LOW/CRAWLが公称速度以上、または互いに同速なら重複variantを生成しない。選択した
速度modeはtyped actionへ保存し、validatorが速度・時間・modeを同じconfigから再計算する。

### 論理斜め距離とcommand距離

トポロジの1斜めhalf-stepは次の値である。

```text
logical = 45 * sqrt(2) = 63.639610306789 mm
```

現firmwareがDS codeへ指令する距離は別の値である。

```text
command = DIST_D_HALF_SEC = 67.279 mm
```

anchor更新と壁判定にはlogical値、直線所要時間にはcommand値を用いる。両者を
混ぜると、斜め経路の位置か時間のどちらかが黙って変わるため、型付きactionにも
両距離を別フィールドで保持する。

## パラメータ選択

詳細な数値とclosure残差は
[`SLALOM_PARAMETER_BASELINE.md`](SLALOM_PARAMETER_BASELINE.md)に固定した。

| Profile | 用途 | start offset | turn角速度cap |
| --- | --- | ---: | ---: |
| `f413-preorder-mode2` | primary | 5 mm | 2200 deg/s |
| `f405-mini-mode2` | comparison | 13 mm | none |
| `f405-mini-mode3` | comparison | 13 mm | none |
| `f405-mini-mode4` | comparison | 13 mm | none |
| `f405-mini-mode5` | comparison | 13 mm | none |

各profileのcase 8/9から直交・斜め直線速度と加速度を読む。現在F413とF405 miniの
parameter sourceはbyte-identicalであり、auditとhost buildが毎回`cmp`したうえで
実ソースをコンパイルしてbaselineとのdriftを検出する。

- small/large 90/180は調整済みの現値を総時間源として保持し、同じ重心速度の
  PC専用exact-closure seedを中心線に使う。
- 45 in/out、V90、135 in/outは全profileでPC専用exact-closure seedを時間・中心線の
  両方に使う。mode5は斜め値が未設定なので、large turnと同じ1200 mm/sを仮の
  重心速度とした。
- 全seedに0.001 mmの固定closure gateを課し、仮値はfirmware parameterへ書き戻さない。

## ゴール評価

最小化する値は、スタート動作開始から任意のG区画へ最初に入る境界を通過するまでの
`goal_entry_us`である。`stop_us`は停止可能性と動作列検証にだけ使い、順位づけの
第1キーにはしない。ゴール後へ減速区間を移せる場合、全停止距離を一度に計画して
境界位置の時刻と速度だけを取り出す。

直線connectorでGへ入る場合は、その方位のまま停止まで延長し、G進入後に新しい
turnを開始しない。turnの連続軌跡がGへ入る場合は、開始済みのturnを完了してから
出口方位へ直進停止する。どちらも最初の境界交差時刻を記録するため
`goal_entry_us <= stop_us`であり、複数Gの内部を通過して後のGを改めて選び直すことは
ない。

G交差自体は可能でも、#1〜#5のturn終了後に減速距離を確保できない場合は
`no-feasible-terminal`を返し、単なる非連結の`no-path`と区別する。32MM2009HXは
G(24,15)が北・西・東の三方を壁で囲まれ、KERI参照の最終#2が北向き公称速度のまま
G中心で終わるため、この契約の固定期待例である。KERI参照はG node到達で探索を終え、
停止を評価しない。実機へ停止不能な経路を渡したり、暗黙に#0へ戻したりはしない。

## 壁通過とクリアランス

connectorはhalf-stepごとに両側セルの壁bitを確認する。turnは同じPC surrogateの
pose列の全線分で
セル境界との交点を列挙し、通過する壁が両側とも開いているか、maze postを貫かないか、
最初のG交差がどこかを再生する。32x32上限、外周壁、隣接壁mirror、未知bitも厳格に
検査する。

別の任意診断として、39x70 mmの基板外形を矩形機体とみなし、厚さ6 mmの壁矩形との
SAT衝突判定を行える。サンプル間の最大並進・回転を矩形膨張へ加えるため保守的だが、
これは完成機のタイヤ・センサ・外装を含む安全包絡ではない。また現状はturnだけの
postcheckで、直進掃引と衝突時の別経路再探索は未実装である。このため
`require_swept_clearance=true`は明示的に`unsupported-safety`を返し、通常matrixを
「実機安全PASS」とは扱わない。

half-size迷路の基準寸法90 mmセル、6 mm壁は日本大会公式規定に合わせた。
[公益財団法人ニューテクノロジー振興財団・競技規定](https://www.ntf.or.jp/?page_id=534)

## 型付き動作とlegacy互換性

plannerは、開始offset、connector、turn、goal stopを型付きaction列として出す。
各actionには開始／turn開始／終端anchor、8方位、速度class、logical/command距離、
入口／turn／出口速度、所要時間、最初のG交差markerがある。同じplanner configを使う
forward replay validatorが壁、pose、速度、時間、停止を再計算し、探索結果と一致させる。
action単体にはexact-closure seedと途中速度profileを複製していないため、このPC版の
planは生成時と同一configと組で扱う契約である。新しい経路を旧
`simplifyPath/convertLTurn/convertDiagonal`へ戻して再変換しない。

既存のraw直交経路から斜め`path[]`へ変換する処理は
`legacy_path_codec.c`へ置換した。厳格文法、transactional出力、DS1..99、長いDSの
分割、全R/L wordの方位とhalf-grid終点保存を検査する。不完全なturn runは無理に
45/135へせず小回りへ戻す。これにより既知の`[201,R,R,0] -> [201,135-in,0]`型の
斜め状態未終端を除去した。

型付きactionから現runner codeへの`slalom_plan_legacy_codec`も用意したが、これは
次の理由で「論理幾何compatibility gate」であり時間等価変換ではない。

- START_OFFSETはrunner固定のfirst sectionが所有するためcodeへ出さない。
- 直交GOAL_STOPの最後の1 half-stepはrunner固定`half_sectionD`へ譲る。
- 斜め終端は現文法が直交復帰を要求し、runner固定停止も直交45 mmなので、
  `terminal-diagonal-unsupported`として黙って変換しない。
- 現runnerは各S/DS codeで直線速度profileを再計画し、typed actionの実入口／出口速度を
  そのまま実行しない。したがって`legacy_time_equivalent=no`を常に表示する。

実機統合では、typed actionを直接実行するexecutorを追加する案を優先する。legacyを
継続する場合は、同じlinear planner、斜め終端停止、code間の速度連続性をrunnerへ
実装してから時間等価gateを有効にする。typed executor案でも、action単体ではなく
対応configを固定・検証し、PC surrogateを実測済みの実行profileへ置換する必要がある。

## PC検証

単体・sanitizer検証:

```sh
tools/solver_host/run_route_motion_tests.sh
tools/solver_host/run_route_clearance_tests.sh
tools/solver_host/run_slalom_profile_audit.sh
tools/solver_host/run_slalom_time_planner_tests.sh
tools/solver_host/run_legacy_path_codec_tests.sh
tools/solver_host/run_path_pipeline_tests.sh
tools/solver_host/run_slalom_plan_legacy_codec_tests.sh
tools/solver_host/run_slalom_time_plan_host_tests.sh
tools/solver_host/run_solver_host_cli_tests.sh
```

単一迷路の詳細表示:

```sh
tools/solver_host/run_solver_host.sh \
  --maze path/to/maze.maze \
  --slalom-time-plan \
  --slalom-profile f413-preorder-mode2 \
  --case 8 --compare-orthogonal --assert-valid
```

固定commitの過去大会24迷路、5profile、case 8/9の240構成:

```sh
tools/solver_host/run_slalom_kerilab_matrix.sh
```

各構成でstrict #1〜#5集合と、小回り＋大回りだけの直交baselineを比較する。集合は
包含関係ではないが、固定corpusでは #1〜#5 の到達可能230構成すべてについて
`diagonal goal_entry_us < orthogonal goal_entry_us`を回帰条件とする。合法なzero-step
接続数、LOW/CRAWL採用数、legacy status、32MM2009の期待terminal失敗も固定する。結果は
`build/solver_host/slalom_kerilab_matrix.tsv`へ保存する。

2026-08-02の固定データcommit
`762ed2b68735ea29148c6a1251a90ed0651ff26b`では、240構成中230構成がvalidatorを通過し、
全230構成で斜めactionを採用して直交baselineより厳密に短い。32MM2009HXの10構成は
`no-feasible-terminal`である。選択経路の小回りは0、KERI guardを通る合法なzero-step
斜め接続は合計1303、低速turnは268（LOW 219、CRAWL 49）だった。現runnerへの論理
幾何変換は65構成が`ok`、165構成は斜め終端のため
`terminal-diagonal-unsupported`であり、後者を黙って近似変換しない。

matrixのTSVは実行ごとの一時ファイルへ書き、全gate通過後に同一ディレクトリ内で
原子的に置換する。並行実行や途中失敗で既存の完全な結果を部分追記で壊さない。

## F413非走行preview

F413にはUART `K`として、同じKERI #1〜#5トポロジと時間契約で経路を導出する
非走行previewを追加した。保存済みFRAM迷路を正常に読めた場合は必ずそれを使い、
read失敗時だけ固定revision `762ed2b68735ea29148c6a1251a90ed0651ff26b` の
`16MM2014CX.maze` 256-cell wall nibbleをprogram Flashから使う。このfallbackはFRAMを
seed/saveせず、UARTへ`maze-source=builtin-16MM2014CX`、`data-rev`、
`fram-load=fail`を明記する。F413で調整済みなのはmode2だけなので、previewも
mode2/case8固定である。ゴールは過去16x16迷路の比較用に中央2x2をdiagnostic専用値
として使う。走行・探索用の`GOAL1..9`とは独立し、FRAMにも保存しない。UART出力は
`goal-source=diagnostic-center-2x2`を明記する。

PC版の動的メモリをそのまま持ち込まず、16x16の2944 poseと、mode2で必要な
NOMINAL/LOW/CRAWLの3速度に限定した8832状態を固定長で扱う。auto traceが停止中に限り
既存のRAM staging bufferをleaseし、距離・親・settled領域として一時利用する。
lease中のtrace開始は拒否し、全終了経路でbufferを返却する。

`K`は`nvm_maze_load_map()`以外のNVM API、motor、fan、run session、既存`path[]`
executorを呼ばない。各turnについて#番号、左右、直交／斜めconnector、速度mode、
anchor、累積時刻を表示し、最後に選択Gへの最初の進入時刻と停止tail完了時刻を出す。
これは実機上で経路導出を観察する入口であり、導出したactionを走らせる入口ではない。

## 実走行へ接続する前に残すgate

PC側のアルゴリズム、変換fixture、F413非走行previewまでは完了したが、実走行への
接続には次を要求する。

1. 45/135/V90を各対象modeで実機調整し、PC仮値を実測値へ置換する。
2. 完成機の安全包絡を測定し、直進を含む掃引判定とclearance-aware再探索を実装する。
3. typed executor、またはlegacy runnerの速度連続・斜め終端を実装する。
4. previewと実行用profileの時間・action列を一致させ、仮parameterを含むconfigを
   build時に固定・検証する。
5. 1 kHz離散制御、壁切れ補正、動画／trace実測を公称時間モデルと照合する。
6. 上記を通した後、HIL安全手順に従い低速・単一primitiveから段階的に有効化する。
