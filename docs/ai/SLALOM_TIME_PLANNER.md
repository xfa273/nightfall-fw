# 斜め対応・時間最短経路プランナ（PC参照実装）

## 到達点と適用範囲

実機ソフトへ接続する前の参照実装として、直交・斜めを同じ時間軸で比較する
経路導出と、経路から型付き動作列への変換を `common/route/` に実装した。
探索対象は次の8プリミティブの左右である。

- 小回り90度
- 大回り90度
- 大回り180度
- 45度斜め入り／斜め出
- 斜めV90度
- 135度斜め入り／斜め出

この段階では新プランナをF405/F413の実行入口へ接続しない。動的メモリを使う
ホスト実装であり、斜めパラメータの一部は机上の仮値である。共有firmware側へ
入る変更は、既存の経路→斜め`path[]`変換器の安全化だけである。

## グラフとアンカー

論理座標は90 mmセルを45 mm単位のhalf-gridで表す。

| アンカー | half-gridの形 | 許す方位 |
| --- | --- | --- |
| セル中心 | `(odd, odd)` | 東西南北 |
| 内部の縦壁中心 | `(even, odd)` | 斜め4方位、および小回り接続用の東西 |
| 内部の横壁中心 | `(odd, even)` | 斜め4方位、および小回り接続用の南北 |

状態は次の有限組である。

```text
(anchor, heading_8, boundary_speed_class_17)
```

速度classはSTARTと8プリミティブの左右を区別する。次のconnector時間が直前
ターンの出口速度に依存しても、全履歴ではなく現在状態だけで辺コストが決まるため、
前向きDijkstraを適用できる。辺は「同一方位connector + 1 turn」のmacro辺で、
直交方位ではconnectorを0個から、斜め方位から次のturnへ入る場合は1個以上から
列挙する。終端だけは停止可能な直線を付ける。

壁中心anchor自体は45度in/out、V90、135度in/outと斜め直線に必要な正規状態である。
一方、斜め方位でconnectorが0個のまま次turnを始めると、同じ壁中心でturnが直接
連結され、`45-in -> V90/45-out -> ...`の短い折れ返しを時間0の直線として利用できる。
この経路品質上の抜け道を防ぐため、planner列挙、turn edge生成、forward replay
validatorのすべてで次の不変条件を課す。

```text
turn && diagonal(start_heading) => connector_steps >= 1
```

primitive単体の幾何検査は壁中心開始を引き続き許す。禁止対象は物理turnそのものでは
なく、plan内で前の斜めturnから直線なしに次turnへ接続する遷移である。

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

各構成で斜めを含む全action集合と、同じplanner/configから斜め5種だけを無効にした
直交集合を比較する。全action集合は直交集合の真の上位集合なので、
`diagonal goal_entry_us <= orthogonal goal_entry_us`を必須不変条件とする。固定matrixは
斜めturnの0-step接続が0件であることを必須とし、斜めaction採用と厳密短縮は
固定結果の225件へ固定する。legacy statusも`ok`または
`terminal-diagonal-unsupported`以外を失敗にする。結果は
`build/solver_host/slalom_kerilab_matrix.tsv`へ保存する。

2026-08-02の固定データcommit
`762ed2b68735ea29148c6a1251a90ed0651ff26b`で240/240構成がvalidatorを通過し、
0-step斜めturn接続は0件だった。225構成で斜めactionが採用されて直交限定より
厳密に短く、15構成は直交経路と同値だった。正の短縮幅は6,179〜16,892,052 us、
全240構成の平均短縮は2,287,136.8 usである。現runnerへの論理幾何変換は90構成が
`ok`、150構成は斜め終端のため`terminal-diagonal-unsupported`であり、後者を黙って
近似変換しない。

matrixのTSVは実行ごとの一時ファイルへ書き、全gate通過後に同一ディレクトリ内で
原子的に置換する。並行実行や途中失敗で既存の完全な結果を部分追記で壊さない。

## 実機ソフト反映前に残すgate

PC側のアルゴリズムと変換fixtureは完了したが、実機反映には次を要求する。

1. 45/135/V90を各対象modeで実機調整し、PC仮値を実測値へ置換する。
2. 完成機の安全包絡を測定し、直進を含む掃引判定とclearance-aware再探索を実装する。
3. typed executor、またはlegacy runnerの速度連続・斜め終端を実装する。
4. hostの約40万状態・動的メモリを、F413のRAMと実行時間に合う固定領域／探索方式へ
   設計し直す。
5. 1 kHz離散制御、壁切れ補正、動画／trace実測を公称時間モデルと照合する。
6. 上記を通した後、HIL安全手順に従い低速・単一primitiveから段階的に有効化する。
