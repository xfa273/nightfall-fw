# mini_r3 KiCad routing tools

`route_power.py` はレビュー済み配置に対してローカル電源ループと分割GNDゾーンを
生成する基板専用スクリプト、`route_board.py` はmini_r3の2層／4層基板で
**信号線だけ**を補助的に配線する決定論的なA*ルータです。4層時はIn1.Cuを
GND/GND2の分割基準面として予約し、F.Cu/In2.Cu/B.Cuだけを配線に使います。
いずれの出力も製造可能性を保証する最終成果物ではありません。

## 安全上の前提

- 先に`route_power.py`で電源、モータ、ファン、MPM3610のレビュー済み
  クリティカルループを作り、残った保護ネットを目視しながら仕上げる。
- 元基板を直接入力・上書きせず、最初はプロジェクト一式の一時コピーで試す。
- 出力後はPCB Editorで全配線を目視確認し、ゾーンを再フィルする。
- 元基板と出力基板のKiCad DRCを比較し、**新規のshort/clearance/edge違反が
  0件**であることを確認する。
- 自動配線を採用する前に、GND/GND2境界をまたぐ信号の帰路と、IMU/ADC/
  センサ配線からPWM・スイッチングノードまでの距離を人が確認する。

`route_board.py`はフットプリントを移動せず、既存トラック／ビアも削除・変更しません。
入力と出力が同じ場合は、明示的な`--in-place`がなければ拒否します。保存は一時
ファイルの再ロード確認後に置換します。入力と異なる既存出力の置換にも
`--overwrite-output`が必要です。JSONレポートは基板の入力・出力と同じパスを
指定できず、既存レポートの置換には`--overwrite-report`が必要です。

## 実行環境

`pcbnew`、NumPy、SciPyが必要なため、macOSではKiCad同梱Pythonを使います。

```sh
KI_PY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3
ROUTER=hardware/mini_r3_0/tools/route_board.py
"$KI_PY" "$ROUTER" --help
```

## 電源ルータ

`route_power.py`は`prepare_board.py`直後の、トラック・ビア・ゾーンがない2層基板
だけを受け付けます。入力と出力を同じパスにした場合、または既存出力を指定した
場合は拒否します。フットプリントを動かさず、次を生成します。

- Q2/D5/R12/R33の3S P-FET保護、VBAT_RAW/VBAT_SW幹線
- MP6551の局所デカップリングと4本のモータ出力
- MPM3610の入力、FB、EN、+5 V出力から3.3 V LDOまでの局所経路
- ファンMOSFET、フライバックダイオード、HFバイパスの局所ループ
- 両面のlogic `GND`ゾーンと、R0を唯一の境界とする高優先度`GND2`ゾーン

配置依存の座標を使うため、新D5（SOD-323）とR0を含む主要pad位置も実行前に
検証します。標準実行は次の順です。

```sh
PREPARER=hardware/mini_r3_0/tools/prepare_board.py
POWER_ROUTER=hardware/mini_r3_0/tools/route_power.py

"$KI_PY" "$PREPARER" source.kicad_pcb prepared.kicad_pcb
"$KI_PY" "$POWER_ROUTER" prepared.kicad_pcb power-routed.kicad_pcb
```

レビュー済みの外層配線を4層へ変換する場合は、最終統合済みの2層候補に
`convert_four_layer.py`を適用します。外層の全itemを保持し、In1.Cuへ
`LOGIC_GND_IN1`と`POWER_GND2_IN1`を追加し、In2.Cuは信号／+3V3配電用に空けます。

```sh
FOUR_LAYER=hardware/mini_r3_0/tools/convert_four_layer.py
"$KI_PY" "$FOUR_LAYER" reviewed-2layer.kicad_pcb routed-4layer.kicad_pcb
```

出力後は同じbasenameの`.kicad_pro`/`.kicad_dru`を隣接させてzoneを再fillし、
DRCと`audit_final_board.sh`を必ず実行してください。

長距離のJ2ファン対、電源スイッチ戻り、VBAT_SWの電圧監視枝、全体の+5 V/+3V3
配電は意図的に残します。U3.3の重複`/MOTOR_L_OUT2` padなど、混雑部の重複IC padも
ratsnestを確認して後段で仕上げます。信号A*ルータはこれらを保護ネットとして除外
するため、`route_board.py`任せにはしないでください。

## 保護ネット仕上げ

`finish_protected.py`は`route_power.py`の出力専用です。フットプリントを動かさず、
レビュー済みの固定座標だけを使って次を仕上げます。

- MP6551の重複OUT pad直結、短いfanout、SR抵抗への配線
- U2/U3 pad 9の`GND2` returnと両面power-zone stitch
- Q2直後4本、各枝2本相当の`VBAT_SW` via bankと電圧監視枝
- J2まで並走させたファン電源・return、VCAP、R0/C25/main logic-GND stitch

一般A*へ渡さない電源・モータ・GND経路なので、全対象を明示した次の実行を標準と
します。入力は必ずプロジェクト全体の一時コピーとし、正しい`.kicad_pro`と
`.kicad_dru`を同じbasenameで隣接させてください。

```sh
PROTECTED=hardware/mini_r3_0/tools/finish_protected.py
PROJECT_COPY=/private/tmp/nightfall-route.example/project
PROTECTED_BOARD="$PROJECT_COPY/HM_Nightfall-mini-3a_v0.kicad_pcb"

"$KI_PY" "$PROTECTED" \
  --input "$PROTECTED_BOARD" --output "$PROTECTED_BOARD" --in-place \
  --report-json "$PROJECT_COPY/protected-report.json" \
  --net /MOTOR_R_OUT1 --net /MOTOR_R_OUT2 \
  --net /MOTOR_L_OUT1 --net /MOTOR_L_OUT2 \
  --net /MOTOR_R_SR --net /MOTOR_L_SR \
  --net 'Net-(U5B-VCAP_1)' --net VBAT_SW \
  --net /FAN_NEG_INTERNAL --net GND --net GND2
```

`/PWR_SWITCH_RETURN`、`/MPM_SW_INTERNAL`、長距離railの残件はこの固定経路へ含めません。
U2のSR escapeだけは0.5 mm pitchのため0.16 mm、その後は0.20 mmです。この段階で
残るvia-in-padは最終製造cleanup前の一時状態であり、標準の未充填ビアを前提に
tentingだけで許容しないでください。終了コード`0`は選択した保護ネットの接続完了、
`2`は安全に未配線を残したことを示します。出力は必ずzoneを再fillし、
`--all-track-errors`付きのプロジェクトDRCで比較してから次段へ渡してください。

## 製造前cleanup

`cleanup_manufacturing.py`はレビュー済み最終配置・配線に対して、次の変更だけを
決定論的に適用します。

- `Q3.3`のvia-in-padを削除し、既存のoff-padファンreturn viaへ集約
- `C9`を0.50 mm移動し、`C8.2`のvia-in-padを0.80/0.40 mmのoff-pad viaへ変更
- Q2 sourceからP1へ向かう長い0.30 mm区間を、0.60/0.80/1.20 mmへ早期拡幅
- 前方の`R1FR0.1`と`R2R0.2`に重なっていた穴を、配線を保った短い
  0.40/0.20 mm toe viaへ変更
- 左前LEDの基板端沿いパルス電流枝を0.10 mm内側へ寄せ、全長を0.16 mmから
  0.20 mmへ拡幅

座標と既存配線がレビュー済み状態に一致しなければ停止します。変換済み基板へ
再実行しても同じトポロジを検証して保存する冪等処理ですが、入力ファイルの
直接上書きと既存出力の置換は拒否します。保存には別basenameの一時PCBを使い、
再ロード確認後にatomic replaceするため、実プロジェクトの`.kicad_pro`をpcbnewが
上書きしません。一時basenameに生成された`.kicad_pro`/`.kicad_prl`も削除します。
zone fill条件を固定するため、入出力それぞれと同じbasenameの`.kicad_pro`と
`.kicad_dru`が存在しない場合も拒否します。

```sh
MANUFACTURING_CLEANUP=hardware/mini_r3_0/tools/cleanup_manufacturing.py
REVIEWED_PROJECT=/private/tmp/nightfall-route.example/reviewed
REVIEWED_BOARD="$REVIEWED_PROJECT/HM_Nightfall-mini-3a_v0.kicad_pcb"

# 出力先にも同じbasenameのproject/rule filesとlocal librariesを用意する
CLEAN_ROOT="$(mktemp -d /private/tmp/nightfall-clean.XXXXXX)"
CLEAN_PROJECT="$CLEAN_ROOT/project"
mkdir -p "$CLEAN_PROJECT"
cp -R "$REVIEWED_PROJECT"/. "$CLEAN_PROJECT"/
CLEAN_BOARD="$CLEAN_PROJECT/HM_Nightfall-mini-3a_v0.kicad_pcb"
rm "$CLEAN_BOARD"  # mktemp配下の、これから生成するPCBコピーだけを削除

"$KI_PY" "$MANUFACTURING_CLEANUP" "$REVIEWED_BOARD" "$CLEAN_BOARD"

# 個別検証する場合は、上の全処理コマンドの代わりに実行
"$KI_PY" "$MANUFACTURING_CLEANUP" "$REVIEWED_BOARD" "$CLEAN_BOARD" \
  --actions q2
```

出力を同basenameの`.kicad_pro`、`.kicad_dru`、ローカルfootprint librariesとともに
置いたプロジェクトコピーでzone refillとDRCを実行してください。この処理は
`R0`、MCUデカップリング、Q3 source、その他センサ抵抗など、最終統合後に確認する
他のvia-in-padを
自動修正しません。

手はんだ機ではU5に
`Nightfall-Local:UFQFPN-48_7X7X0P55MM-M_noEP_hand_solder`を割り当てます。このvariantは
側面pad 1～48だけを保持し、露出pad 49と中央F.Pasteを持ちません。回路図symbolの
pin 49はデータシートどおりGNDのまま維持し、footprint側で意図的に省略します。

## 最終基板監査

4層配線候補のレビュー中はstale fillを障害物として扱わないため、In2のpourを
省略できます。全track/viaとIn1の境界が確定した後、`finalize_four_layer.py`を
別出力へ1回適用し、続くKiCad CLI DRCで全zoneをrefillします。この処理は最終
In1の分割輪郭をIn2へコピーするため、途中版の境界を持ち越しません。

```sh
FOUR_LAYER_FINALIZER=hardware/mini_r3_0/tools/finalize_four_layer.py
"$KI_PY" "$FOUR_LAYER_FINALIZER" \
  ROUTED.kicad_pcb ROUTED_WITH_IN2_POURS.kicad_pcb
```

`audit_final_board.sh`は4層化した任意の最終候補PCBを引数で受け取り、候補自体を変更せずに
`/private/tmp/mini3-final-audit.*`へプロジェクトを組み立てて監査します。候補PCBと、
このリポジトリの正本`.kicad_pro`、`.kicad_dru`、`.kicad_sch`を同じbasenameで
コピーし、KiCad CLIの`--refill-zones --save-board`はその一時コピーにだけ実行します。

```sh
AUDITOR=hardware/mini_r3_0/tools/audit_final_board.sh
CANDIDATE=/absolute/path/to/HM_Nightfall-mini-3a_v0.kicad_pcb

# 現在の4層統合baseを確認する例
FOUR_LAYER_BASE=/private/tmp/mini3-four-layer.kqclwf/HM_Nightfall-mini-3a_v0.kicad_pcb
"$AUDITOR" --candidate "$FOUR_LAYER_BASE"

# 作業途中の客観レポート。FAIL項目があっても監査自体が成功すればexit 0
"$AUDITOR" --candidate "$CANDIDATE"

# 製造候補のrelease gate。FAIL項目があればレポート生成後にexit 2
"$AUDITOR" --candidate "$CANDIDATE" --strict
```

標準出力の`AUDIT_DIR`に、機械可読な`audit.json`、要約`audit.md`、KiCadの
`drc.json`、回路図からexportしたnetlist XML、再fill済み一時PCBを残します。監査は
次をrelease gateとして判定します。

- 正本`.kicad_pro`/`.kicad_dru`でzoneを再fillしたDRC errorと未配線が0件
- `F.Cu`/`In1.Cu`/`In2.Cu`/`B.Cu`の全層をKiCad connectivityとtrack幅集計の
  対象にし、`GND`/`GND2`が各1 connected group、6つのGND zone self-edgeが0件、
  `R0.1=GND2`/`R0.2=GND`
- `R0`以外に`GND`/`GND2`を跨ぐfootprintがない
- 全4銅箔層で全viaと全padの銅箔重なり（via-in-pad）が0件、
  0.40/0.20 mm toe viaの位置一覧、
  via径/ドリル/層ペア
- 層別・net別track幅のmin/max/histogram、0.16 mm未満の2 oz限界違反、
  0.16～0.20 mmの局所escape一覧
- copper-to-edge 0.20 mm、銅箔層が正確に
  `F.Cu / In1.Cu / In2.Cu / B.Cu`の4層構成
- `In1.Cu`には`GND`/`GND2` zoneを各1つ以上置き、track segmentと銅箔graphicを
  置かない。全netのF.Cu–B.Cu through-via通過は許可し、`GND`/`GND2` viaは
  same-net stitchとして分類する
- `In2.Cu`には分割した`GND`/`GND2`の低優先度pourを必須とし、その上に
  logic/センサ信号と`+3V3`を配線する。`+5V`、その他`HighCurrent`、
  `SwitchNode`のtrack/zoneは禁止する
- In1/In2それぞれについて、再fill済み`GND`/`GND2` zoneだけを数えた保守的な
  銅箔被覆率が25%以上であること（signal track/via copperは被覆率に加算しない）
- 回路図とPCBの`U2=logical LEFT`、`U3=logical RIGHT`対応、およびR34/R35、
  TP1～TP4、U5モータ制御pinの対応
- U5の露出pad 49と中央F.Paste windowがなく、側面padだけの手はんだvariantであること

旧2層候補にも通常監査を実行できます。その場合、In1/In2固有policyは
`applicable=false`かつPASSとして扱われ、4層構成check
`four_copper_layers_f_in1_in2_b`だけが層構成理由でFAILします。DRC、接続性、
R0、via-in-pad、論理pin契約など従来の監査項目はそのまま実行されます。

JLCPCBの公開値では2 ozの最小配線幅/間隔は0.16/0.16 mm、ルータ加工端から銅箔は
0.20 mm以上です。0.40/0.20 mm viaは製造可能範囲ですが、公開capability表に従い
small-via optionが必要な項目としてレポートします。銅厚自体はKiCad基板ファイルに
完全には符号化されないため、発注時に**4 layers / 2 oz outer copper**を指定し、
選択したJLCPCB stackupを確認してください。

- https://jlcpcb.com/help/article/jlcpcb-copper-weight
- https://jlcpcb.com/capabilities/pcb-capabilities/
- https://jlcpcb.com/help/article/inner-layer-copper-coverage-pcb

## 推奨ワークフロー

まずプロジェクトディレクトリ全体を`/private/tmp`へコピーします。`.kicad_dru`、
ローカルフットプリント、プロジェクト設定を同時にコピーしないと、DRC比較条件が
変わるため注意してください。

```sh
PROJECT_DIR=hardware/mini_r3_0/cad/kicad/HM_Nightfall-mini-3a
ROUTE_TEST_DIR="$(mktemp -d /private/tmp/nightfall-route.XXXXXX)"
mkdir -p "$ROUTE_TEST_DIR/project"
cp -R "$PROJECT_DIR"/. "$ROUTE_TEST_DIR/project/"

ROUTE_TEST_BOARD="$ROUTE_TEST_DIR/project/HM_Nightfall-mini-3a_v0.kicad_pcb"
"$KI_PY" "$ROUTER" \
  --input "$ROUTE_TEST_BOARD" \
  --output "$ROUTE_TEST_BOARD" \
  --in-place \
  --report-json "$ROUTE_TEST_DIR/route-report.json" \
  --verbose
```

全信号を一度に適用する前に、限定したネットで経路品質を確認できます。

```sh
"$KI_PY" "$ROUTER" \
  --input "$ROUTE_TEST_BOARD" \
  --dry-run \
  --net /USART1_RX \
  --net /USART1_TX \
  --verbose
```

DRCはコピーしたプロジェクト名の基板に対して実行します。

```sh
/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli pcb drc \
  --all-track-errors --severity-all --refill-zones --save-board \
  --output "$ROUTE_TEST_DIR/routed-drc.rpt" \
  "$ROUTE_TEST_BOARD"
```

基準基板にも同じ`--refill-zones`条件を使い、保存する場合は必ず一時コピー上で
行います。通常zoneはルータの固定障害物ではないため、この再フィルは必須です。

終了コードは、全対象ネットを接続できた場合が`0`、安全に未配線を残した場合が
`2`です。引数解析エラーもargparseの規約により`2`、その他のファイル・保存エラー
は通常`1`です。未配線を含む出力基板も保存されるため、JSONレポートとPCB Editor
のratsnestを確認してください。

## 常に除外するネット

次のネットはコマンドラインからも有効化できません。

- `GND`、`GND2`
- `+3V3`、`+5V`
- `VBAT_RAW`、`VBAT_SW`
- `/FAN_NEG_INTERNAL`
- `/MOTOR_L_OUT1`、`/MOTOR_L_OUT2`、`/MOTOR_R_OUT1`、`/MOTOR_R_OUT2`
- `/MPM_`で始まるネット
- `/PWR_GATE_INTERNAL`
- `/PWR_SWITCH_RETURN`
- `Net-(D3-A)`
- `Net-(U5B-VCAP_1)`（MCU内部レギュレータのデカップリング）
- `Net-(IR_LED_*-C)`、`Net-(IR_LED_*-PadA)`（センサLEDのパルス電流経路）
- `unconnected-`で始まるネット、およびnet 0

`/FAN_PWM`、`/MOTOR_L_PWM`、`/MOTOR_R_PWM`などのロジック信号は対象です。
追加で除外する場合は`--exclude-net`へ正規表現を渡します。

```sh
"$KI_PY" "$ROUTER" \
  --input "$ROUTE_TEST_BOARD" \
  --dry-run \
  --exclude-net '^/SPI2_' \
  --exclude-net '^/SENSOR_'
```

## 既定の設計値

- 2層基板のみ（`F.Cu` / `B.Cu`）
- 配線幅: 0.20 mm
- 一般クリアランス: 0.20 mm
- `Sensitive`: 0.25 mm
- `HighCurrent`対`Sensitive`: 0.40 mm
- `SwitchNode`対全配線: 0.50 mm
- 幅0.40 mm以下のSMD padとK1のescapeだけ: 0.16 mm、pad外周から0.60 mm以内
- ビア: 0.60 mm / ドリル0.30 mm
- 既存穴から新規ビア銅箔まで: 0.25 mm
- 穴端間: 0.20 mm
- 基板端クリアランス: 0.50 mm
- グリッド: 0.10 mm
- 1接続あたり最大2ビア

機能別クリアランスは`.kicad_pro`のnetclassと`.kicad_dru`に合わせた下限で、
`--clearance`を小さくしても緩和されません。netclass情報が読めないboard単体コピー
でも、mini_r3の既知ネット名には保守的なfallback分類を適用します。

0.16 mmの局所escapeは、mini_r3のファインピッチICと1.27 mm書き込み
コネクタから0.20 mm配線を引き出すためのものです。JLCPCB 2層・2 ozの最小値を
前提にしているため、製造条件を変更する場合は再確認が必要です。

## 段階的な探索

混雑した基板では、一度に探索量を増やすより、コピー基板上で小さい予算から試し、
各段階でDRC差分を確認します。

```sh
# Stage 1: 短時間で通る経路だけ
"$KI_PY" "$ROUTER" -i "$ROUTE_TEST_BOARD" -o "$ROUTE_TEST_BOARD" \
  --in-place --max-expanded 100000 --max-net-expanded 200000 \
  --candidate-pairs 4 --net-order fewest-pads

# Stage 2: 残ったネットに探索量を追加
"$KI_PY" "$ROUTER" -i "$ROUTE_TEST_BOARD" -o "$ROUTE_TEST_BOARD" \
  --in-place --max-expanded 300000 --max-net-expanded 900000 \
  --candidate-pairs 8 --net-order long-first
```

`--net-order`は`long-first`、`fewest-pads`、`short-first`、`mcu-first`から
選べます。`mcu-first`は、指定がなければU5を含むネットを先に処理します。
各ネットについてU5の孤立padから出る最初の接続も優先する場合は
`--fanout-reference U5`を追加します。これは長い探索へ予算を使いやすいため、
通常の順序とDRC結果を比較して採否を決めます。
`--front-axis horizontal`で層の優先方向も比較できます。設定を変えた候補同士を
同じ基準DRCと比較し、違反がない方だけを採用してください。

## 既知の制約

- A*は0.10 mmラスタ近似であり、KiCadの形状DRCそのものではない。
- 別ネットのpadと既に銅箔が重なっているpadは、新しいshorting itemを増やさない
  ように自動的に配線対象から外す。先に配置またはフットプリントを修正する。
- 通常の銅箔ゾーンは配線後に後退・再フィルされる前提で、固定障害物にはしない。
  トラック／ビア禁止keepoutは障害物として扱う。
- 接続先はpadグループを基準にし、既存配線の任意の中間点を積極的な終点にはしない。
- MCU周辺などで経路が見つからない場合は、探索上限で止めて未配線を残す。
- カスタムF.Mask開口の下を通る経路がsolder-mask警告を増やす場合がある。
- GNDリターン、差動・等長、スイッチングループ、熱設計、ゾーン分割は自動最適化
  しない。

したがって、出力は「手動仕上げを始める候補」であり、そのまま製造へ送るデータ
ではありません。
