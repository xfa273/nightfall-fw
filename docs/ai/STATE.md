# AI Coordination State

このファイルは、Codex の現在の運用状態を記録する正本です。

## 目的

- 会話履歴ではなく、リポジトリ内の状態ファイルで運用を継続可能にする
- 実装・調査・検証の実行方針を固定し、再現性を高める

## 現在の構成

- 実行者: Codex
- 実行方式: Codex が本作業ツリーで直接実行
- 主作業: STM32F413共通ファームの `mini_r2_0` / `mini_r3_0` 対応、実機HIL確認、ログ/ツール整備（F405既存機は維持）
- mini_r3発進積分対策（2026-09-22 JST、t0.20）: 10:34 R90/10:36 R180の旋回最大角度誤差1.073/1.211°へ改善、発進角度+0.702/+0.128°から70ms内に+1.267/+0.764°へ追加ずれ。encoderも同方向。保持中の角度Iが+14/+13deg/s、角速度Iが左旋回出力約2.8/1.8%残るためFAN_ON角度I0.1→0、停止保持→非ゼロ速度profile公開時にfan稼働・非旋回条件で角速度I/D・FF微分履歴のみ原子的にクリア（pose/目標/並進I維持）。R180速度平均1.391/min1.264m/s・出力49.2%で未飽和のため速度P0.35→0.45。FF等維持。R90停止は約0.0004m手前で正確な目標横切りを待って滞留したのでtest modeでは減速profile完了後に既存0.00075m/0.020m/s/0.250s整定へ移行。fan slew/300ms待機維持・新gateなし。実tick両fan/方向/前後発進・保持解除と非解除、session停止許容/timeout/maze非変更、既存profile/path219/mode6全10+2736/route14210・両MCU build PASS。実機操作なし、R90発進・停止→R180追試待ち。詳細 `docs/MINI_R3_SUCTION_STARTUP_FIX.md`。
- mini_r3全ゲイン再確認（2026-09-22 JST、t0.19）: 10:17 mode6/case0/sub1ログでt0.18の並進FFに整合する出力を確認。直進後半feedback1.505/encoder1.498 m/s、停止誤差ログ上+0.000265 m・並進PWM0.49%、timeout/encoder flagなし。開始方位+1.427°・旋回中最大角度遅れ3.104°を改善するためFAN_ON角度P10→15、omega FF0→0.040、alpha FF0→0.0008（既存4ms/120deg/s先行補償に追加）。並進/距離・各I/D・omegaP0.8・fan100%は維持、壁/斜めゲインは同定根拠なし。機体/停止/全軸ゲイン選択/FF両符号/高速feedback ASan/UBSan・両MCU build PASS。実機書込/駆動なし、同じR90で再確認後sub2へ。現180°はalpha41000で90°45500より小さく、主に最高omega2200へ上がる試験。詳細 `docs/MINI_R3_SUCTION_GAIN_REVIEW.md`。
- mini_r3吸引FF初期調整（2026-09-22 JST、t0.18）: 09:52 mode6/case0/sub1ログの直進エンコーダ速度/加速度と左右平均PWMから摩擦・速度・加速度FFを推定し、FAN_ONを35/0.035/0.004→65/0.110/0.0100へ。1.5 m/s定速FFは8.75→23.0%。現在の速度P0.35・角速度P0.8・mode6ファン100%をそのまま保存、I0.001等は維持。停止積分残留はFF不足改善後に再評価。全吸引モード共通なのでmode4ファン50%・低速/逆転適合は未確認。機体profile/停止/ゲイン選択/速度feedback host試験・F413/F405 build PASS。実機書込/駆動なし、ユーザ再走行待ち。詳細 `docs/MINI_R3_SUCTION_FF_TUNING.md`。
- mini_r3速度追従確認（2026-09-22 JST）: 最新09:52:00 mode6/case0/sub1は目標1.500 m/s、feedback最大1.393。1.2 clip再発なし。直進後半のfeedback平均1.375/encoder1.369 m/s、左右平均PWM21.75%・全ログ最大45.1%。1.5指令は336msで、出力は現ローカルFAN_ONのP0.35/I0.001・FF35+0.035vとRMS0.256‰で一致（実機FW SHAはログにない）。定速FF8.75%に対し不足分をP/Iで補い、短い区間で追従が完了しない形。停止時は距離目標506.398/実測520mm、速度指令-27mm/sでも積分残留約7%により前進PWM約2.5%。終了の共通timeout/encoder flagと約250ms待機は停止整定timeoutと整合するがUART理由文字列は未取得。速度未達はFF/PID調整不足で説明可能、I単独増量よりFFとPを先に調整。調査のみ、FW/ゲイン変更・実機操作なし。
- F413固定上書き撤去（2026-09-21 JST）: r2/r3共通の推定±1200と直進/斜め/ターンの固定速度制限を撤去。走行paramsを直接採用。omega上限は任意モードパラメータへ移し、r3 mode4/6だけ2200 deg/sで既存形状を維持。case0強制制動を設定加速度から求める停止距離へ、mode2 case6固定経路を保存迷路生成へ、PID並進試験の角度固定ゲインをFAN_ON/OFF設定へ修正。r3 t0.17/r2 t0.2。実制御tickの高速feedback、全モード設定と既存経路/機体/両MCU buildをhost検証。実機書込/駆動なし。case0の停止尾部は設定に応じ45 mmより長くなる（開始ログstop_mm）。詳細 `docs/F413_RUN_PARAMETER_LIMITS.md`。以下の未修正診断は修正前の履歴。
- mini_r3速度頭打ちの原因確認（2026-09-21 JST、未修正）: 最新23:26:26 mode6/case0/sub1はreal/accel速度が直進中1200mm/sへ固定。共通制御のF413_CTRL_VEL_EST_MAX=1200がIMU併用推定値をclipし、ENABLE_CONTROL=1の直進でfeedbackにも使うため、1.5m/s目標との架空誤差が残る。3.055..3.095sは目標約1.499/表示1.200/encoder換算平均1.594m/s、距離差でも1.60m/s、PWM左右約28%（全ログ最大42.2%）。物理速度不足/出力飽和という説明ではない。ターン中だけencoder LPFへ切替わるためrealは1.2を超え得る。実production tick host入力1.5m/sでもfeedback1.2/目標1.5/PWM252を再現。今回はユーザの原因照会への調査でFW/ゲイン変更・実機操作なし。mode6設定時に残存上限を見落としており、次は低速条件の互換性を保って推定値の固定clipを修正してからゲイン調整。ログにはlive FW SHAなし、現ローカルP速度0.35/omega0.8を実機値とは断定しない。
- ログ表示単位修正（2026-09-21 JST）: 最新23:04:42 mode6/case0/sub1の角速度1.5M表示は、Webがraw mdpsと換算dpsをdeg/s軸へ同時描画する1000倍混在。実角速度ピーク1684.2deg/s、角度にも同じ不具合。WebとPlotJuggler既定をs/m/m/s/m/s²、角度deg・角速度deg/s（ユーザ追加指定）、motor比率へ換算し重複系列を除去、ADC/encoder/予約欄はraw明示。壁観測予約欄の偽tune判定もTUNEフラグ必須へ修正。10ホスト試験と最新3866行・3.986s再生PASS、起動中ビューアを再起動し実画面でも単位軸・omega約-1700を確認。保存ログ・fw/NVMスキーマ・調整中paramsは不変、実機操作なし。旧独自PlotJugglerレイアウトは変換時--legacy-unitsで対応。詳細tools/logging/README.md。
- mini_r3高速吸引ゲイン調整準備（2026-09-21 JST）: ユーザがt0.15で吸引走行安定を報告。t0.16はmode6小回り1200/大回り・斜め1500mm/s、fan50%・600ms ramp/300ms待機を継承しcase0全10項目を用意。PR21/ab377c8理想モデルからalpha/in/outを生成し左右16ターンの終点誤差1mm未満、ただし斜め保守的3mm余裕未達（V90有効-0.027mm）で実機調整初期値。全9ケース直進/斜め1500mm/s・加減速25000mm/s²は45mm停止条件から設定、最短発進/方向の合う全ターン対のS1/DS1接続/停止2736経路PASS。ユーザ指示で実fan PWM状態に応じFAN_ONを選択し全19項目を現FAN_OFFから独立数値コピー、mode4も同じ選択。別値ON/OFFの実制御tick・PWM状態/吸引session・機体profile・既存mode4/path225/route14210・両MCU build PASS。r2fanless/F405、mode4ターンと開始タイミング維持。実機書込/駆動なし、高速追従性・25m/s²加減速・迷路壁余裕未検証。詳細 `docs/MINI_R3_MODE6_SUCTION_TUNING.md`。
- mini_r3吸引立上げ時間調整（2026-09-21 JST）: ユーザがt0.14で大幅改善・若干不安定と報告。指示どおりt0.15で一定レート（50%まで600ms、100%まで1200ms）、指定出力到達後のmini_r3待機300msへ。mode4出力50%、先行姿勢保持・角度基準・guardは継続。hostで25/50/75/100%の300/600/900/1200ms到達と到達後300ms、遅延/時刻wrap/全phase中止・機体runtime profile・route14210・両MCU build PASS。実機書込/駆動なし。詳細 `docs/MINI_R3_MODE4_SUCTION_TUNING.md`。以下t0.14の300ms ramp/100ms待機は履歴。
- mini_r3吸引後の即終了修正（2026-09-21 JST）: 22:07 mode4/case0/sub2ログはctrl開始1205ms→20ms先行＋fan100ms＋整定1秒で2325msにcleanup、2386msにtimeout/encoder共通flag。角度-1.717..+2.664degで前回追加0.5deg条件に達せず、発進しない原因は未検証の整定gate。t0.14でgate撤去、fan OFF校正→位置/角度保持20ms→fanを300msで50%へramp→100ms待機→同じ角度基準・連続制御で発進。fan duty更新はPWM停止/再起動なし、guard/全終了cleanup継続。ゲイン/ターン幾何/非吸引/F405不変。実ログ相当poseでも余分なtimeoutなし、ramp遅延/時刻wrap/PWM連続/全phase異常・既存path/route/制御と両MCU build PASS。実機操作なし、rampの実際の反力抑制と開始yawは追試待ち。詳細 `docs/MINI_R3_MODE4_SUCTION_TUNING.md`。下記t0.13整定gateは撤去済み履歴。
- mini_r3吸引始動姿勢保持（2026-09-21 JST）: 21:50 mode4/case0/sub2ログで最初の制御値更新が開始1321ms後。fan→100ms→IMU静止校正→ctrl開始という順序で反力を抑えられず、停止中IMU未更新＋開始時角度resetのため始動yaw実量はログから復元不可。t0.13はfan OFF校正→位置/角度0保持20ms→fan50%→保持下100ms→角度0.5deg/角速度10deg/s/位置1mm/速度10mm/s以内20回連続確認（HAL実待機では通常40ms）→連続制御で発進。追加1秒未収束は中止、保持中NVM flush抑止。ゲイン/ターン幾何/非吸引順序不変。production session全phase中止・整定gate、実1kHz制御の両方向姿勢/位置補正、既存path/routeと両MCU build PASS。実機書込/駆動なし、始動姿勢の実測再確認待ち。詳細 `docs/MINI_R3_MODE4_SUCTION_TUNING.md`。
- mini_r3吸引調整準備（2026-09-21 JST）: ユーザが非吸引探索・mode2最短の動作確認を報告。mode4をr2準拠800/1000/1200mm/s・fan50%で準備、PR21/ab377c8理想モデルからalpha/in/outを算出（実測r2モデルは外挿せず）。case0全subを有効な入口/停止列に整理、mode4吸引時のみturn/diagonal上限1200へ。既存調整済みゲイン維持。hostで経路・fan開始/全終了・PWMを検証、実機操作なし。斜めの保守的3mm余裕は未達でcase0調整初期値、吸引迷路走行の資格は未取得。詳細 `docs/MINI_R3_MODE4_SUCTION_TUNING.md`。
- 機体選択: NVM identityの機種・個体IDからハード設定と走行profileを起動時選択。運用は `docs/F413_MACHINE_CONFIG.md`、未登録IDは安全停止
- unit002近接復帰方針変更（2026-09-21 JST）: 20:02ログはback選択後138msでwall_fault、実速度168mm/s・FR/FL2862/2696（41.75/43.18mm）、前壁合わせ未実行。ユーザ指示でt0.11は42.5mm打切り廃止、近接/飽和/近距離範囲外でも停止待ちから位置合わせへ。片側でも目標－許容誤差より近ければ旋回より後退-30mm/s優先、前進積分を消去、両側回復20周期＋実測後退0.5mm以上で通常補正へ。補正全体実時間1秒・後退1回12mm上限、未収束は駆動停止・timeout伝播。連続位置合わせtestもactive補正1秒上限、欠測/非有限/20ms stale/stop/IMU/encoder guard維持。実wait/alignment/controller/control tick ASan/UBSan・実LUT過去3ログreplay・machine/NVM guard/path225/solver/F413/F405 build PASS。ゲイン/LUT/IMU/NVM/stable不変、共通F413のためr2にも作用、F405ソース不変。実機書込/駆動なし・物理的後退/迷路完走未検証。詳細 `docs/F413_STOP_APPROACH_FIX.md`。下記t0.10の近接打切り方針は履歴。
- unit002ゴール折り返し停止修正（2026-09-21 JST）: 19:34/19:36ログは全面探索smap9でback選択後、前壁位置合わせより前の45mm減速停止が5000ms timeout。経路閉塞ではなく、残距離約6.4/3.5mmで停止待ち。共通F413制御の減速終了後accel/FF残留と離散終点不足を修正、低速補正±30mm/s、探索停止判定をprofile完了＋誤差1mm以内＋速度10mm/s以内20周期へ。折り返し前のみvalid前壁45mm到達でencoder目標を破棄し位置合わせへ引渡し、近接42.5mm未満/飽和/欠測/20ms staleは駆動停止・異常終了。ゲイン/LUT/IMU/地図/NVM/stableは不変、r3識別版t0.10。r2にも共通制御修正が及ぶ、F405ソース不変。実wait/control tickのASan/UBSan、実LUTログ値replay、既存machine/NVM guard/path-linear225/solver host・両MCU build PASS。実機操作・書込なし、停止精度/制動距離/迷路再走は未検証。詳細 `docs/F413_STOP_APPROACH_FIX.md`。
- unit002 前壁LUT追加測定反映（2026-09-21 JST）: ユーザの中心80..110mm測定を受領。t0.9で既存40..75mmを保持、80mmを最新FR599/FL568へ置換し、85..110mmを追加した15点LUTへ更新（90mm422/422、110mm223/233）。FR+FL合計も更新、PCHIP/validity/低信号/飽和guardは維持。90mm入口と80..88mm前壁ターン目標が測定範囲内。t0.8保存済みゲイン・側壁LUT・オフセット/NVM・r2/F405・ターン幾何不変、stable保存点も不変。測定元 `params/mini_r3_0/calibration/front_centre_20260921.csv`、全受領列はextensionファイル。生成一致、15点/全ADC単調性/90mm accessor/82mm crossing/範囲外拒否のhost試験、NVM params/guard、F413/F405 build/diffcheck PASS。今回は実機操作/書込なし。次はt0.9書込後の距離表示と前壁あり/なしのターン追試。下記追加測定待ちは解消、実機補正精度の確認は未完。
- unit002 2S非吸引のユーザ調整値保存（2026-09-21 JST）: ユーザが非吸引調整完了・基本迷路走行はr2と同じ走行値で良好と報告。作業ツリーの速度P0.24/角速度P0.45をsource929f000、`mini-r3-2s-fanoff-t0.8`に保存。その他ゲイン・IMU併用=1/omega例外=0・探索/最短配列維持。保存点 `stable/mini/unit002/s20260921-mini-r3-2s-fan-off/` にsource SHA/ゲイン/適用範囲を記録。前壁補正は現F413では既に距離式で、r2旧基準から+38mmへの対応済み（探索82/80mm、最短80..88mm）。r3 LUT上限80mmで入口90mmを読めないため補正がfallbackする条件を確認。ユーザは追加測定を後で実施するため、今回はLUT/入口幾何を変更しない。換算表 `docs/MINI_R3_FRONT_TURN_REFERENCE.md`。F413/F405 build・ASan/UBSan機体/換算audit・保存hash/YAML/diffcheck PASS。今回は実機操作/書込/NVM変更なし。過去SWD書込保留後の実機binaryは未再確認、走行評価はユーザ報告。吸引/3S/全mode・unit001故障機の資格を広げない。
- unit002 IMU併用復帰の書込保留（2026-09-21 JST）: ユーザ指示でmini_r3の速度feedbackをencoder+IMUへ復帰、source82157c1/t0.7・ENABLE_CONTROL=1、従来のomega-profile例外=0と並進/距離ゲインは維持。ユーザ編集中omegaP0.2はローカルbuildに含むが復帰commitには含めない。F413/F405 build・ASan/UBSan機体test PASS。native CLIのSTLINK一覧は成功するがHOTPLUG4MHz/1MHzともUSB timeout/core ID読取失敗、ID退避も未完了。V3専用USB復旧helperは対象なしでresetなし。書込/erase/reset/UART/駆動/NVM変更なし。実機は最後に当方確認した0020011dirty/t0.6のままで、t0.7適用は未確認。電源/debug配線・USB挿し直し後、ID退避→app flash/verify→非駆動でruntime flag確認が次。詳細 `docs/MINI_R3_UNIT002_TRANSLATION_TUNING.md`。
- unit002浮上並進粗調整完了（2026-09-21 JST）: 挿し直し後UART復旧。2S相当8V2A/fanOFF/浮上固定で速度・距離のみ試験。旧t0.5の300stepで大振動/飽和/VBAT低下を認め同設定反復せず、r3速度P0.08/I0.001・staticFF35・encoder3msLPFへ変更、距離P2/I0へ弱化。source0020011・通常FW0020011dirty/t0.6、最終300step定速304/SD15mm/s・peak PWM7.3%、距離trapezoid269/270mm。途中の500速度試験も安定、試験限定18%cap/500set1は完全撤去済み。r2/F405・角度/角速度・壁値不変。全build/host PASS、全IDsector/校正prefix保持・maze_known60不変、traceのみ各run更新。最終fault0/全駆動OFF/mode0/UART解放。発熱・異音の人手回答は未取得。床上/旋回/吸引/3S未検証、次は低速床上短直進（新規床上許可が必要）。詳細 `docs/MINI_R3_UNIT002_TRANSLATION_TUNING.md`。下記UART保留は解消。
- unit002並進調整の開始前保留（2026-09-21 JST）: ユーザは2S非吸引・浮上固定で並進のみ粗調整を依頼。SWD/UID/VDD3.24V/進行するtick/fault0は確認したが、UART921600はreset後bootもw等の応答も受信できず。再open/flow制御解除でも不変、原因未確定。ログなしでの駆動を避け、ゲイン/firmware/NVM不変・motor/fanコマンド未送信。全PWM/enable/direction停止をSWD確認、UART解放済み。ユーザへUSB/機体側debug connector挿し直しを依頼、通信復旧後に300mm/s並進set0から評価する。
- unit002壁校正移植完了（2026-09-21 JST）: 同じ光学部品をunit001から移植したユーザの指示で遮光後sensor68Bを保存、offset FR82/FL66/R27/L43。元256B空領域を退避、UID/layout/空領域限定一時処理で保存/読戻し後に完全撤去、通常FW974b280dirtyへ復帰・LOCKED。両app flash sectors0..6 verify、source/destination256B完全一致、identity全sector/distance空prefix不変・trace400。補正後512平均17/6/35/63・全壁なし。motor/fan非駆動、fault0/全出力OFF/mode0/UART解放。共通距離LUTt0.5維持、unit001の無効なdistance診断fixtureは非移植。横壁中央基準L1941/R1989は暫定のまま。次は2S/fanOFFで中央壁基準→床上直進/旋回→低速迷路。fan通常走行連携/ゲイン選択、3S保護、r3用事前計算経路テーブルは別途ソフト作業が必要。下記「旧校正コピーなし」は登録時履歴。詳細 `docs/MINI_R3_UNIT002_BRINGUP.md`。
- unit002左極性修正完了（2026-09-21 JST）: ユーザが左のみ逆回転を目視確認し設定適用を指示、以後encoder正常として方向判定可と確認。unit002専用left-forward-IN2 Highへ変更、encoder L+1/R-1・右極性・他個体は不変。source18c5660、machine/PWM hostとF413/F405 build PASS、app sectors0..6 flash/verify、実機18c5660dirty・unit2・L/R=1/1・LOCKED。浮上固定8V2Aで12%500ms、L前+2577/後-2233・R前+2569/後-2236、非駆動側0。方向不整合は解消し下記の未解決記載は履歴。床上/閉ループは今回未実施。ID全sector/校正prefix不変、fault0・全駆動OFF、mode0/UART解放。
- unit002再試験追記（2026-09-21 JST）: ユーザが吸引OK、配線はunit001同様のはずと報告。依頼により6/7/8/9を予告・約5秒間隔で再試験、L前-2195/後+2568、R前+2570/後-2225、非駆動側0。左符号逆は再現、目視方向回答待ちで設定未変更・閉ループ不可。全駆動停止/fault0/mode0/UART解放。以下の吸引動作追認待ちは解消（電流・温度の数値は未取得）。
- unit002追加実装HIL（2026-09-21 JST）: 壁センサ/encoder/走行motor/fan実装、浮上固定8V/2A・debugger5V OFFをユーザ確認。FW001d688dirtyのまま6/7/8/9を各12%500ms、L前-2192/後+2581、R前+2535/後-2210、非駆動側0。左だけ想定符号が逆で目視方向確認待ち、閉ループ走行不可・極性未変更。fan20/50/80%各1.2秒→停止のコマンド完了、実回転/吸引/熱等は追認待ち。壁ADC4系統は壁設置後512平均FR1235/FL837/R1757/L1863・全壁あり、撤去後71/58/35/48・全壁なしに復帰、飽和なし/offset0。reset後mode0、全駆動OFF/fault0、NVM変更なし、UART解放。詳細 `docs/MINI_R3_UNIT002_BRINGUP.md`。
- unit002 FRAM追試完了（2026-09-20）: ユーザ許可後、ログ領域内4か所各256Bへ00/FF/55/AA/位置依存パターンを書込み・読戻し、全PASS。元1024Bを復元し一致確認。校正prefixとidentity全sector不変。専用一時コードを撤去し通常FW `001d688 DIRTY=1`へ復帰、NVM guard LOCKEDとk拒否、IMU/ADC、fault0・motor/fan停止確認、mode0/UART解放。全容量・電源断保持試験ではない。以下のFRAM許可待ち/健全性未確定は履歴となる。詳細 `docs/MINI_R3_UNIT002_BRINGUP.md`。
- unit002非駆動確認のユーザ追認（2026-09-20）: 全LED点灯を目視確認、8V入力74mAを報告。以下のLED目視/電流確認待ちは解消。スイッチ押下確認、稼働後の発熱再確認は未完。
- mini_r3新基板の登録後（2026-09-20）: ユーザ承認により `mini_r3_0_unit002` / UID `001D0038-32345108-36383936` を登録、IMU実装済み確認。共通FW `3ee4d27 DIRTY=1` で100MHz正常起動、左右前進IN2 Low/High（unit001の左反転を継承せず）、profile0x30001を暫定選択。IMU ID/設定と静止gyro/約1.01g加速度、未実装壁ADC取得、releasedスイッチ、LED出力コマンド完了を確認。FRAM読出しはゼロで、書込み試験の許可待ち・健全性未確定。旧校正コピーなし、FRAM変更なし、motor/fan非駆動、fault0、最終mode0/UART解放。LED目視・押下入力・稼働後電流/温度は未確認。詳細 `docs/MINI_R3_UNIT002_BRINGUP.md`。以下の新基板「登録承認待ち」は同日の登録前履歴、旧unit001の発熱問題は未解決のまま別管理。
- mini_r3新基板（2026-09-20）: ユーザが新規実装、8V/2A・発熱なし、センサ/モータ/ブザー未実装と報告（IMU実装範囲は確認待ち）。新MCU UID `001D0038-32345108-36383936`、identity空、app先頭も空を確認。共通app `1bf4966 DIRTY=1`をbuild/sector0..6 flash/verifyし、100MHz設定でUART `[SAFE] machine=identity-invalid` 起動、CFSR/HFSR=0、ST-LINK VDD3.24Vを確認。旧unit001とは別個体。unit002新規登録のユーザ承認待ちで、registry/identity未変更、IMU/FRAM/ADC等はSAFEにより未初期化・未検査。モータ/fan/NVM操作なし、UART解放済み。PCの既定CubeCLIはx86-onlyで起動不能だったため、同梱ARM版の一時コピーと既存依存ライブラリ/DBリンクで実行（アプリ本体は不変）。
- mini_r3壁距離: 遮光後 `mini-r3-wall-centre-t0.5` / source `3cc013e`。2026-09-12モータ再確認で実機boot `aba28d5 DIRTY=1` / t0.5を確認（この再確認ではflashなし）。実効/保存offset FR82/FL66/R27/L43、校正prefixは試験前後不変。下記0906の値を無断復元しない。機体中心基準FR/FL/合計40..80mm各9点・L/R23..80mm各12点、前壁目標45mm、横壁制御ADC生値、閾値/ゲイン・mini_r2不変。測定時delta/offsetとの対応確認と新LUTの45mm実寸確認は未実施。詳細 `docs/MINI_R3_COMMISSIONING.md`
- mini_r3駆動の未解決事項（2026-09-12）: 浮上固定8V/2Aで左前後進12%・各500msを2回ずつ試し、全てencoder0。ユーザ目視でも全く動かず電流変化も見えない。右は前進+2421/後退-2146。機体設定/PWM端子設定に不整合なし、波形/ドライバ入力は未測定。左配線・モータ・U2/R35周辺を電源OFFで確認する段階。追加高出力・固定機体の閉ループ旋回は実施しない。最終mode0、全駆動OFF、UART解放済み。FW変更なし。
- mini_r3通電試験中止（同日追記）: ユーザがメカに問題なし・電源投入だけでU2発熱・8V入力約0.23Aと報告。安定化電源とデバッガ両方の切離しを依頼、切離し完了は未確認。以後ライブ操作なし。停止中発熱はU2損傷/実装短絡/EN・GND接続不良/近傍からの伝熱を切分ける必要があり、入力0.23Aは正常判定根拠にならない。電源OFFで外観・モータ切離し後の出力短絡確認を優先し、駆動再開しない。
- mini_r3次基板の実装方針（2026-09-15）: ルーペでU2周辺不良は見えず、密集実装のため再実装は現実的でないとユーザ判断。劣化ペースト使用と他箇所の実装不良修正歴があり、新基板で再発予防を検討。はんだ不良起点の二次的U2損傷は有力な仮説だが未確定。新ペースト・定量印刷・指定リフロー条件、U2電源/GNDとC6/C7接合確認、未駆動時の電流/温度基準取得から段階的試験を提案。単純なEN/出力の未接続だけでは停止中発熱を説明しにくい。SR0Ωは許容設定だがサージ余裕未検証、3S/固定旋回/高出力試験は保留。現基板への再実装・ライブ操作・コード/CAD変更なし。
- mini_r3既知の配置不良（2026-09-15）: ユーザ報告でC21がブザーと干渉し未実装。CADでC21=100nF、NRST–GND（MCU7番/RESET0/K1へ接続）を確認。NRSTノイズ抑制の推奨部品で、UFQFPN48の内蔵電源リセットは省略により無効にはならない。C21省略単独は基板再発注を必須とする障害とは判断しないが、正常な駆動系を持つ機体でリセット監視付き段階試験が前提。U2発熱個体の走行許可ではない。C21移設/短配線での暫定追加とリセット要因ログは未実施の改善候補。未実装ランドとブザー電極の接触にも注意。
- 遮光前の2026-09-06校正復旧（履歴）: ユーザ承認後mini_r3 unit001のsensor FRAMを前回正常68Bへ完全復元。実効offset FR708/FL681/R551/L570、壁なし512平均10/14/45/76・全壁なし判定。LUT/閾値/側壁基準値は変更なし。通常ファーム `d5099d8 DIRTY=1` は破壊的診断 a/d/s/m/t/q/Q/r/k を拒否し、全9文字の実機拒否と校正prefix不変を確認。専用復元処理は撤去して再flash済み。正規OP校正と走行ログは従来通り、旧ファームへ戻すと保護は失われる。手かざしの人手確認と距離精度の独立確認は別途。詳細は `docs/MINI_R3_COMMISSIONING.md`。
- 旧運用: Windsurf/Cascade前提の資料はバックアップ済み。互換資産は `docs/ai/archive/` と `.windsurf/` に保持

## Codex 実行ポリシー

- 多ファイル変更、長時間調査、反復ビルド/テストを含む作業も Codex が直接対応する
- ビルド・ホストテストは必要に応じて自律実行する
- ST-LINK/UART実機操作は `docs/ai/HIL_SAFETY.md` に従う
- モータ・ファン・走行・探索・最短・NVM破壊的操作は、安全条件と作業意図が明確な場合だけ実行する
- 旧委譲関連の互換資産は `docs/ai/archive/delegation-legacy/` にアーカイブとして保持する

## 外部マイクロマウス実装の優先参照

壁制御、位置合わせ、探索、走行制御などの公開実装を調査するときは、上位勢の参考実装としてまず以下のGitHubユーザのリポジトリを優先確認する。

- Naophis: https://github.com/Naophis?tab=repositories
- satoshihamasuna: https://github.com/satoshihamasuna?tab=repositories
- kerikun11: https://github.com/kerikun11?tab=repositories

## 標準フロー

1. Codex がタスクを整理
2. Codex が関連ドキュメントとコード境界を確認
3. Codex が実装・調査・検証を直接実行
3. 必要に応じて `docs/ai/WORKLOG.md` に重要イベントを記録
4. Codex が最終報告

## 更新ルール

- 運用方式が変わったときに更新する
- 些細な一時メモは `WORKLOG.md` に残し、このファイルは方針のみ管理する
