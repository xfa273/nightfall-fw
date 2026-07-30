# スマートフォン動画によるターン調整

## 1. 目的と現在地

Androidスマートフォンを迷路上方に固定し、次の一連の処理を再現可能に
する。

1. 動画の各フレームを迷路平面へ射影する。
2. 機体上のARマーカを使わず、PCB・輪郭・LEDから位置とyawを求める。
3. 動画PTSとF413 traceの時刻を合わせる。
4. 実軌道、IMU、エンコーダ、目標軌道を同じ時間軸で比較する。
5. 同一条件の反復試験から次のターンパラメータ候補を作る。

現時点で実装済みなのは、ホスト側の実現性確認用パイプラインである。
既存の`IMG_1592.mov`では、機体上ID 3を推定入力から完全に外し、固定
ArUco、緑PCB、単一の赤LED、前景輪郭から515/515フレームを追跡できた。
冒頭の静止区間で外部パラメータを合わせ、走行区間をhold-out評価した
一回の結果は、位置RMSE約6.3 mm、yaw RMSE約4.0度だった。
固定マーカと解析格子の実測mm layout、空迷路clipによる背景、PTS/画像
重複QA、trace全列を保持するmotion同期、安全gate付きの終了姿勢候補生成
までをホスト側に用意している。

この720p/30 fpsの結果は「機体マーカなしで追跡できる可能性」の確認で
あり、自動調整へ使える精度の証明ではない。特に、次は未実装である。

- 実時間120/240 fpsとCaptureResult sidecarを保存するAndroid recorder
- 4枚を超える任意個数の固定マーカを同時利用する盤面校正
- カメラ内部パラメータ、レンズ歪み、LED高さ、rolling shutterの補正
- 複数LEDを幾何拘束で追跡する姿勢推定
- セッション全体のQAを一括で強制するrunner
- trace位相に基づくターン区間抽出と、軌道形状・時間を使うparameter fit
- LEDパターンによるoffsetとclock driftの二点同期

したがって、現在の候補生成は専用の単一ターン試験に限定し、候補を
人が確認して次の試験へ渡す半自動工程として扱う。

## 2. 推奨する最初の撮影構成

### 2.1 撮影範囲と架台高さ

最初のターン調整では全迷路を映さず、標準180 mmセルで4×4程度、最大
でも6×6程度に絞る。1080pで全16×16迷路を映すと機体の画素数が減り、
特にyaw精度を失う。全迷路撮影は長い走行のデバッグ段階で、4K/60 fps
との比較対象にする。

レンズから迷路面までの必要距離は、実際の録画モードにおける水平・垂直
画角をそれぞれ`HFOV`、`VFOV`、必要な撮影幅・高さを`W`、`H`とすると、
概ね次で求める。

```text
lens_height >= max(
    W / (2 * tan(HFOV / 2)),
    H / (2 * tan(VFOV / 2))
)
```

正方形に近い迷路範囲を16:9動画へ収める場合、通常は狭い垂直画角が高さ
を決める。水平画角だけで架台を設計しない。たとえば録画時の実測VFOVが
48度なら、マーカ余白を含む1.0 m角には約1.12 m、1.3 m角には約1.46 mの
レンズ高さが必要になる。

HFRモードは通常動画とcropが異なる場合がある。架台は高さを調整可能に
し、実際の1080p/120モードで外周マーカと走行余白が常時入ることを確認
して最終固定する。ここでいう高さはスマートフォン筐体ではなくレンズ
中心から迷路面までの距離である。

### 2.2 端末

最初の接続・capability確認はPixel 8から始める。

- 公称でスローモーション最大240 fpsに対応する。
- 187 gで、227 gのXiaomi 13 Ultraより架台負荷が小さい。
- Google端末を先に使うことは、Camera2調査時の再現性を取りやすいという
  運用上の判断であり、画質が常に優れるという意味ではない。

製品仕様はCamera2へ公開される解像度、固定fps、露光制御の組合せを保証
しない。両端末を接続後、各camera IDについて次をprobe APKで比較して
最終選定する。

- `REQUEST_AVAILABLE_CAPABILITIES_CONSTRAINED_HIGH_SPEED_VIDEO`
- `StreamConfigurationMap.getHighSpeedVideoSizes()`
- `getHighSpeedVideoFpsRangesFor(size)`
- manual sensor、露光時間、ISO、AF、OIS、EISの利用可否
- `SENSOR_TIMESTAMP`、`SENSOR_FRAME_DURATION`などのresult key
- HFR encoder profile

`SENSOR_ROLLING_SHUTTER_SKEW`の実値や、要求した露光・ISOが適用されたか
は静的probeでは分からない。将来のrecorderで各CaptureResultを保存して
確認する。

2026-07-30にXiaomi 13 Ultra
（`2304FPN6DG`、MIUI `V14.0.5.0.TMAMIXM`、Android 13）を
実機probeした。背面camera ID 0はCamera2上で1080p/120・240・480 fpsを
広告し、manual sensorとREALTIME sensor timestampを持つ。しかし実際の
120 fps constrained sessionは、MediaCodec、MediaRecorder、previewの
どの出力面でもvendor HALの`configureStreams`に失敗した。preview 1面
だけでも失敗するため、当該firmwareでは一般アプリから公開Camera2 HFRを
使用できないと判断する。広告値だけを端末選定の合格条件にしてはならない。

一方、Xiaomi純正Cameraの既存`HSR_240` clipは1280×720、1,337 frameで、
MP4の`com.android.capture.fps=240`、PTS実測239.981 fps、gap 0、隣接
同一frame 0としてQAを通過した。この端末では当面、純正スローモーション
を`collect_stock_slowmo.sh`で回収する方式を実用fallbackとする。正式な
構成判断は、初期化後のPixel 8でcustom recorderを再試験してから行う。

両端末で同じ1080p/120または240 fpsが得られる場合、1型主センサを持つ
Xiaomi 13 Ultraは短時間露光時のS/Nで有利な可能性がある。これはセンサ
寸法からの推定であり、最終判断は同じ照明・画角・露光での実測誤差に
よって行う。480 fps以上は解像度や補間有無を公式仕様だけで確定できない
ため、初期構成では使わない。

### 2.3 初期撮影設定

最初に検証する設定は次とする。

| 項目 | 初期値 |
|---|---|
| カメラ | 背面主カメラ1x |
| 解像度・fps | 1920×1080、120 fps |
| 保存時刻 | センサ時刻と対応した実時間PTS |
| コーデック | H.264を先に検証し、HEVCは次段 |
| 露光 | 1/1000秒、光量があれば1/2000秒も比較 |
| ISO | 飽和せずノイズが少ない最小値 |
| focus | 迷路面へ合わせた後に固定 |
| white balance | 固定 |
| EIS/OIS | 利用可能ならオフ |
| HDR・夜景・自動レンズ切替 | オフ |

固定架台でもEISは画像をcrop/warpして座標系を動かすため使用しない。
OISも可能ならオフにする。Camera2 HFRでmanual設定が拒否される場合は、
AE/AWB/AFを静止状態で収束させてlockし、実際のCaptureResultを確認する。

短い露光はmotion blurを減らすが、rolling shutterの走査時間は短く
しない。走査中の機体回転による形状歪みは、別に測定・合否判定する。

照明は、拡散したフリッカレスDC照明を迷路の複数方向から当てる。短時間
露光で暗くなること、赤LEDが飽和して重心が膨らむこと、壁の影が機体
輪郭へ連結することを同時に避ける。HFR録画は発熱と容量消費が大きい
ため、初期試験は短いclipにし、端末温度と欠落フレーム率を各試験で記録
する。

Android側にはcapability probeに加え、Camera2 constrained HFR、
CaptureResult JSONL、encoded sample JSONLを保存する実験用recorderを
実装した。ただし上記Xiaomi firmwareではsession作成前にHALが拒否する。
stock cameraのslow-motion clipはMP4の`com.android.capture.fps`と
PTS/content QAを毎回確認した場合に暫定入力として使える。Xiaomiの確認
clipは実時間240 fpsだったが、stock経路ではmanual露光、stabilization、
sensor timestamp、frame durationをsidecarとして取得できない。したがって
trace同期にはLEDなどの光学eventを併用し、正式recorderを使える端末との
誤差比較を残す。

## 3. 固定マーカと盤面座標

「機体にはマーカを貼らない」と「盤面にもマーカを使わない」は分けて
考える。盤面外周の固定マーカは、カメラの傾きと移動を検出・補正する
ため残す。

### 3.1 固定マーカの実測layout

固定IDの役割は次である。

- ID 5: 左上
- ID 7: 右上
- ID 4: 右下
- ID 6: 左下
- dictionary: OpenCV `DICT_4X4_50`

homographyへ使うのは現在もID 5、4、6の3枚である。3枚が同じフレームに
揃わないと直前homographyへfallbackし、6フレーム連続すると停止する。
ID 7は検出数のQAだけに使い、homographyにも幾何残差にも使わない。

`--board-layout`を省略するlegacy modeでは、既存8×8試験ボード向けの
幾何をハードコードしている。既定
`--canonical-size 900 --marker-margin 50`での仮定は次である。

- 4マーカの中心が正方形の四隅にある。
- 全マーカは同じ物理寸法・同じ盤面向きである。
- マーカ黒枠の一辺 / 隣接するマーカ中心間隔が
  `39.6 / 800 = 0.0495`、すなわち約4.95%である。

文書上の位置名だけを見て任意の場所へ置くと、平面が歪むかhomography
推定に失敗する。特に全迷路へ小さなマーカを離して置く構成は、この比率
と一致しない。

新しい`nightfall_vision_board_layout_v1`では、固定マーカ中心、黒枠の
一辺、盤面内回転、解析格子をmmで記述し、layoutから直接homographyの
目的角とmetric gridを作る。4×4区画用の実寸例は
`tools/vision/board_layout_4x4_example.json`である。

- 90 mm pitchを8個、原点`[0, 0]`から`720 × 720 mm`
- ID 5/7/4/6中心をそれぞれ
  `[-30,750] / [750,750] / [750,-30] / [-30,-30] mm`
- マーカ黒枠一辺40 mm、`rotation_deg = 0`
- 4枚とも迷路床と同一平面、白いquiet zoneを含めて固定

`side_mm`は紙全体ではなく黒いArUco外枠の実測寸法である。印刷倍率と
中心位置をノギスまたはスケールで確認し、実測値にJSONを直す。この
layoutを指定した場合、JSONの`grid.pitch_mm`がmetric scaleとなり、
`--cell-size-mm`や明るい格子線の自動検出へ依存しない。
座標は+xが右、+yが画面上方で、`rotation_deg = 0`では印刷マーカの上辺を
+yへ向ける。正のrotationは盤面上の反時計回りである。

例の`canvas_bounds_mm = -55..775`は830 mm角の**解析crop**であり、紙や
治具の外形、または必要FOVの外端ではない。外周マーカの黒枠は
`-50..770 mm`まで達するため、その外側へ白いquiet zoneを最低7 mm、
推奨10 mm以上確保する。推奨値なら固定マーカまでの物理的な可視範囲は
約840 mm角となる。実際のFOVには、さらに機体の走行余白と設置ずれの余白
を加える。前節の1.0 m角はこの余白を含めた初期値である。

ただし、現実装がhomographyへ使うのはlayout modeでもID 5、4、6だけで
ある。4〜8枚の全可視マーカをRANSACで使い、leave-one-out QAを行う一般
solverは将来拡張である。

### 3.2 `grid-cells`の意味

`--board-layout`を使わないlegacy modeでは、尺度と原点をrectify後の
背景から検出した等間隔の明るい格子線で決める。`--grid-cells`は迷路
全体の公称セル数ではなく、**画角内の解析格子pitch数**である。

初期構成では解析pitchを90 mmとする。

- 標準180 mmセルを4×4区画映す: `--grid-cells 8`
- 標準180 mmセルを6×6区画映す: `--grid-cells 12`
- 既存8×8の90 mm試験ボード: `--grid-cells 8`

したがって、4×4区画しか映っていない動画へ`--grid-cells 16`を指定
しない。90 mm pitchを使う場合だけ`--cell-size-mm 90`を指定し、実測済み
の場合だけ`--cell-size-confirmed`を加える。

legacy方式は、画角全域に90 mm間隔の明るい校正線が見えることを前提に
する。通常の迷路壁だけでは線が不足する可能性があるため、新しい撮影
治具では前節の既知mm layoutを使う。

### 3.3 レンズ歪みと高さ

homographyは平面射影だけを表し、レンズの非線形歪みを除去しない。また
盤面homographyは床面を表す一方、LEDとPCBは床から高さを持つ。斜め画角
では視差が位置依存になる。homographyの再投影残差が小さくても、画角
内部のmm精度が正しいとは限らない。

2 mm級の調整へ進む前に、実際の解像度・fps・cropでカメラ内部パラメータ
を求めてundistortし、既知LED高さのray-plane補正を行う。さらに、盤面の
複数位置・複数yawへ一時的な基準治具を置き、通常走行では使わないAR
マーカなどをground truthとして絶対位置・yaw誤差をhold-out評価する。

固定架台では、多フレーム平均した静的homographyの方が毎フレームの
corner noiseを減らせる場合がある。本番実装では静的校正を基本とし、
固定マーカは架台移動の監視と再校正に使う方式も比較する。

## 4. 機体マーカなしの姿勢推定

### 4.1 現在実装している方法

`markerless_trajectory.py`の現在の有効フレームには、次がすべて必要で
ある。

1. ID 5、4、6による盤面homography、または最大5フレームのfallback
2. そのフレームで250 canonical pixel以上の緑PCB component
3. 緑PCB近傍で得られる背景差分の前景cluster

既定では、単一の赤componentを追跡点と有向yaw cueにする。赤cueが
見つからないときは前景principal axisを前フレームyawへ連続化するが、
現在はcue lever arm、直近cue距離、axis anisotropy、最大yaw rateを
検査し、`heading_valid`を位置の`pose_valid`と別にCSVへ出す。したがって
現在の実装は「任意の機体輪郭だけで追跡できる」ものではなく、「緑PCBを
毎フレームseedとして使い、単一赤LEDを優先する」ものである。

背景は動画全体から等間隔に選んだ41フレームのmedianで作る。同じ場所を
機体が半数以上のsampleで占有すると機体が背景へ入り、検出に失敗する。
run clipだけを使う場合は開始・終了静止を動画の大半にせず、機体が各
画素を一時的にしか占有しないようにする。推奨構成ではカメラと露光を
変えずに空迷路clipを先に撮り、`--background-video`で指定する。

固定マーカをまだ置けない机上確認用として
`desk_green_pair_probe.py`も用意した。現Nightfall機体では緑PCBが離れた
2 componentとして見えるため、component面積、2点間隔、前frameからの
移動量でpairを選び、その中点と無向長軸を画像pixelで出す。Xiaomi純正
Cameraの1080p/240 fps机上clipでは、手による部分遮蔽を含む2,823/2,823
frameでpairを検出し、間隔はmedian 148.6 px、p95 164.1 pxだった。

この結果は「機体マーカなしでも現在の画角・画質でPCBが追える」ことを
示す。ただし机上probeには盤面homography、mm尺度、レンズ補正、前後を
決める有向cue、ground truthがない。画像内で約972 px移動した軌跡を
mm精度と解釈してはならず、ターン調整には固定外周マーカを入れた本来の
`markerless_trajectory.py`を使う。

### 4.2 推奨する次段

輪郭長軸だけでは前後が180度曖昧で、影や反射にも弱い。機体にARマーカを
貼らずに精度を上げる順序は次とする。

1. 既存LEDのうち、前後または三角形になる2〜3灯を既知配置で点灯する。
2. 各LEDを個別追跡し、剛体配置から位置とyawを求める。
3. LED欠落時だけ、PCB輪郭とmotion modelを短時間fallbackに使う。

複数LED solverはまだ実装していない。現在の単一赤component trackerへ
同色の左右LEDを同時点灯するとcomponentの選択・切替が起こり得るため、
現ツールでは1灯だけを使う。

LEDから車軸中心などの制御基準点への変換では、オフセットの向きに注意
する。`turn_video_tune.py`の`--anchor-right-mm`と
`--anchor-forward-mm`は、**制御基準点から動画の追跡点へ向かうベクトル**
を機体右・前方向へ分解した値である。

たとえば追跡するLEDが制御基準点の23 mm後方にある場合、
`--anchor-forward-mm -23`とする。ツールは

```text
control_reference = tracked_point - anchor_right * right_axis
                                  - anchor_forward * forward_axis
```

で基準点へ戻す。逆向きの「LEDから制御基準点」を渡さない。
`--cue-yaw-offset-deg`は、前景重心から赤cueへ向く角度と機体前方yawの差で
あり、別に0/90/180/270度の静止姿勢で校正する。

## 5. 動画時刻とtrace同期

### 5.1 PTS QAの限界

`video_timing_qa.py`は、encoded frame PTSの単調性、median cadence、
大きなgapに加え、各decoded frameを幅最大192 pxへ縮小してgrayscale化
したfingerprintが完全一致する隣接frameも確認する。原寸・全色pixelの
byte比較ではない。`--expected-fps`は必須である。`frame / fps`ではなく、
必ず実PTSを使う。

ただし、fingerprintが少し異なる複製・補間frame、sensor側のdrop、stock
slow-motionのcapture metadataまでは判定できず、縮小・grayscale化で差が
失われる場合もある。PTS/content QAは必要条件であり十分条件ではない。
正式recorderではsensor timestamp、frame duration、露光、ISO、
rolling-shutter skewとencoded PTSの対応をsidecarへ保存する。

`markerless_trajectory.py`はffprobeがない、timestamp数がdecode frame数
と異なる、またはPTSが非単調なら停止する。`frame_index / container_fps`
への黙ったfallbackは行わない。

### 5.2 現在使えるmotion同期

ターン動画のyawを微分した`video_omega_dps`と、traceの
`real_omega_mdps / 1000`を相互相関する。直線主体なら動画速度と
`real_velocity_mm_s`を使う。

```text
trace_relative_s = scale * video_pts_s + offset_s
```

短い単一ターン試験ではscaleを1に固定する。数十秒以上で開始・終了の
独立した特徴がある場合だけ`--estimate-drift`を診断的に使う。

相関だけでは正しい同期を証明できない。似たターンが複数あると別のpeak
を選ぶことがある。Nightfallと動画はともにCCW正なので、既定は
`--yaw-sign same`であり、符号の自動探索は診断用に明示した場合だけ行う。
さらに短いoverlap候補を除外し、80 ms以上離れた第二候補との相関margin、
signal gainをgateする。`sync_report.json`では少なくとも次を確認する。

- `correlation >= 0.7`
- 設計した座標系なら`video_sign`が期待どおりであること
- `gain_trace_per_video`、bias、RMSEが単位・scaleの誤りを示さないこと
- 選ばれたoffsetが目視した同一ターンに対応すること

`video_sign = -1`を単に許容して自動調整へ進めず、yaw座標の校正を直す。
出力`fused.csv`は同期信号が欠けた行を含む元traceの全CSV data row・全列
を保持し、各行へ動画列を追加する。元CSVの`#key=value` comment metadata
は`sync_report.json`の`trace_metadata`へ移す。空行、key/valueでない
comment、commentの順序は`fused.csv`へ複製しない。

### 5.3 将来追加するLED同期

trace開始後、モータ始動前に既知のLEDパターンを約0.5秒表示し、停止後
にも同じパターンを出す。firmware tickとsession IDをCSV metadataへ
記録する。

```text
#video_sync_start_tick_ms=...
#video_sync_slot_ms=40
#video_sync_code=...
#video_session_id=...
```

開始・終了の二点があればoffsetとclock driftを独立に検証できる。姿勢
推定に使うLEDを点滅する場合、その静止同期区間はpose評価から除外する。
このfirmware変更と動画側decoderは未実装である。

## 6. ホスト環境とAndroid probe

2026-07-30時点の開発Macには次を導入済みである。

- FFmpeg/ffprobe 8.1.2
- Android Platform Tools/adb 37.0.1
- OpenJDK 17.0.20
- Android API 36 / build-tools 36.0.0
- Gradle wrapper 9.4.1 / Android Gradle Plugin 9.2.0
- `<repo>/.venv-vision`
  - OpenCV contrib 4.12.0
  - NumPy 2.0.2

OS側のFFmpeg導入後、Python環境は再現用scriptで構築する。

```sh
brew install ffmpeg
brew install --cask android-platform-tools
brew install openjdk@17 gradle
brew install --cask android-commandlinetools
env JAVA_HOME=/opt/homebrew/opt/openjdk@17/libexec/openjdk.jdk/Contents/Home \
  ANDROID_HOME=/opt/homebrew/share/android-commandlinetools \
  sdkmanager 'platforms;android-36' 'build-tools;36.0.0'
tools/vision/setup_host.sh
```

端末を接続した最初の確認:

```sh
adb devices -l
SERIAL=REPLACE_WITH_ADB_SERIAL
adb -s "$SERIAL" shell getprop ro.product.model
adb -s "$SERIAL" shell dumpsys media.camera > /tmp/media-camera.txt
```

`dumpsys`だけではCamera2の全HFR組合せを確実に判定できないため、
`tools/vision/android_camera_probe/`に静的capability probeを用意した。
現行probeはversion 0.2.0、version code 2で、端末なしでdebug APKのbuild
まで確認済みである。

```sh
tools/vision/android_camera_probe/build_debug.sh
adb -s "$SERIAL" install -r \
  tools/vision/android_camera_probe/app/build/outputs/apk/debug/app-debug.apk
tools/vision/android_camera_probe/collect_report.sh \
  "$SERIAL" /path/to/session-artifacts
```

collectorが固有nonce付きでアプリをforce-startし、一致する新規reportだけを
取得するため、手動の`am start`や`Refresh camera report`は不要である。
初回は端末をunlockし、表示されたcamera permissionを許可する。2台接続時
もinstall、shell、collectのすべてで同じ`SERIAL`を明示する。

このAPKは録画せず、静的capabilityだけを列挙する。各CaptureResultを
JSON sidecarへ保存するrecorderは次段の実装である。

## 7. セッション構造と現パイプライン

動画とログは原則リポジトリ外へ置く。誤ってrepo直下へ作った場合に備え
`/sessions/`も`.gitignore`しているが、checkout容量を増やさないため
外部rootを使う。

```sh
export NIGHTFALL_VISION_SESSIONS=/path/outside/nightfall-fw/nightfall-sessions
session_dir="$NIGHTFALL_VISION_SESSIONS/20260730_120000_large_r90_mode2"
mkdir -p "$session_dir/analysis"
```

現在の出力構造は次である。

```text
$NIGHTFALL_VISION_SESSIONS/
  20260730_120000_large_r90_mode2/
    manifest.json
    board_layout.json
    empty_background.mp4
    capture.mp4
    trace.csv
    analysis/
      timing_report.json
      trajectory.csv
      trajectory.png
      trajectory_topview.mp4
      reference_topview.png
      qa_report.json
      calibration.json
      fused/
        fused.csv
        sync_report.json
      turn_report.json
```

`manifest.json`の自動生成は未実装である。手作業でも最低限、端末modelと
build fingerprint、camera ID、解像度/fps、codec/bitrate、要求・適用
露光/ISO/focus/stabilization、固定マーカlayout、照明、video SHA-256、
firmware SHA/dirty、machine unit、params、mode/case/turn code、battery、
fan、タイヤ状態、traceファイルを記録する。

### 7.1 動画時刻

```sh
./.venv-vision/bin/python \
  tools/vision/video_timing_qa.py \
  "$session_dir/capture.mp4" \
  --expected-fps 120 \
  --fps-tolerance-percent 1.0 \
  --maximum-gap-rate 0.001 \
  --maximum-cadence-deviation-percent 10 \
  --maximum-cadence-deviation-rate 0.001 \
  --maximum-content-duplicate-rate 0.001 \
  --report-json "$session_dir/analysis/timing_report.json"
```

exit code 0でも、前述のsensor frameと複製frameの限界は残る。

### 7.2 マーカレス軌跡

次は、標準180 mmセルを4×4区画、90 mm解析pitchで映し、実測layoutと
空迷路clipを使う例である。

```sh
./.venv-vision/bin/python \
  tools/vision/markerless_trajectory.py \
  "$session_dir/capture.mp4" \
  --output-dir "$session_dir/analysis" \
  --board-layout "$session_dir/board_layout.json" \
  --background-video "$session_dir/empty_background.mp4" \
  --initial-yaw-deg 180 \
  --cue-yaw-offset-deg 180 \
  --maximum-missing-fraction 0.01 \
  --maximum-heading-invalid-fraction 0.01
```

layoutの固定マーカ幾何、green PCB、single red cueの条件を満たす試験
だけに使う。`qa_report.json`のtimestamp source、homography、
`accepted_cue_lever_arm_px`、heading validityを次の工程前に確認する。

### 7.3 traceとのmotion同期

```sh
./.venv-vision/bin/python \
  tools/vision/fuse_trace_video.py \
  "$session_dir/analysis/trajectory.csv" \
  "$session_dir/trace.csv" \
  --minimum-correlation 0.70 \
  --yaw-sign same \
  --minimum-correlation-margin 0.02 \
  --minimum-signal-gain 0.5 \
  --maximum-signal-gain 2.0 \
  --output-dir "$session_dir/analysis/fused"
```

短い試験ではdriftを推定しない。長時間走行でも、同形turnが複数ある
場合はmotion相関だけで自動確定しない。

### 7.4 終了姿勢の反復評価と候補

現`turn_video_tune.py`が安全に解釈できるtrialは、現在のturn model区間
と一致する専用試験で、開始前と終了後に150 ms以上の静止poseがあるもの
に限る。進入直線、複数turn、脱出直線を含む連続走行に自動速度閾値を
使うと、モデル外の移動までendpointへ含まれる。

ツールは各150 ms pose windowについて、`--minimum-pose-window-coverage`
で指定した時間coverageと、`--maximum-pose-window-speed-mm-s`で指定した
median速度をhard gateする。以下の0.8では各側120 ms以上が実データに含ま
れ、median速度が10 mm/s以下でなければ候補生成前に停止する。これは実際
に150 ms以上の静止区間を撮影する運用要件を緩和するものではない。

`--start-s/--end-s`で走行中の内部turnだけを切る場合も、現在の150 ms
medianは移動中のposeを平均してbiasを生む。この用途は、trace位相境界へ
poseを補間する実装を追加するまでparameter fitへ使わない。

5回の同一専用trialについて、QA基準を明示的にgateする例:

```sh
anchor_forward_mm=-23.0  # 例のみ。制御基準点→追跡LEDを実測して置換する
anchor_right_mm=0.7      # 例のみ。別機体へコピーしない

./.venv-vision/bin/python \
  tools/tuning/turn_video_tune.py \
  trial01/trajectory.csv \
  trial02/trajectory.csv \
  trial03/trajectory.csv \
  trial04/trajectory.csv \
  trial05/trajectory.csv \
  --runner shortest --mode 2 --code 501 \
  --anchor-forward-mm "$anchor_forward_mm" \
  --anchor-right-mm "$anchor_right_mm" \
  --minimum-valid-fraction 0.99 \
  --minimum-heading-valid-fraction 0.99 \
  --maximum-pose-window-speed-mm-s 10 \
  --minimum-pose-window-coverage 0.8 \
  --minimum-fit-trials 5 \
  --maximum-endpoint-std-mm 2.0 \
  --maximum-yaw-std-deg 1.0 \
  --maximum-turn-yaw-error-deg 30 \
  --vary angle \
  --propose-fit \
  --feedback-gain 0.5 \
  --report-json "$session_dir/analysis/turn_report.json"
```

座標変換は開始yawを`h0`として次である。

```text
dx = x - x0
dy = y - y0
x_right   = sin(h0) * dx - cos(h0) * dy
y_forward = cos(h0) * dx + sin(h0) * dy
theta     = unwrap(yaw - h0)
```

候補生成は現在のsimulation endpointと実測median endpointとの差を
feedback gain分だけ次のsimulation targetへ反映し、既存のbounded fitter
へ渡す。これは誤差が局所的に同じ向きで変化するという仮説に基づく次の
実験候補であり、source、build、flash、走行を変更・実行しない。

## 8. 合格基準

撮影系をparameter候補生成へ接続する前に、少なくとも次を満たす。

- encoded PTSが単調で、期待120 fpsとの差が1%以内
- encoded PTS gap率が0.1%以下
- timestamp sourceが`ffprobe_best_effort_timestamp`
- 画像内容とCaptureResultでも複製・drop・retimingがない
- homographyのp95内部再投影RMSEが2 px以下
- homography fallbackが0
- markerless pose有効率が99%以上
- markerless heading有効率が99%以上
- 単一LEDを使う現構成ではcolour cue率が99%以上
- cue欠落時のaxis fallbackはanisotropyとyaw rate gateを通ること
- 静止時の位置・yaw標準偏差を記録
- temporary ground truthによる画角内の絶対位置・yaw精度を記録
- 同一条件5回の終了位置標準偏差が2 mm以下
- 同一条件5回の終了yaw標準偏差が1度以下
- trace/video同期相関が0.7以上、第二候補marginが0.02以上で、sign、gain、
  offsetも妥当

現ツールはこれらを一括では強制しない。コマンドのexit codeだけでなく、
`timing_report.json`、`qa_report.json`、`sync_report.json`、
`turn_report.json`を人が確認する。特にhomography内部残差は、レンズ
歪みや誤った物理scaleに対する絶対精度保証ではない。

最初の720p/30 fps検証値はこの基準を満たしていない。1080p/120 fps、
狭い画角、短時間露光、複数LED、内部パラメータ補正で改善するかを実機で
評価する。

## 9. 現在の調整範囲と安全

現在の`turn_video_tune.py`がfitへ使うのは終了位置と終了yawだけである。
duration、path length、peak speedはreportへ出すが、軌道形状やtrace残差
をfit objectiveへ使わない。`fuse_trace_video.py`も別のdebug CSVを作る
だけで、parameter候補生成へは接続されていない。

推奨する段階調整は次である。現fitterはin/out offsetも選択できるが、
段階を自動で進めるworkflowはなく、上の初回例では`--vary angle`だけを
明示的に有効化している。2〜4は一連の自動工程として未実装である。

1. 専用単一turn試験の終了yawをangle候補で合わせる。
2. 横・前後終端誤差をin/out offsetで合わせる。
3. 全軌道と時間をvelocity/alpha候補で合わせる。
4. 動画yaw、IMU yaw、左右encoder yawを比較し、slipとmodel誤差を分ける。
5. 各変更後に前段の終了姿勢と反復性を再確認する。

候補ツールはparams編集、build、flash、fan、モータ、走行開始を行わない。
実機操作は`docs/ai/HIL_SAFETY.md`に従う。

- 浮かせ試験では、機体が持ち上げられ確実に固定されたことを確認する。
- ターン軌跡を測る床上・迷路走行は、実行する特定の試験についてユーザー
  が明示的に承認した場合だけ行う。
- Codexが承認なしにfloor runを開始しない。
- firmware、params、コマンド列、fan条件、停止方法を試験前に記録する。

今回の端末なし作業では、モータ・fan・走行・flash・NVM操作を行って
いない。

## 10. 端末到着後の順序

1. Pixel 8とXiaomi 13 Ultraでprobe JSONを取得する。
2. 1080p/120の実時間capture、manual control、sidecar保存が可能なrecorder
   を実装し、短い静止clipで検証する。
3. 実測layout治具とempty-background clipを作り、印刷・設置誤差を測る。
4. 同じHFR modeでlens calibrationとrolling-shutter計測を行う。
5. 一時ground-truth治具で画角内の位置・yaw誤差mapを作る。
6. 単一の低リスクtrialを撮り、動画・trace同期とQAだけを確認する。
7. 5回の反復性が基準内に入ってから候補生成を有効にする。
8. 複数LED、LED二点同期、trace phase segmentation、全軌道fitを追加する。

## 11. 公式資料

- Google Pixel hardware specifications:
  <https://support.google.com/pixelphone/answer/7158570?hl=en>
- Xiaomi 13 Ultra specifications:
  <https://www.mi.com/global/product/xiaomi-13-ultra/specs/>
- Android constrained high-speed capture:
  <https://developer.android.com/reference/android/hardware/camera2/CameraConstrainedHighSpeedCaptureSession>
- Android `StreamConfigurationMap`:
  <https://developer.android.com/reference/android/hardware/camera2/params/StreamConfigurationMap>
- Android video stabilization request:
  <https://developer.android.com/reference/android/hardware/camera2/CaptureRequest#CONTROL_VIDEO_STABILIZATION_MODE>
