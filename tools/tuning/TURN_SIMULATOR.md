# 実測データを使うターンシミュレータ

`turn_simulator.py` は動画から抽出した位置列で機体応答を同定し、理想軌道・予測軌道・実測を比較するホスト専用ツールです。ブラウザで速度、角加速度係数、実効旋回角、入口・出口距離を変更でき、目標終点に向けたパラメータ探索、JSON保存もできます。ファームウェア、NVM、接続機体は変更しません。

## 起動

Python 3.10以降、NumPy、SciPyを使用します。必要なら仮想環境内に依存を入れてください。

```sh
python3 -m pip install -r tools/tuning/requirements.txt
# 実測モデルなしでも理想軌道の調整画面を使用できます。
python3 tools/tuning/turn_simulator.py serve
```

表示される `http://127.0.0.1:8765` を開きます。外部配信・CDNは不要です。サーバーはlocalhostだけで待ち受けます。

過去の動画データがあるチェックアウトを `--data-root` に指定すると、同梱カタログから実測モデルを生成できます。生動画・CSVはコピーしません。

```sh
python3 tools/tuning/turn_simulator.py inventory \
  tools/tuning/data/historical_mode2_d135_in_manifest.json \
  --data-root /path/to/nightfall-fw-with-sessions

python3 tools/tuning/turn_simulator.py fit \
  tools/tuning/data/historical_mode2_d135_in_manifest.json \
  --data-root /path/to/nightfall-fw-with-sessions \
  --output build/turn_simulator/historical_model.json

python3 tools/tuning/turn_simulator.py serve \
  --model build/turn_simulator/historical_model.json
```

当初のデータ置き場は `/Users/xfa273/workspace/micromouse/nightfall-fw` です。このパスは実行コードに固定していません。モデルJSONには、係数、元データのSHA-256、正規化方法、推定開始時刻と不確かさ、検証指標、適用範囲、画面表示用の実測点が入ります。生成物は `build/` に保存し、Git管理しません。

## CLIでの調整

```sh
# 右135度、500 mm/sの予測。角度は「実効指令角」です。
python3 tools/tuning/turn_simulator.py simulate \
  --model build/turn_simulator/historical_model.json \
  --velocity 500 --alpha 9750 --angle -135 --dist-in 4 --dist-out 17.5 \
  --output build/turn_simulator/prediction.json

# 90 mmピッチの右135-in目標の例。解が範囲内で得られるとは限りません。
python3 tools/tuning/turn_simulator.py tune \
  --model build/turn_simulator/historical_model.json \
  --target-x 90 --target-y 45 --target-theta -135 \
  --output build/turn_simulator/candidate.json
```

座標は **右 +X、前 +Y、左旋回 +角度**。`t=0` は入口直線の開始です。`alpha` はファームのプロファイル係数であり、最大角加速度そのものではありません。通常の丸め係数1.2ではコサイン波の最大角加速度は約 `1.309 × alpha` です。

現行F413の `angle_accum_mode` が有効な経路は、paramsの補正角ではなく45/90/135/180度を実行します。`--angle` は実際に実行される角度を指定してください。探索器は速度と実効角度を固定し、alphaと入口・出口距離だけを探索します。出力の `velocity/alpha/dist_in/dist_out` は汎用名であり、対応するターンのCフィールドに直接書き込む機能はありません。

校正モデル付きの探索は、alpha・入口・出口距離の観測範囲を境界にします。入口ゼロのモデルは入口ゼロを保持します。数値最適化の `converged` は目標到達を意味しません。`optimization.target_reached`、終点誤差、進行方向の誤差を確認してください。局所探索なので、解が見つからないことは数学的な不可能性の証明ではありません。手入力の予測は範囲外でも可能ですが、外挿項目を表示します。

機体の性能限界が別途測定できている場合、`--limit-velocity` [mm/s]、`--limit-omega` [deg/s]、`--limit-alpha` [deg/s²]、`--limit-lateral-accel` [mm/s²] を `simulate/tune/serve` に指定できます。要求量と限界の比率を表示し、探索にも超過ペナルティを与えます。これらは推測で補いません。未指定では性能余裕は「未評価」です。限界内でも壁との接触やモータ飽和の成立性は判定できません。

## モデルと検証

既存 `turn_tune.py` の入口直線・1 kHzコサイン角速度指令・出口直線を利用します。単位応答は従来の理想軌道と一致します。その上に速度ゲイン、速度一次遅れ、角速度ゲイン・遅れ・むだ時間、機体角に対する進行方向の一次遅れを重ねます。

動画位置だけでは角速度遅れと進行方向遅れを分離できません。既定の同定対象は **速度ゲインと進行方向の実効遅れの2個**です。機体角は指令通りという仮定に固定します。UIは未同定の機体角と、位置から推定する進行方向を区別します。この場合の終点角度探索は進行方向に対して行います。実際の機体角が校正されたデータだけが、`fit --vary velocity_gain,yaw_tau_s,lateral_tau_s` などの角応答同定に使用できます。

各動画を同じ重みで最小二乗し、全繰返しを同じパラメータ条件のグループにまとめて、1条件を丸ごと除外する交差検証を行います。同一動画の再処理を独立な試行として数えず、接触記録、SHA不一致、非単調時刻、50 ms超の欠測を除外します。3条件未満では交差検証成立とは表示しません。モデルJSONには全foldと全試行のRMS・95%点・最大・終点誤差を残します。

### 2026-09-21の既存コーパス検証

元の保存先には185個の `trajectory.csv`、471個の撮影レポート、398個のtrace CSVがありました。ただし、全ての走行パラメータが自動的に確定できるわけではありません。今回取り込んだ正確な対応付けのあるカタログは、9バッチ・27本の非接触動画、8パラメータ条件、別に1件の接触記録です。繰返し条件を統合し、入口ゼロ6本を別条件として除いた **21本・6条件**で以下を得ました。

| 指標 | 結果 |
|---|---:|
| 対象 | mini_r2_0、右135-in、500 mm/s |
| alpha範囲 | 6888〜10250 deg/s² |
| 理想モデルの軌跡RMS | 7.128 mm |
| 全データ学習時のRMS | 1.720 mm |
| 条件を除外した交差検証RMS | 1.746 mm |
| 交差検証中の最大位置誤差 | 4.964 mm |
| 同定速度ゲイン | 1.02439 |
| 同定進行方向の実効遅れ | 9.420 ms |

これは**正規化したターンコアと80 msの後続区間**の位置誤差です。入口・出口直線の絶対配置や壁との距離の検証ではありません。過去のラベル高さ補正・撮影校正契約が不足しており、推定した開始時刻にも不確かさがあります。係数はタイヤ特性や制御遅れを個別に測った値とは扱えません。右135度500 mm/s以外、mini_r3_0等の別機体への精度は未検証です。

`safe_recommendation_available` と `qualification.safety_qualified` は常にfalseです。現在のコーパスからは「シミュレータの値でそのまま迷路走行できる」との判定はできません。上の右135-in目標を学習範囲内だけで探索した例でも、終点誤差約5.6 mmが残り、`target_reached=false` でした。モデルが理想軌道より改善することと、実機にそのまま使えるパラメータが得られることを別々に報告します。

## 新しいデータの追加

`nightfall_turn_dataset_v1` のJSONでCSVと実行パラメータを明示します。自動で現在のparamsを過去動画に割り当てることはありません。パスはmanifest基準（または `--data-root` 基準）です。

```json
{
  "schema": "nightfall_turn_dataset_v1",
  "machine": "mini_r2_0_unit001",
  "constants": {"rounding_scale": 1.2, "omega_cap_deg_s": 2200},
  "runs": [{
    "id": "recording-001",
    "csv": "run001/trajectory.csv",
    "csv_sha256": "REPLACE_WITH_EXACT_SHA256",
    "contact": false,
    "params": {"velocity_mm_s": 500, "alpha_deg_s2": 9000,
               "signed_angle_deg": -135, "dist_in_mm": 4, "dist_out_mm": 20},
    "sample_mode": "full",
    "coordinate_frame": "board",
    "segment": {"start_s": 1.2, "end_s": 1.6, "heading_deg": 90},
    "heading_trusted": false,
    "provenance": {"fw_git_sha": "EXACT_COMMIT", "fw_git_dirty": false,
                   "calibration_sha256": "EXACT_CALIBRATION_HASH",
                   "angle_policy": "angle_accum_exact", "fan": "record actual setting"}
  }]
}
```

CSVは `t_s` / `time_s` / `video_pts_s`、`x_mm,y_mm`、任意の `theta_deg` / `yaw_deg_unwrapped` を読みます。`columns` で `time,x,y,theta` の列名を指定できます。`pose_valid` と `heading_valid` があれば尊重します。`board` のheadingは+board-Xから反時計回り、`local` は+Yから左正です。segment開始時の位置を原点にします。原点を別に固定する場合は `origin_x_mm,origin_y_mm` をsegmentに入れます。

`sample_mode=core` ではspecの入口・出口を0にし、元の実行パラメータを別の `original_params` に記録できます（速度・alpha・実効角度は一致させる）。これにより、入口距離が正の走行から切り出したコアでも入口ゼロの実行と混同しません。

機体、プロファイル定数、実行条件の異なるデータは分けて校正してください。定数または機体名の混在は拒否します。制御ゲイン・壁制御・電池・ファン・タイヤ・床条件はprovenanceに記録し、同じ応答とみなせる条件に絞ってください。次の検証には、既存資料の撮影校正を満たした動画、実効角・params・FW SHA/dirty、同期trace、左右の繰返し、速度とalphaの範囲、接触有無が必要です。迷路への適用にはさらに実際の機体外形と壁配置に対する全軌道の検証が必要です。

## 回帰検証

```sh
python3 -m unittest discover -s tools/tuning/tests -v
python3 -m unittest discover -s tools/tuning -p test_turn_dynamics.py -v
python3 -m py_compile tools/tuning/*.py
git diff --check
```

プロファイル検証は実際のF413 runner/control C関数を取り出してホストCコンパイラで実行し、Pythonのコサイン指令との符号・上限・丸めの一致を確認します。合成データの検証では、学習に使わないalphaへの予測、繰返しを跨ぐ学習漏れ防止、パラメータ探索、無効入力・時刻・SHAの拒否を確認します。
