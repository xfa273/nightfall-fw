# mini_r3 mode1〜4 調整値保存点

2026-10-04、ユーザが「一通りmode1〜4でそれなりに走ることを確認」と報告し、
現在の調整値の保存を依頼した。継続して調整しているmini_r3_0_unit002の保存点とする。
調整値版は **`mini-r3-mode1-4-t0.36`**。
Git tagは **`stable/mini/unit002/s20261004-mini-r3-mode1-4`**。

## 保存範囲

`params/mini_r3_0/` の現在値を保存し、今回追加の数値調整はしない。
タイヤ径、探索・最短の全配列、各ゲイン、壁切れ・前壁補正、LUTと校正元CSVを
共通FWソースと一緒にタグへ固定する。ファイル同一性はmanifestのSHA256で確認できる。
mode5/6の作業中の値もそのまま保存するが、今回の実走確認範囲には含めない。

直前のcommit `b6c83fb` に対する主な保存差分と現設定は次の通り。

- `D_TIRE = 14.3mm`。
- mode2のcase3〜6の `acceleration_straight = 2000mm/s²`。
- mode3の小回り角加速度48000deg/s²、後offset3.0mm、大90後3.5mm、大180後3.0mm。
  case1〜7の `acceleration_straight = 20000mm/s²`。
- mode4の小回り角加速度74600deg/s²、後offset3.5mm。
  大90は41000deg/s²・前2.0mm/後11.4mm、大180は31000deg/s²・前0mm/後13.0mm。
  case1〜7の `acceleration_straight = 25000mm/s²`。dash/斜めの詳細は配列を参照。
- mode4の通常直進・短方式壁切れ追加距離は **8mm**。
  共通 `dist_wall_end` はmode3=-15mm、mode4=-23mmを維持。
- fanはmode3=50%、mode4=70%。全ゲインとmotion policy `0x7FFF` は維持。
  連続前壁・前後offset統合・短方式全域化・大180追従FF・最終停止修正を含む。
- mode5/6の角加速度・後offset、およびmode5の一部case加速度も作業ツリーの値を保存。
  詳細はタグの最短配列が正本であり、保存と走行資格を区別する。

探索配列・LUTは今回数値差分なし。mode1も既存ソースを含めて復元できる。
mini_r2側の未コミット調整、書込ツール、CAD等の無関係な変更はこの保存点に含めない。
生成route表もmini_r2側の未コミット差分に由来するため、今回commitへ混ぜない。

## 確認範囲

実走の根拠はユーザ報告。今回Codexは走行・ログ採取・実機binary読出しを行っていない。
mode1〜4の全case、全コース、全電源条件、斜め走行を網羅した確認ではない。
mode5〜7へ確認範囲を拡張しない。

保存時には機体profile/個体選択とゴール停止の既存host試験（ASan/UBSan）、
route precompute14210 checks、F413/F405 Debug build、diff checkを実施しPASS。
これらはsource/buildの確認であり、新しい実走結果ではない。
UART/ST-LINK/flash/reset/motor/fan/run/NVMコマンドは一切実行していない。

## 復元

作業中の設定を上書きせず、保存タグから別のチェックアウトを作成してbuildする。

```sh
git worktree add --detach ../nightfall-mini-r3-t0.36 stable/mini/unit002/s20261004-mini-r3-mode1-4
cd ../nightfall-mini-r3-t0.36
cmake --preset Debug -DNIGHTFALL_TRACE_F413_USE_UART=1 -DNIGHTFALL_F413_UART_BAUD_RATE=921600 -DNIGHTFALL_F413_DESTRUCTIVE_NVM_DIAGNOSTICS=OFF
cmake --build --preset Debug-stm32f413
```

現在のFWに値だけ移す場合は、保存タグの`params/mini_r3_0/`を参照する。
当時の挙動を再現する際は、motion policyや制御実装との組合せを保つためタグ全体を使用する。
単一のF413 binary内で、既存identityがmini_r3_0/unit002のprofile・ハードウェア設定を選択する。

この保存点はソース調整値の保存であり、identity/FRAMの完全バックアップではない。
校正blobやmazeを消去・再登録する操作は含まない。
`runtime_settings.yaml`は運用参照用で、現FWが自動読込みするものではない。
旧保存点`s20260921-mini-r3-2s-fan-off`もそのまま残す。
