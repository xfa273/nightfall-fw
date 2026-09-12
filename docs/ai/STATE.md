# AI Coordination State

このファイルは、Codex の現在の運用状態を記録する正本です。

## 目的

- 会話履歴ではなく、リポジトリ内の状態ファイルで運用を継続可能にする
- 実装・調査・検証の実行方針を固定し、再現性を高める

## 現在の構成

- 実行者: Codex
- 実行方式: Codex が本作業ツリーで直接実行
- 主作業: STM32F413共通ファームの `mini_r2_0` / `mini_r3_0` 対応、実機HIL確認、ログ/ツール整備（F405既存機は維持）
- 機体選択: NVM identityの機種・個体IDからハード設定と走行profileを起動時選択。運用は `docs/F413_MACHINE_CONFIG.md`、未登録IDは安全停止
- mini_r3壁距離: 遮光後 `mini-r3-wall-centre-t0.5` / source `3cc013e`。2026-09-12モータ再確認で実機boot `aba28d5 DIRTY=1` / t0.5を確認（この再確認ではflashなし）。実効/保存offset FR82/FL66/R27/L43、校正prefixは試験前後不変。下記0906の値を無断復元しない。機体中心基準FR/FL/合計40..80mm各9点・L/R23..80mm各12点、前壁目標45mm、横壁制御ADC生値、閾値/ゲイン・mini_r2不変。測定時delta/offsetとの対応確認と新LUTの45mm実寸確認は未実施。詳細 `docs/MINI_R3_COMMISSIONING.md`
- mini_r3駆動の未解決事項（2026-09-12）: 浮上固定8V/2Aで左前後進12%・各500msを2回ずつ試し、全てencoder0。ユーザ目視でも全く動かず電流変化も見えない。右は前進+2421/後退-2146。機体設定/PWM端子設定に不整合なし、波形/ドライバ入力は未測定。左配線・モータ・U2/R35周辺を電源OFFで確認する段階。追加高出力・固定機体の閉ループ旋回は実施しない。最終mode0、全駆動OFF、UART解放済み。FW変更なし。
- mini_r3通電試験中止（同日追記）: ユーザがメカに問題なし・電源投入だけでU2発熱・8V入力約0.23Aと報告。安定化電源とデバッガ両方の切離しを依頼、切離し完了は未確認。以後ライブ操作なし。停止中発熱はU2損傷/実装短絡/EN・GND接続不良/近傍からの伝熱を切分ける必要があり、入力0.23Aは正常判定根拠にならない。電源OFFで外観・モータ切離し後の出力短絡確認を優先し、駆動再開しない。
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
