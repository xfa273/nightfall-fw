# Component Type Outlines

KiCad PCB Editor上で、各フットプリントのCourtyard（未定義ならBounding Box）を
抵抗・コンデンサ・LED別のUserレイヤーへ矩形として生成するAction Pluginです。矩形は
フットプリント内の図形として追加されるため、部品の移動・回転・反転に追従します。

同時に、フットプリント内の`F.Silkscreen`図形・文字・フィールドを
`User.5`（`Footprint Silk (No Print)`）へ移動します。基板直下の`F.Silkscreen`図形は
変更しないため、PCB Editorへ直接インポートしたDXFアートワークや、基板直下に配置した
機体名だけを製造用シルクに残せます。

| 種類 | 判定 | レイヤー | 推奨色 |
| --- | --- | --- | --- |
| 抵抗 | `R` + 数字/`_`/`?` | `User.1` | 黄色 `#FFD54F` |
| コンデンサ | `C` + 数字/`_`/`?` | `User.2` | 水色 `#4FC3F7` |
| LED | `LED` + 数字/`_`/`?`、またはLED情報を持つ`D` | `User.3` | 黄緑 `#AEEA00` |

| シルクの用途 | レイヤー | Gerber出力 |
| --- | --- | --- |
| DXFアートワーク・機体名など、実際に印刷するもの | `F.Silkscreen` | 出力する |
| 通常のフットプリントシルク | `User.5` (`Footprint Silk (No Print)`) | 出力しない |

上記以外のIC、コネクタ、ダイオードなどには追加の枠を生成しません。枠がないこと自体を
「その他」の判別情報として使うため、表示の重なりと情報量を抑えられます。

各フットプリントには、生成した枠のUUIDを記録する非表示の
`__component_type_outline__`カスタム属性が追加されます。プラグインを再実行すると、
この属性で特定した生成物をその場で更新します。既存のユーザー作図は変更しません。
枠はロックせずフットプリントの子要素として保持するため、通常どおり部品を選択して
移動できます。
枠線は0.10 mm、CourtyardまたはBounding Boxからの余白は0.15 mmです。

## KiCad 10 (macOS)へのインストール

このディレクトリを次の場所へ配置し、PCB Editorで
`ツール > プラグインを再読み込み`を実行します。

```text
~/Documents/KiCad/10.0/scripting/plugins/component_type_outlines
```

次に`ツール > 外部プラグイン > Component Type Outlines`を実行します。
各Userレイヤーの色はAppearanceパネルで設定し、通常のCourtyard表示は
必要に応じて非表示にしてください。製造用Gerberでは`F.Silkscreen`を選択し、
`User.5`を出力対象および「すべてのレイヤーにプロット」から外してください。

KiCad 10ではAction Pluginからフットプリントへ新しい図形を追加した場合、保存して
基板を開き直すまで表示へ反映されません。初回実行後や新しい部品を追加した後は、
基板を保存して開き直してください。

## 旧版からの一度だけの移行

旧版の`__component_type_outlines__`グループ、または「その他」の生成枠がある基板は、
PCBエディターを閉じてからKiCad付属Pythonで次を実行します。開いた基板から生成図形を
削除するとKiCad 10が異常終了するため、プラグイン本体は安全のため移行を中断します。

```sh
/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3 \
  ~/Documents/KiCad/10.0/scripting/plugins/component_type_outlines/component_type_outlines.py \
  /path/to/board.kicad_pcb
```

移行コマンドは既存のプラグイン生成枠だけを削除し、抵抗・コンデンサ・LEDの
フットプリント追従型枠へ置き換えます。また、フットプリント所有の`F.Silkscreen`だけを
`User.5`へ移し、基板直下のDXFアートワークは`F.Silkscreen`に残します。

KiCad 10.0.3で動作確認しています。
