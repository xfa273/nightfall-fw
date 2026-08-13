# Component Type Outlines

KiCad PCB Editor上で、各フットプリントのCourtyard（未定義ならBounding Box）を
部品種類別のUserレイヤーへ矩形として生成するAction Pluginです。矩形は
フットプリント内の図形として追加されるため、部品の移動・回転・反転に追従します。

| 種類 | 判定 | レイヤー | 推奨色 |
| --- | --- | --- | --- |
| 抵抗 | `R` + 数字/`_`/`?` | `User.1` | 黄色 `#FFD54F` |
| コンデンサ | `C` + 数字/`_`/`?` | `User.2` | 水色 `#4FC3F7` |
| LED | `LED` + 数字/`_`/`?`、またはLED情報を持つ`D` | `User.3` | 黄緑 `#AEEA00` |
| その他 | 上記以外 | `User.4` | KiCad標準のピンク `#FF26E2` |

各フットプリントには、生成した枠のUUIDを記録する非表示の
`__component_type_outline__`カスタム属性が追加されます。プラグインを再実行すると、
この属性で特定した生成物をその場で更新します。既存のユーザー作図は変更しません。
枠線は0.10 mm、CourtyardまたはBounding Boxからの余白は0.15 mmです。

## KiCad 10 (macOS)へのインストール

このディレクトリを次の場所へ配置し、PCB Editorで
`ツール > プラグインを再読み込み`を実行します。

```text
~/Documents/KiCad/10.0/scripting/plugins/component_type_outlines
```

次に`ツール > 外部プラグイン > Component Type Outlines`を実行します。
各Userレイヤーの色はAppearanceパネルで設定し、通常のCourtyard表示は
必要に応じて非表示にしてください。

KiCad 10ではAction Pluginからフットプリントへ新しい図形を追加した場合、保存して
基板を開き直すまで表示へ反映されません。初回実行後や新しい部品を追加した後は、
基板を保存して開き直してください。

## 旧版からの一度だけの移行

旧版の`__component_type_outlines__`グループがある基板は、PCBエディターを閉じてから
KiCad付属Pythonで次を実行します。開いた基板から旧グループを削除するとKiCad 10が
異常終了するため、プラグイン本体は安全のため移行を中断します。

```sh
/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3 \
  ~/Documents/KiCad/10.0/scripting/plugins/component_type_outlines/component_type_outlines.py \
  /path/to/board.kicad_pcb
```

移行コマンドは旧版の基板直下の枠および開発版のグループ形式だけを削除し、
フットプリント追従型の枠へ置き換えます。

KiCad 10.0.3で動作確認しています。
