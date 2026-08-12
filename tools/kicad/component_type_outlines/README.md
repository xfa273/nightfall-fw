# Component Type Outlines

KiCad PCB Editor上で、各フットプリントのCourtyard（未定義ならBounding Box）を
部品種類別のUserレイヤーへ矩形として生成するAction Pluginです。

| 種類 | 判定 | レイヤー | 推奨色 |
| --- | --- | --- | --- |
| 抵抗 | `R` + 数字/`_`/`?` | `User.1` | 黄色 `#FFD54F` |
| コンデンサ | `C` + 数字/`_`/`?` | `User.2` | 水色 `#4FC3F7` |
| LED | `LED` + 数字/`_`/`?`、またはLED情報を持つ`D` | `User.3` | 黄緑 `#AEEA00` |
| その他 | 上記以外 | `User.4` | KiCad標準のピンク `#FF26E2` |

生成物はロックした`__component_type_outlines__`グループにまとめられます。
プラグインを再実行すると、前回の生成物を削除して最新状態へ置き換えます。
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

KiCad 10.0.3で動作確認しています。
