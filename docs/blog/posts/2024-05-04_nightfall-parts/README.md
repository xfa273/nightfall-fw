# クラシックサイズDCマイクロマウス"Nightfall"のパーツ紹介

- 公開日（移行元）: `2024-05-04`
- 移行元記事: https://xfa273-backofchirashi.hatenablog.com/entry/2024/05/04/153028

2023年度シーズンは学生大会まで `Ca.161/bis` を使っていましたが，不満点を踏まえて新作 `Nightfall` を開発中です．
このページでは，主に部品構成と購入先をまとめます．

![Nightfall 全体](resources/image-1.jpg)

**Nightfall 試作機の外観**

足回りの詳細は別記事を参照してください。

- [【既製品で作る】クラシックサイズDCマイクロマウスの足回り](https://xfa273-backofchirashi.hatenablog.com/entry/2024/05/09/025442)

## メカ

### モーターとエンコーダ

- Faulhaber `1717T003SR + IEH2-4096` ×2
- 目安: ¥13,000〜20,000 / 個
- 購入先例: RTショップ、新光電子（まとめ買い）
- https://www.rt-shop.jp/index.php?main_page=product_info&products_id=3683

### ギヤホイール

アライさんの記事の構成を参考にしています。

- 参考: https://amac-araisan.blogspot.com/2018/09/blog-post_17.html?m=1
- ミニッツホイール（ナロー +2.0mm）
- ミニッツ ローハイトスリックタイヤ `MZW39-40`
- ホイール側ギヤ: ミスミCナビ `M0.5 42T t2.0 内径8`
- ベアリング: `SMR63ZZ`
- シャフトねじ: M3x12 など（設計依存）

### ピニオンギヤ

- KKPMO `M0.5 13T t1.5 内径1.5` ×2
- https://shop.kkpmo.com/

### モータマウント

- 設計依存（目安: ¥5,000程度）
- CNC切削アルミやMJFナイロン等を利用
- 例: https://jlcpcb.com/

### 吸引ファン

- DMM.make 3Dプリント（高精細アクリル）
- 4個で¥7,000程度
- https://make.dmm.com/print/personal/

### 吸引ファンモータ

- 1020コアレス（一般には8520も選択肢）
- 例: https://ja.aliexpress.com/item/1005004925984006.html

## エレキ

回路構成は前作に近いため，ここでは主要部品のみ記載します。

### マイコン

- `STM32F405RGT6`
- WMMC会員向けには提供品利用可
- https://akizukidenshi.com/catalog/g/g113219/

### 壁センサ

- 赤外線LED: `CL-1KL7`
- フォトトランジスタ: `ST-1KL3A`
- 定番の `SFH-4550` を使う構成も可能

### IMU（ジャイロ）

- `ICM-20689`
- https://www.mouser.jp/ProductDetail/TDK-InvenSense/ICM-20689

### ブザー

- `SMT-0540-S-R`
- https://www.mouser.jp/ProductDetail/PUI-Audio/SMT-0540-S-R

### モータドライバ

- `TB6612`
- https://akizukidenshi.com/catalog/g/g111317

### チップ抵抗・コンデンサ

定数を幅広く揃えるため，セット購入を使うと作業が早いです。

### バッテリー

- 単セルLi-Poを2セル化して運用
- 例: 502030サイズ（5mm×20mm×30mm）

必要ならDiscordやXで質問を受け付ける，という元記事の方針です。
