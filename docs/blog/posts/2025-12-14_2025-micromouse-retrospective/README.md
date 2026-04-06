# 今年のマイクロマウス振り返り

- 公開日（移行元）: `2025-12-14`
- 移行元記事: https://xfa273-backofchirashi.hatenablog.com/entry/2025/12/14/002140

これはマイクロマウスAdvent Calendar 2025の14日目の記事です．

- Advent Calendar: https://adventar.org/calendars/12206
- 前日の記事: 「今年の新型機について」（ロングシリーズさん）

今年から北九州のメーカーへ入社し，地方移住した新社会人としてのマウス開発を振り返ります．

## 開発環境構築

### ハード編

新入社員寮での生活開始に合わせて，デスク環境を強化しました．

- 31.5インチ 4Kモニタ
- 18.5インチ FHD縦モニタ
- ScreenBar など周辺機器

また，寮環境での作業性を上げるために，裏面パッドの実装向けにホットプレートも導入しています．

- [ALIENTEK HP15](https://ja.aliexpress.com/item/1005008093053677.html?channel=twinner)

さらに9x9迷路も購入し，競技環境に近い条件で調整できるようにしました．

### ソフト編

AIコーディングを本格導入し，Windsurfを中心に開発環境を構築しました．

- [Referrals | Windsurf](https://windsurf.com/refer?referral_code=ox6uaybj3yxyfn5u)

特に効果が大きかったのはログ取得・描画ツールの整備で，UART受信→CSV保存→可視化までを自動化でき，調整サイクルが大幅に短縮されました．

## 機体紹介

![Nightfall-mini](resources/image-1.jpg)

**Nightfall-mini 全体像**

今シーズン製作したハーフサイズ機が `Nightfall-mini` です．設計自体は学生時代に行い，組立と実戦投入は社会人になってから進めました．

### コンセプト

前作 `CyberRat` の部品構成を引き継いで無難に動かしつつ，大電流構成で速度を底上げする方針です．

- IMU: `ISM330DHCX`（4000dps）
- モータドライバ: `MP6551`（最大5A）
- モータ: `NFP-D0612-2-3.4`（1.7Ω版）
- バッテリ: Palm Beach Bots 100/180mAh 45C

1セルでも大電流を流せば十分な出力を確保できるという考えで，実測ではターン速度1.4m/s，加速度18m/s^2，吸引力100g程度を確認しています．

- [Palm Beach Bots Li-Po](https://palmbeachbots.com/products/palm-power-2s-100mah-45c-lipo-battery)
- [NFP-D0612](https://nfpshop.com/product/6mm-coreless-motor-high-speed-low-current?srsltid=AfmBOoobms5y1rmxkS3yfLPGHCmOpW_iBEXL1oHp0uhPDKruz6n7D804)

## まとめ

新社会人になって開発時間は減ったものの，環境整備とツール整備で継続できる形を作れた一年でした．

機体情報で知りたい点があれば，今後追記予定です．
