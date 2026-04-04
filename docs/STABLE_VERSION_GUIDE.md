# 安定版の運用ガイド

このドキュメントは、nightfall-fw の安定版リリースで何を行っているか、および後から安定版を選んで使う方法を説明します。

---

## 1. 安定版リリースで行っていること

安定版リリースでは、以下の3つを行っています。

### 1.1 Git タグの付与

| タグ | 例 | 意味 |
|---|---|---|
| FW バージョンタグ | `v1.0.0` | ファームウェア全体の版 |
| 安定版タグ | `stable/mini/unit001/s20260404-nationals` | 特定個体の調整済み構成を指すブックマーク |

タグを付けることで、後からそのコミットに正確に戻れます。

### 1.2 安定版 manifest の作成

`stable/<family>/<unit_serial>/<stable_name>/` に以下の3ファイルを置きます。

- **`manifest.yaml`** — 固定情報（FW version, target, HW rev, params profile など）
- **`runtime_settings.yaml`** — 後から編集可能な運用設定（ゴール座標、既定モードなど）
- **`notes.md`** — 構成の概要、調整状況、既知の制約

### 1.3 origin への push

タグとファイルを origin に push し、リモートにもバックアップを残します。

---

## 2. 現在の安定版一覧

| 安定版名 | 機体 | FW | Tune | タグ |
|---|---|---|---|---|
| `s20260404-nationals` | mini unit001 (r1.0) | v1.0.0 | t1.0 | `stable/mini/unit001/s20260404-nationals` |
| `s20260404-nationals` | classic unit001 (r1.0) | v1.0.0 | t1.0 | `stable/classic/unit001/s20260404-nationals` |

---

## 3. 安定版を後から使う方法

### 3.1 安定版のソースに戻る

```bash
# 安定版タグの一覧を確認
git tag -l 'stable/*'

# 安定版のコミットに移動（読み取り専用）
git checkout stable/mini/unit001/s20260404-nationals
```

### 3.2 安定版をビルドして書き込む

```bash
# ビルド
cmake --preset Debug
cmake --build build/Debug --target nightfall_mini_r1_0

# 書き込み
python3 tools/flashing/flash_uart --bin build/Debug/nightfall_mini_r1_0.bin
```

### 3.3 ゴール座標だけ変更して使う

現時点では、ゴール座標はソースコード内にハードコードされています。
安定版をベースにゴール座標だけ変更する手順は以下です。

```bash
# 1. 安定版タグから作業ブランチを作る
git checkout -b temp/goal-change stable/mini/unit001/s20260404-nationals

# 2. runtime_settings.yaml でゴール座標を確認・編集（メモ用）
#    実際の変更はソースコード側で行います

# 3. ゴール座標を変更するソースファイルを編集
#    （現在は mode*.c や search.c 内で定義されています）

# 4. ビルドして書き込み
cmake --preset Debug
cmake --build build/Debug --target nightfall_mini_r1_0
python3 tools/flashing/flash_uart --bin build/Debug/nightfall_mini_r1_0.bin

# 5. 用が済んだら main に戻る
git checkout main
git branch -D temp/goal-change
```

> **補足**: 将来的には、ゴール座標などの運用設定を NVM に保存し、再ビルドなしで変更できるようにする予定です（開発計画 Phase 7）。現時点ではソースコード編集 + 再ビルドが必要です。

> **ツールパス互換**: 既存運用のため `tools/flash_uart` は互換ラッパーとして維持していますが、新規手順では `tools/flashing/flash_uart` を使用してください。

### 3.4 安定版の manifest を確認する

```bash
cat stable/mini/unit001/s20260404-nationals/manifest.yaml
cat stable/mini/unit001/s20260404-nationals/runtime_settings.yaml
cat stable/mini/unit001/s20260404-nationals/notes.md
```

---

## 4. 新しい安定版を作る手順

パラメータ調整が完了し、新たに安定状態になったら、以下の手順で安定版を追加します。

```bash
# 1. main ブランチが最新であることを確認
git checkout main
git pull

# 2. 安定版ディレクトリを作成
mkdir -p stable/mini/unit001/sYYYYMMDD-<short>

# 3. manifest.yaml, runtime_settings.yaml, notes.md を作成
#    既存の安定版からコピーして編集するのが簡単です
cp stable/mini/unit001/s20260404-nationals/manifest.yaml \
   stable/mini/unit001/sYYYYMMDD-<short>/manifest.yaml
#    fw_tag, params_tune_version, description などを更新

# 4. コミット
git add stable/
git commit -m "stable: add mini unit001 sYYYYMMDD-<short>"

# 5. タグ付け
git tag -a stable/mini/unit001/sYYYYMMDD-<short> \
  -m "Stable: mini unit001 sYYYYMMDD-<short> (FW vX.Y.Z, tune tX.Y)"

# 6. push
git push origin main --tags
```

---

## 5. 安定版の構成図

```
nightfall-fw/
├── CMakeLists.txt              ← FW_VERSION を定義
├── params/mini_r1_0/params.h   ← PARAMS_TUNE_VERSION を定義
├── stable/
│   ├── mini/
│   │   └── unit001/
│   │       └── s20260404-nationals/
│   │           ├── manifest.yaml         ← 何が固定されているか
│   │           ├── runtime_settings.yaml ← 後から変えてよい設定
│   │           └── notes.md              ← 人間向けメモ
│   └── classic/
│       └── unit001/
│           └── s20260404-nationals/
│               ├── manifest.yaml
│               ├── runtime_settings.yaml
│               └── notes.md
└── (ソースコード)

Git タグ:
  v1.0.0                                        ← FW 版
  stable/mini/unit001/s20260404-nationals        ← 個体の安定版
  stable/classic/unit001/s20260404-nationals      ← 個体の安定版
```
