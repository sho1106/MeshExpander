# MeshExpander — 技術ガイド（日本語版）

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](../LICENSE)

高性能・凹形状対応の 3D メッシュ膨張ライブラリ。
入力メッシュを距離 `d` だけ確実に包摂する凸多面体集合を生成します。

---

## 目次

1. [概要](#概要)
2. [ビルド手順](#ビルド手順)
3. [使い方](#使い方)
4. [アルゴリズム詳細](#アルゴリズム詳細)
5. [テスト](#テスト)
6. [Stanford Bunny テスト](#stanford-bunny-テスト)
7. [設計原則](#設計原則)
8. [ファイル構成](#ファイル構成)

---

## 概要

MeshExpander は、3D メッシュを指定した距離 `d` だけ「外側に膨張させる」ライブラリです。

**主な特徴:**
- **保守性保証（Zero Shrinking）**: 膨張後メッシュは元形状 + 距離 `d` を必ず包摂
- **凹形状対応**: Stanford Bunny のような複雑な凹形状も正確に処理
- **高精度**: 面法線ベースの適応的半空間生成で球 +3.3%、円柱 +1.2%、円錐 +1.3% の精度
- **NaN/Inf 安全**: 全浮動小数点エッジケースを Safety Margin で処理
- **依存ゼロ**: Eigen のみ（CMake FetchContent で自動取得）

---

## ビルド手順

### 必要環境

| ツール | バージョン |
|---|---|
| CMake | ≥ 3.16 |
| C++ コンパイラ | C++17 対応（MSVC 2019+, GCC 9+, Clang 10+） |
| インターネット接続 | Eigen・GoogleTest の自動取得に必要（初回のみ） |

### Windows (MSVC)

```powershell
git clone https://github.com/sho1106/MeshExpander.git
cd MeshExpander

cmake -S . -B build
cmake --build build --config Release

# テスト実行
cmake --build build --config Release --target check
```

### Linux / macOS

```bash
git clone https://github.com/sho1106/MeshExpander.git
cd MeshExpander

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build

# テスト実行
cmake --build build --target check
```

---

## 使い方

### 凹形状メッシュの膨張（推奨）

```cpp
#include "expander/RobustSlicer.hpp"
#include "expander/StlReader.hpp"
#include "expander/StlWriter.hpp"

// STL 読み込み
expander::Mesh input = expander::StlReader::read("part.stl");

// 5mm ボクセルセルサイズ、2mm 膨張
auto slicer = expander::RobustSlicer::withCellSize(0.005);
expander::Mesh result = slicer.expandMerged(input, 0.002);

// STL 出力（単一統合ファイル）
expander::StlWriter::write("expanded.stl", result, "expanded part");
```

### 凸形状メッシュの膨張（高精度・高速）

```cpp
#include "expander/ConservativeExpander.hpp"

expander::ConservativeExpander expander;
expander::Mesh result = expander.expand(input, 0.002);
```

### 個別多面体リストの取得

```cpp
// 各多面体は独立した閉じた凸メッシュ（体積計算が正確）
auto slicer = expander::RobustSlicer::withCellSize(0.005);
std::vector<expander::Mesh> polytopes = slicer.expandMulti(input, 0.002);

double totalVolume = 0.0;
for (const auto& m : polytopes)
    totalVolume += computeVolume(m);  // 発散定理で正確な体積
```

### 解像度モード（適応的セルサイズ）

```cpp
// AABB 最長辺 / resolution でセルサイズを自動決定
expander::RobustSlicer slicer(64);  // resolution = 64
expander::Mesh result = slicer.expandMerged(input, 0.002);
```

---

## アルゴリズム詳細

### ConservativeExpander（凸形状向け）

```
入力メッシュ（頂点 + 面データ）
  │
  1. 面法線抽出
  │    全面法線を収集 → 20°閾値でマージ（近似平行な法線を統合）
  │    面データなし → 26方向固定フォールバック
  │
  2. 半空間生成
  │    各方向 n に対して: D = max(V·n) + d
  │    （全頂点の射影最大値 + 膨張距離）
  │
  3. 堅牢な交差計算
  │    C(k,3) 個の平面三つ組みを ColPivHouseholderQR で解く
  │    全半空間の内側にある点のみ保持
  │    近接頂点の重複排除
  │
  4. メッシュ組み立て
  │    各面の境界頂点を角度でソート → Fan 三角形分割
  │
  5. デノーマライズ
       正規化座標 → ワールド座標へ復元

出力: 単一閉多面体（頂点数 ≦ C(k,3)、入力サイズ非依存）
```

**精度実測値（d = 1.0mm）:**

| 形状 | ratio | over% | 旧26方向との比較 |
|---|---|---|---|
| 球 (R=10-100) | 1.033 | +3.3% | 1.140 → **-11%** |
| 円柱 | 1.012 | +1.2% | 1.055 → **-4%** |
| 円錐 (H=3R) | 1.013 | +1.3% | 2.2 → **-107%** |

### RobustSlicer（凹形状向け）

```
入力メッシュ（任意のトポロジー）
  │
  1. VoxelGrid::build()
  │    三角形-AABB 保守的ラスタライズ
  │    cellSize: 固定値 または AABB最長辺 / resolution
  │
  2. VoxelGrid::greedyMerge()
  │    占有ボクセルを非重複の直方体集合に分解
  │    凹部の空気ボクセルは空のまま（凹形状保持）
  │
  3. Per-Box 局所膨張（各直方体に対し）
  │    a. 直方体と AABB が交差する面を選択
  │    b. 局所面法線を 20°閾値でマージ
  │    c. D_i = max(局所頂点 · n_i) + d
  │    d. ClippingEngine::clip(expandedBox, half-spaces)
  │
  4. 出力
       vector<Mesh>: 各直方体に対する凸多面体
       expandMerged(): 全多面体を連結した単一 STL ファイル
```

**保守性の証明:**
頂点 v がボクセル内 → ボックス B に包含 → expandedBox の内側
→ v·n_i ≤ D_i - d（全半空間に対して）→ clip(expandedBox, hs) の内側に余裕 d で存在

### ClippingEngine

半空間クリッピングエンジン。各ボックスを局所的な平面群で切り取る。

- `kSafetyMargin = 1e-6`: 全半空間オフセットに加算（外側方向に）
- `kOnPlaneEps = 1e-5`: 平面上判定の閾値
- 並列法線の重複排除: `dot > 1 - 1e-8` なら tighter（小さい D）を保持

### VoxelGrid

スパースボクセルグリッドによる三角形-AABB 交差判定とグリーディボックスマージ。

- `build()`: 三角形の AABB でボクセルを保守的に塗る（False negative なし）
- `greedyMerge()`: 大きい直方体優先でボクセルを統合（凹部を保持）

---

## テスト

```bash
cd build/Release

# ユニットテスト（高速、約 1 秒）
./unit_tests

# 統合テスト（精度評価、約 10 秒）
./integration_tests
```

### テスト構成

| スイート | テスト数 | 検証内容 |
|---|---|---|
| ConservativeExpander | 8 | 保守性・堅牢性・頂点数上界 |
| MathUtils | 10 | 正規化・方向生成・マージ |
| ClippingEngine | 7 | 半空間クリッピング正確性 |
| VoxelGrid | 8 | ボクセル化・グリーディマージ |
| RobustSlicer (unit) | 8 | 空入力・L字保守性・NaN 安全性 |
| ShapeExpansion | 4 | 球・円柱・円錐の精度評価 |
| ConcaveExpansion | 4 | L字・C字の保守性 + 体積比較 |
| StanfordBunny | 3 | 保守性 + 膨張距離 ≥ d + 統合 STL |

**合計 44 テスト — 全 PASS**

---

## Stanford Bunny テスト

Stanford Bunny（スタンフォードバニー）を使った実世界凹形状テスト。

**結果（cellSize = 5mm, d = 1mm）:**

| メッシュ | 多面体数 | robust / conservative | 保守性 |
|---|---|---|---|
| res4 (948面) | 365 | 0.50 (−50%) | 99.8% |
| res3 (3851面) | 578 | 0.42 (−58%) | 99.5% |
| フル (69451面) | 616 | 0.36 (−64%) | 99.8% |

テストデータのダウンロード: [tests/data/README.md](../tests/data/README.md)

---

## 設計原則

### 1. Zero Shrinking（縮小ゼロ）

膨張後メッシュは元形状 + 距離 `d` を必ず包摂する。
Safety Margin（`kSafetyMargin = 1e-6`）を全半空間オフセットに加算することで、
浮動小数点誤差を常に「外側」に逃がす。

### 2. Uniform Scaling（等方スケーリング）

正規化は等方スケーリングのみ使用（`[-1, 1]^3` への写像）。
非等方スケーリングは角度と対角距離を変形するため禁止。

### 3. Numerical Safety（数値安全性）

- 縮退面（法線ノルム < `kEpsilon`）はスキップ
- 並列半空間は tighter な方を残して重複排除
- ColPivHouseholderQR による堅牢な線形系求解

### 4. Input-Size Independence（入力サイズ非依存）

ConservativeExpander の出力頂点数は `C(k, 3)` で上界。
入力メッシュの密度に関わらず出力サイズが一定（高密度メッシュでも高速）。

---

## ファイル構成

```
MeshExpander/
├── include/expander/
│   ├── IExpander.hpp           基底インタフェース
│   ├── Mesh.hpp                頂点+面データ構造
│   ├── MathUtils.hpp           正規化・方向生成・半空間ユーティリティ
│   ├── ConservativeExpander.hpp  凸形状向け膨張器
│   ├── ClippingEngine.hpp      半空間クリッピング（box + planes）
│   ├── VoxelGrid.hpp           ボクセル化 + グリーディボックスマージ
│   ├── RobustSlicer.hpp        凹形状向け膨張器
│   ├── StlReader.hpp           バイナリ STL リーダー（ヘッダーオンリー）
│   └── StlWriter.hpp           バイナリ STL ライター（ヘッダーオンリー）
├── src/
│   ├── ConservativeExpander.cpp
│   ├── ClippingEngine.cpp
│   ├── VoxelGrid.cpp
│   └── RobustSlicer.cpp
├── tests/
│   ├── unit/                   各クラスのユニットテスト
│   └── integration/            精度評価・実世界メッシュテスト
├── docs/
│   └── README_JA.md            このファイル
└── CMakeLists.txt
```

---

## ライセンス

MIT License — [LICENSE](../LICENSE) 参照。
