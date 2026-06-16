# MeshExpander — 技術ガイド（日本語版）

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](../LICENSE)

[English](../README.md) | **日本語**

**CADアセンブリから部品ごとの削り出し膨張モデルを生成する C++17 ライブラリ。**
入力メッシュを距離 `d` だけ保守的に膨張させた閉多面体（加工クリアランスモデル）を生成します。膨張量 `d` はアルゴリズムで保証され、頂点・辺・面の取りこぼしはゼロです。

> **単位**: 単位系非依存。座標と `d` は入力モデルの単位系で指定します。各部品はクリッピング前に内部で正規化されるため、スケールに対して頑健です（µm〜km で検証済み）。`d` を形状と同じ単位で与えてください。

---

## 目次

1. [概要](#概要)
2. [使いどころ / 使わないところ](#使いどころ--使わないところ)
3. [ビルド手順](#ビルド手順)
4. [使い方](#使い方)
5. [アルゴリズム詳細](#アルゴリズム詳細)
6. [凹形状対応](#凹形状対応)
7. [テスト](#テスト)
8. [設計原則](#設計原則)
9. [ファイル構成](#ファイル構成)

---

## 概要

CNC・放電加工では、工具経路計算・干渉チェック・治具設計のために**加工クリアランスモデル**（元形状を `d` だけ膨らませた閉多面体）が必要になります。MeshExpander はマルチパート CAD ファイルの各部品メッシュを、面法線ベースの「削り出し法」で膨張させます。

**主な特徴:**
- **保守性保証（Zero Shrinking）**: 膨張後の形状は元形状 + 距離 `d` を必ず包摂
- **形状適応**: 面法線ベースの適応的半空間生成で球 +3.3%、円柱 +1.2%、円錐 +1.3% の過膨張
- **入力密度非依存**: 出力頂点数は `C(k+6, 3)` に上限（k = 面法線方向数）。入力面数に比例しない
- **凹形状対応**: concavity 駆動の空間ボックス分割（`BoxPartitioner`）で凹コーナー・段差を分解
- **数値安全**: 全半空間に `kSafetyMargin = 1e-6` を付加し、浮動小数点誤差を常に外側へ逃がす
- **軽量依存**: コアは Eigen のみ（CMake FetchContent で自動取得）。STEP/OBJ/FBX 入出力は任意の Assimp レイヤ

---

## 使いどころ / 使わないところ

MeshExpander は元形状を内包する**保守的な外殻**を生成します。真の最小オフセット（Minkowski オフセット）ではない点に注意してください。

| ✅ 向いている | ❌ 向いていない |
|---|---|
| 干渉チェック・治具/クランプの包絡体 | ポケット・穴・溝・スロット内側の正確なクリアランス（凹フィーチャは充填される） |
| 素材ブロック（ストック）寸法の保証付き見積り | オーガニック/自由曲面の忠実なオフセット |
| 「絶対に元形状を下回らない」保証が要る用途 | 凸包 = AABB になる貫通溝・キー溝（分解しても改善が小さい） |
| 高密度メッシュから軽量・低ポリの閉多面体が欲しい | 距離を縮める/負オフセットしたい |

### なぜ OpenVDB / CGAL ではなくこれか

| ツール | 強み | このタスクでの弱み |
|---|---|---|
| **MeshExpander** | 解析的な包含保証（縮みゼロ）、低ポリ（入力密度非依存）、Eigen のみ・MIT、pip で入る | 過膨張する／凸寄り、溝・チャンネルに弱い |
| **OpenVDB** | 任意トポロジ・凹形状の真のオフセット、堅牢 | 解像度依存で*保証*なし、出力が高ポリ、重い依存（TBB 等）。膨張は等方半径のみ（方向別クリアランス不可） |
| **CGAL** (Minkowski) | 厳密なオフセット | O(n³m³) で大規模 CAD に遅い、GPL/GMP のビルド・ライセンス負担 |
| **trimesh / Open3D** | I/O・SDF クエリ | メッシュオフセット機能は内蔵されていない |

要するに **「機械加工向けに、保証付き・低ポリ・軽量依存のクリアランス外殻が欲しい」** が MeshExpander の wedge です。真のオフセットや複雑な凹面追従が必要なら OpenVDB を使ってください。

---

## ビルド手順

### 必要環境

| ツール | バージョン |
|---|---|
| CMake | ≥ 3.16 |
| C++ コンパイラ | C++17 対応（MSVC 2019+, GCC 9+, Clang 10+） |
| インターネット接続 | Eigen・GoogleTest の自動取得に必要（初回のみ） |

### ビルド

```bash
git clone https://github.com/sho1106/MeshExpander.git
cd MeshExpander

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release

# テスト実行
cmake --build build --config Release --target check
```

### STEP / OBJ / FBX 入出力を使う場合（任意）

コアライブラリは STL のみを扱います。STEP/OBJ/FBX などの CAD フォーマットは Assimp ベースの IO レイヤを有効化してビルドします。

```bash
cmake -S . -B build -DMESHEXPANDER_BUILD_IO=ON
cmake --build build --config Release
```

### 他の CMake プロジェクトから利用する

`cmake --install build` 後、`find_package` で取り込めます。

```cmake
find_package(MeshExpander REQUIRED)
target_link_libraries(your_target PRIVATE MeshExpander::mesh_expander)
```

---

## 使い方

### 単一 STL メッシュの膨張（追加依存なしで動く最短経路）

```cpp
#include "expander/BoxExpander.hpp"
#include "expander/StlReader.hpp"
#include "expander/StlWriter.hpp"

expander::Mesh input  = expander::StlReader::read("part.stl");  // バイナリ STL
expander::BoxExpander exp;
expander::Mesh result = exp.expand(input, 1.0);               // d = 1mm
expander::StlWriter::write("expanded.stl", result);
```

> `StlReader` は**バイナリ STL のみ**対応です（C++ の `StlReader::read` は ASCII STL に対して空メッシュを返し、Python の `read_stl`/`expand_file` は明確なエラーを送出します）。多くの CAD/メッシュツールは ASCII を既定出力にするため、エクスポート形式に注意してください。

### CADアセンブリから部品ごとの膨張モデル（Assimp IO 必須）

```cpp
#include "expander/AssemblyExpander.hpp"
#include "io/AssimpLoader.hpp"        // -DMESHEXPANDER_BUILD_IO=ON でビルド
#include "expander/StlWriter.hpp"

expander::io::AssimpLoader loader;
std::vector<expander::Mesh> parts = loader.load("assembly.stp");  // 各メッシュ = 1部品

// 内包関係にある部品を統合（穴・ボスなどのサブフィーチャ用）
parts = expander::AssemblyExpander::mergeContained(parts);

expander::AssemblyExpander expander;
std::vector<expander::Mesh> models = expander.expand(parts, 1.0);

expander::Mesh merged = expander.expandMerged(parts, 1.0);
expander::StlWriter::write("assembly_expanded.stl", merged);
```

### 凹形状の膨張（concavity 駆動の空間ボックス分割）

```cpp
// 既定は単一凸（凹みは凸包まで充填される）。凹対応はオプションで有効化する。
expander::AssemblyExpander::Options opts;
opts.maxConvexPieces = 8;     // 1部品あたり最大 8 凸ピースまで分割
opts.concavityTol    = 0.0;   // concavity がこの許容値（モデル単位）以下になるまで分割

expander::AssemblyExpander expander(opts);
std::vector<expander::Mesh> models = expander.expand(parts, 1.0);
```

### 異方性（方向別）クリアランス

`d` を軸別 `[dx, dy, dz]` で指定できます。各軸平行面はその成分ぶんだけ膨張し（オフセットは楕円体支持 `‖n⊙d‖`）、均一な `d` はスカラ（ボール）膨張と完全に一致します。例: 軸方向（Z）に 3mm の抜き代、半径方向（X/Y）に 0.5mm の仕上げ代。

```cpp
#include "expander/BoxExpander.hpp"
expander::BoxExpander exp;
expander::Mesh result = exp.expand(mesh, Eigen::Vector3d(0.5, 0.5, 3.0));
```

```python
import meshexpander as me
out_v, out_f = me.expand_np(verts, faces, d=[0.5, 0.5, 3.0])
result = me.BoxExpander().expand(mesh, [0.5, 0.5, 3.0])
```

---

## アルゴリズム詳細

```
入力メッシュ（1 部品）
  │
  1. 初期ボックス  メッシュの AABB を取得 → expandedBox = AABB ± d
  │
  2. 面法線抽出   全三角形の面法線を収集 → 20° 以内の近似平行法線をマージ → k 方向
  │
  3. 半空間生成   各方向 n に対して D_i = max(V · n) + d
  │               （全頂点の法線方向射影の最大値 + 膨張オフセット）
  │
  4. 削り出し     ClippingEngine::clip(expandedBox, 半空間群)
  │               expandedBox の 6 面 + k 個の面法線半空間を交差
  │               → C(k+6, 3) 個の平面トリプレット交点を列挙し保持
  │
  出力: 単一の閉凸多面体（頂点数 ≤ C(k+6, 3)）
```

### 保守性の保証

入力の任意の頂点 `v` について、各方向 `n_i` に対して `v · n_i ≤ max(V · n_i) = D_i − d < D_i` が成り立ちます。したがって `v` は出力多面体の**すべての半空間の内側**に距離 `d` のマージンをもって収まります。

### 精度（d = 1.0mm、26方向固定法との比較）

| 形状 | 体積比 | 過膨張率 | 旧26方向固定 |
|---|---|---|---|
| 球 (R=10–100) | 1.033 | +3.3% | ~1.14（apex dominance で悪化） |
| 円柱 | 1.012 | +1.2% | ~1.055 |
| 円錐 (H=3R) | 1.013 | +1.3% | ~2.0–2.2（(1,1,1) が apex 直撃） |

面法線モードは形状固有の方向を使うため、固定 26 方向で生じる円錐 apex の過膨張（ratio 2.0 超）を回避できます。

### ClippingEngine

半空間クリッピングエンジン。`expandedBox` を局所的な平面群で切り取ります。

- `kSafetyMargin = 1e-6`: 全半空間オフセットに加算（外側方向に）
- `kOnPlaneEps = 1e-5`: 平面上判定の閾値
- 並列法線の重複排除: `dot > 1 − 1e-8` なら tighter（小さい D）を保持

---

## 凹形状対応

`maxConvexPieces` / `concavityTol` を指定すると、`BoxPartitioner` が部品を **concavity 駆動で軸平行ボックスに分割**し、各ボックスを削り出して和をとります。ボクセルグリッドは使いません（少数ボックスの適応的二分割）。

| 形状 | ピース数 K | 体積比 | 出力面数 |
|---|---|---|---|
| L 字プリズム | 1（=単一凸） | 1.775 | 12 |
| L 字プリズム | 4 | 1.591 | 48 |
| L 字プリズム | 8 | 1.418 | 96 |

K を増やすほど凹みの埋め（無駄な体積）が減り、出力面数は K に概ね比例します。全 K で Cov% = 100%。

> **限界**: 凹コーナー・段差形状で有効。チャンネル（溝）のように凸包が AABB に一致する形状は、軸平行ボックスの膨張和では削り出せず改善が限定的（単一凸と同等になります）。
>
> **注意**: 分解後の結果（`expandMerged` / 分解された部品）は複数多面体の連結で単一閉多面体ではないため、発散定理による体積計算は近似値になります。

---

## テスト

```bash
cmake --build build --config Release --target check
```

| スイート | 区分 | 検証内容 |
|---|---|---|
| BoxExpander | unit | 保守性・堅牢性・頂点数上限 |
| MathUtils | unit | 正規化・方向生成・マージ |
| ClippingEngine | unit | 半空間クリッピング正確性 |
| BoxPartitioner | unit | concavity 計算・凸/凹分割・空ボックス除外 |
| AssemblyExpander | unit | マルチパート展開・mergeContained |
| ShapeExpansion | integration | 球・円柱・円錐の精度比 |
| CadShape | integration | トーラス・ギア・星型・中空シリンダー |
| ComplexShape | integration | BumpySphere・GroovedCylinder の VolRatio |
| ConcaveExpansion | integration | L字・C字の Cov%=100% + 分解による体積削減・ノブ単調性 |
| AssemblyExpansion | integration | マルチパート Cov%=100%・部品別 vs 統合体積 |
| ComplexAssembly | integration | 5 パーツ設備アセンブリ |
| AssimpIO | io | AssimpLoader / AssimpExporter ラウンドトリップ（IO 有効時のみ） |

---

## 設計原則

1. **ゼロ縮小** — 膨張後の形状は入力 + 距離 `d` を必ず包含する。浮動小数点誤差はすべて外側に押し出す。
2. **部品境界はファイルから** — CADファイルのメッシュ構造（ソリッドボディ単位）が部品境界を決める。内部再分割は行わない。
3. **形状適応** — 面法線ベースのため固定方向に依存しない。形状固有の方向で最小の過膨張を実現。
4. **入力密度非依存** — 出力頂点数は `C(k+6, 3)` に上限。
5. **数値安全性** — `kSafetyMargin = 1e-6` を全半空間オフセットに加算。縮退面は黙って読み飛ばす。

---

## ファイル構成

```
MeshExpander/
├── include/expander/
│   ├── BoxExpander.hpp           コアアルゴリズム（削り出し法）
│   ├── AssemblyExpander.hpp      マルチパートオーケストレータ
│   ├── BoxPartitioner.hpp        凹対応: concavity 駆動の空間ボックス分割
│   ├── ClippingEngine.hpp        半空間クリッピング（BoxExpander 内部）
│   ├── Mesh.hpp                  頂点 + 面データ構造
│   ├── MathUtils.hpp             正規化・方向生成・半空間ユーティリティ
│   ├── StlReader.hpp             バイナリ STL リーダー（ヘッダーオンリー）
│   └── StlWriter.hpp             バイナリ STL ライター（ヘッダーオンリー）
├── src/
│   ├── BoxExpander.cpp
│   ├── ClippingEngine.cpp
│   ├── BoxPartitioner.cpp
│   └── AssemblyExpander.cpp
├── python/                       pybind11 バインディング + 型スタブ
├── tests/                        unit / integration / io
├── docs/
│   └── README_JA.md              このファイル
└── CMakeLists.txt
```

---

## コントリビュート

Issue / PR を歓迎します。[CONTRIBUTING.md](../CONTRIBUTING.md) を参照してください。脆弱性報告は [SECURITY.md](../SECURITY.md) へ。質問は GitHub Issue でどうぞ。

## ライセンス

MIT License — [LICENSE](../LICENSE) 参照。
