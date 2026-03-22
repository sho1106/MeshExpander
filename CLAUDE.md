# MeshExpander — Claude Project Instructions

## プロジェクト概要

C++17 による保守的 3D メッシュ膨張ライブラリ。主用途は削り出し加工モデルの膨張モデル生成。
マルチパートファイル（STEP/OBJ/FBX）から読み込んだ部品ごとに膨張モデルを生成する。

## アーキテクチャ

```
BoxExpander                   ← 削り出し法コア機能（最重要）
  │
  ├── expand(box, mesh, d):   コアAPI
  │     expandedBox = box ± d
  │     collectFaces(mesh, box) → 面法線収集 → 20°マージ
  │     D_i = max(local_vertices · n_i) + d
  │     ClippingEngine::clip(expandedBox, hs) → 単一閉凸多面体
  │
  └── expand(mesh, d):        コンビニエンスAPI（mesh の AABB をボックスとして使用）

AssemblyExpander              ← マルチパートオーケストレータ
      ファイルのメッシュ構造が部品境界 → 全部品を BoxExpander で膨張
      Options: { faceNormalMergeDeg = 20.0 }

ClippingEngine                ← 半空間クリッピング（BoxExpander 内部）
MathUtils                     ← 正規化・方向生成・half-space ユーティリティ
```

## 重要な設計決定

### なぜ面法線モードか
- 26固定方向の問題: 円錐 apex に (1,1,1)/√3 が直撃 → ratio=2.0-2.2 (apex dominance)
- 面法線は形状固有 → 円錐 ratio=1.013, 球 ratio=1.033, 円柱 ratio=1.012

### 並列半空間の重複排除（ClippingEngine）
- kSafetyMargin=1e-6 < kOnPlaneEps=1e-5
- 円柱 cap (0,0,1) と box face (0,0,1) が並列 → assembleMesh が cap を 2 重三角形分割
- clip() 内で dot>1-1e-8 なら tighter (小さい D) を残す

### ボリューム計算の正確性
- 発散定理は単一閉多面体のみ有効。複数 mesh の連結は隣接面がキャンセル
- BoxExpander は expand() 1 回につき必ず単一多面体を返す

## ファイル構成

```
include/expander/
  IExpander.hpp            基底インタフェース
  Mesh.hpp                 頂点+面データ構造
  MathUtils.hpp            正規化・方向・半空間ユーティリティ
  BoxExpander.hpp          削り出し法コアAPI（box + mesh → 閉凸多面体）
  AssemblyExpander.hpp     マルチパートオーケストレータ
  ClippingEngine.hpp       半空間クリッピング（BoxExpander 内部）
  StlReader.hpp            STL ファイル読み込み
  StlWriter.hpp            STL ファイル出力

src/
  BoxExpander.cpp
  ClippingEngine.cpp
  AssemblyExpander.cpp

tests/
  unit/                    ユニットテスト（BoxExpander/MathUtils/ClippingEngine/AssemblyExpander）
  integration/             統合テスト（CAD 形状・アセンブリ）

python/
  meshexpander_core.cpp    pybind11 バインディング
  meshexpander/
    __init__.py            パブリック API
    meshexpander.pyi       型スタブ（IDE 補完・mypy 用）
```

## ビルド・テストコマンド

```bash
# 初回セットアップ（ワークスペースルートから）
cmake -S projects/MeshExpander -B projects/MeshExpander/build -DCMAKE_BUILD_TYPE=Release

# ビルド + 全テスト
cmake --build projects/MeshExpander/build --config Release --target check --parallel 8

# ユニットテストのみ
projects/MeshExpander/build/Release/unit_tests.exe

# 統合テスト（ratio 出力あり）
projects/MeshExpander/build/Release/integration_tests.exe

# Python バインディングビルド（miniforge）
cmake -S projects/MeshExpander -B projects/MeshExpander/build_py -DCMAKE_BUILD_TYPE=Release -DMESHEXPANDER_BUILD_PYTHON=ON -DPython3_EXECUTABLE="C:/ProgramData/miniforge3/python.exe"
cmake --build projects/MeshExpander/build_py --config Release --target meshexpander_core --parallel 8
```

## 現在の精度（BoxExpander、face-normal モード、d=1.0mm）

| 形状 | Cov% | Exp% |
|---|---|---|
| Torus | 100% | 100% |
| Gear 12-tooth | 100% | 100% |
| Star prism 5pt | 100% | 100% |
| Hollow cylinder | 100% | 100% |

## テスト拡充方針

- 精度指標: Cov% = 全入力頂点が出力多面体に包含される割合
- 膨張保証: Exp% = 面法線方向 d 先プローブが出力に包含される割合
- 新形状追加時は `tests/integration/` に追加し `CMakeLists.txt` へ登録
