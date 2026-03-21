# MeshExpander — Claude Project Instructions

## プロジェクト概要

C++17 による保守的 3D メッシュ膨張ライブラリ。入力メッシュを距離 d だけ確実に包摂する凸多面体（または凸多面体集合）を生成する。

## アーキテクチャ

```
ConservativeExpander          ← 凸形状向け（唯一のエントリポイント）
  │
  ├── 面データあり → 面法線モード（形状適応・精度高）
  │     入力面の法線を収集 → 20°でマージ → 半空間交差 → 単一閉多面体
  │
  └── 面データなし → 26方向フォールバック（旧来互換）
        26固定方向 → 半空間交差 → 単一閉多面体

RobustSlicer                  ← 凹形状向け（Stanford Bunny 等）
  │
  ├── VoxelGrid::build()       保守的 AABB ラスタライズ（BFS flood-fill）
  ├── VoxelGrid::greedyMerge() 直方体集合へ統合（top-down 再帰パーティショナー）
  └── Per-Box Local Expansion  各直方体に ConservativeExpander を適用

VoxelGrid                     ← ボクセル化 + グリーディマージ
ClippingEngine                ← 半空間クリッピング（box + half-spaces）
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

### ボクセルラスタライザ選択（IVoxelRasterizer）
- AABB が最良: 最速 + 最少ボックス + 100% 保守性 → 採用
- SDF は最悪: 遅い(×10-20)、ボックス多い(×2-20)
- SAT の罠: Gear 形状で Cov%=99.3% — 表面頂点がセル境界に落ちると voxel が漏れる

### ボリューム計算の正確性
- 発散定理は単一閉多面体のみ有効。複数 mesh の連結は隣接面がキャンセル
- ConservativeExpander は expand() 1 回につき必ず単一多面体を返す

## ファイル構成

```
include/expander/
  IExpander.hpp            基底インタフェース
  Mesh.hpp                 頂点+面データ構造
  MathUtils.hpp            正規化・方向・半空間ユーティリティ
  ConservativeExpander.hpp 唯一の公開エキスパンダ（凸形状）
  ClippingEngine.hpp       半空間クリッピング (VoxelGrid 用)
  VoxelGrid.hpp            ボクセル化 + グリーディマージ
  StlWriter.hpp            STL ファイル出力

src/
  ConservativeExpander.cpp
  ClippingEngine.cpp
  VoxelGrid.cpp
  RobustSlicer.cpp

tests/
  unit/                    ユニットテスト（ConservativeExpander/MathUtils/ClippingEngine/VoxelGrid/RobustSlicer）
  integration/             統合テスト（球・円柱・円錐・凹形状・CAD・Stanford Bunny）

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

## 現在の精度（全 57 テスト PASS）

### 凸形状（ConservativeExpander、face-normal モード、d=1.0mm）

| 形状 | ratio | over% |
|---|---|---|
| 球 (R=10-100) | 1.033 | +3.3% |
| 円柱 | 1.012 | +1.2% |
| 円錐 (H=3R) | 1.013 | +1.3% |

### 凹形状（RobustSlicer）

| 形状 | 保守性 | 備考 |
|---|---|---|
| Stanford Bunny (full) | 99.8% | ConservativeExpander 比 -64% 体積 |
| CAD 形状 (Torus/Gear/Star/HollowCylinder) | 100% | Cov=100%, Exp=100% |

## テスト拡充方針

- 精度指標: sum(individual volumes) / ideal_volume
- 保守性確認: 全入力頂点が union of output boxes に含まれること
- 新形状追加時は `tests/integration/` に追加し `CMakeLists.txt` へ登録
