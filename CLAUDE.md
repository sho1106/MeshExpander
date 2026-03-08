# MeshExpander — Claude Project Instructions

## プロジェクト概要

C++17 による保守的 3D メッシュ膨張ライブラリ。入力メッシュを距離 d だけ確実に包摂する凸多面体を生成する。

## アーキテクチャ

```
ConservativeExpander          ← 凸形状向け（唯一のエントリポイント）
  │
  ├── 面データあり → 面法線モード（形状適応・精度高）
  │     入力面の法線を収集 → 20°でマージ → 半空間交差 → 単一閉多面体
  │
  └── 面データなし → 26方向フォールバック（旧来互換）
        26固定方向 → 半空間交差 → 単一閉多面体

VoxelGrid                     ← ボクセル化 + グリーディマージ（凹形状サポート用）
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

### ボリューム計算の正確性
- 発散定理は単一閉多面体のみ有効。複数 mesh の連結は隣接面がキャンセル
- ConservativeExpander は expand() 1 回につき必ず単一多面体を返す

## ファイル構成

```
include/expander/
  IExpander.hpp            基底インタフェース
  Mesh.hpp                 頂点+面データ構造
  MathUtils.hpp            正規化・方向・半空間ユーティリティ
  ConservativeExpander.hpp 唯一の公開エキスパンダ
  ClippingEngine.hpp       半空間クリッピング (VoxelGrid 用)
  VoxelGrid.hpp            ボクセル化 + グリーディマージ
  StlWriter.hpp            STL ファイル出力

src/
  ConservativeExpander.cpp
  ClippingEngine.cpp
  VoxelGrid.cpp

tests/
  unit/                    33 テスト（ConservativeExpander/MathUtils/ClippingEngine/VoxelGrid）
  integration/             4 テスト（球・円柱・円錐の精度 + NaN 堅牢性）
```

## ビルド・テストコマンド

```bash
# 初回セットアップ
cmake -S projects/MeshExpander -B projects/MeshExpander/build

# ビルド + 全テスト
cmake --build projects/MeshExpander/build --config Release --target check

# 統合テストのみ（ratio 出力あり）
./projects/MeshExpander/build/Release/integration_tests
```

## 現在の精度（face-normal モード、d=1.0mm）

| 形状 | ratio | over% |
|---|---|---|
| 球 (R=10-100) | 1.033 | +3.3% |
| 円柱 | 1.012 | +1.2% |
| 円錐 (H=3R) | 1.013 | +1.3% |

## 次のステップ: 凹形状サポート（RobustSlicer）

### 設計仕様

```
入力: 凹形状メッシュ（Stanford Bunny 等）
  │
  ▼
1. VoxelGrid::build()      保守的三角形-AABB ラスタライズ
     voxel_size = aabb.maxDim() / resolution (resolution=64 推奨)
     triangle_aabb 交差判定で塗り
  │
  ▼
2. VoxelGrid::greedyMerge() 凹形状の空隙を保持した直方体集合へ統合
     優先度: 大きい直方体から。空き格子セルのみ使用
  │
  ▼
3. Per-Box Local Expansion  各直方体に対し:
     a. 直方体内のメッシュ三角形を選択（重複 OK）
     b. 選択三角形の面法線でクリッピング（ClippingEngine::clip）
     c. Safety Margin 強制加算（kSafetyMargin >= 1e-5）
     d. Post-Inclusion Check: 全入力頂点が何らかの直方体に含まれていることを確認
  │
  ▼
4. 出力: std::vector<Mesh>  独立閉多面体のリスト
         体積 = 個別体積の和（相互に重複可）
```

### 実装予定クラス

```cpp
class RobustSlicer : public IExpander {
public:
    explicit RobustSlicer(int resolution = 64, double safetyMargin = 1e-4);
    // IExpander は単一 Mesh を返すが、凹形状では vector<Mesh> が適切
    // → expandMulti() を追加するか IExpander を拡張
    Mesh expand(const Mesh& input, double d) override;  // 互換用: 最初のボックスのみ
    std::vector<Mesh> expandMulti(const Mesh& input, double d);
private:
    int resolution_;
    double safetyMargin_;
};
```

### 注意点
- VoxelGrid はすでに実装済み（greedyMerge まで）
- ClippingEngine も実装済み
- 未実装: Per-Box Local Expansion の三角形選択 + Post-Inclusion Check
- 三角形選択: AABB 拡張後のボックスと三角形の intersection test (SAT)

## テスト拡充方針

- RobustSlicer 向け: 凹み L字形、中空球、Stanford Bunny 等
- 精度指標: sum(individual volumes) / ideal_volume
- 保守性確認: 全入力頂点が union of output boxes に含まれること
