#pragma once
// ---------------------------------------------------------------------------
// BoxExpander — 削り出し法コア機能 (Carving Expansion Core)
//
// アルゴリズム:
//   1. expandedBox = box ± d  (初期ポリトープの境界)
//   2. collectFaces(mesh, box) でボックスと重なる面を収集
//   3. 面法線を faceNormalMergeDeg でマージ
//   4. 半空間生成: D_i = max(local_vertices . n_i) + d
//   5. ClippingEngine::clip(expandedBox, half-spaces) → 閉凸多面体
//
// AssemblyExpander は mesh の AABB 全体をボックスとして使用してこのクラスを呼び出す。
// ---------------------------------------------------------------------------

#include "IExpander.hpp"
#include "MathUtils.hpp"

#include <Eigen/Geometry>
#include <vector>

namespace expander {

class BoxExpander : public IExpander {
public:
    // faceNormalMergeDeg: merge threshold for near-parallel face normals (degrees).
    explicit BoxExpander(double faceNormalMergeDeg = 20.0);

    // Core carving function: expand a box using face normals from mesh.
    //   box         : the region of interest (mesh faces that overlap are used)
    //   mesh        : source mesh providing face normals and vertices
    //   d           : expansion distance
    //   expandedBox = box ± d is used as the initial polytope boundary,
    //   which is then carved by local half-spaces from nearby face normals.
    //   Returns a single closed convex polytope.
    Mesh expand(const Eigen::AlignedBox3d& box,
                const Mesh&                mesh,
                double                     d) const;

    // Convenience: use mesh's own AABB as the box.
    // Equivalent to: expand(mesh.aabb(), mesh, d)
    Mesh expand(const Mesh& mesh, double d) override;

    // Returns indices of faces whose triangle AABB intersects box.
    static std::vector<int> collectFaces(const Mesh&                mesh,
                                          const Eigen::AlignedBox3d& box);

private:
    double faceNormalMergeDeg_;
};

} // namespace expander
