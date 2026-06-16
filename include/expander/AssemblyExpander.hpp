#pragma once
// ---------------------------------------------------------------------------
// AssemblyExpander — conservative expansion for multi-part 3D assemblies
//
// Accepts a list of Mesh objects (one per part) — typically loaded from a
// multi-body file (STEP/OBJ/FBX) via IModelLoader — and expands each part
// independently using BoxExpander (削り出し法):
//   AABB を初期ボックス、面法線で削り出し → 単一閉凸多面体。
//   部品境界はファイルのメッシュ構造から決まる（1メッシュ = 1部品）。
//
// This class has NO dependency on Assimp or any file I/O library.
// Use IModelLoader / IModelExporter (mesh_expander_io) for file loading/saving.
//
// Typical usage:
//   auto parts  = loader->load("assembly.stp");
//   auto merged = AssemblyExpander::mergeContained(parts);
//   AssemblyExpander expander;
//   Mesh result = expander.expandMerged(merged, 0.002);
//   exporter->write("result.obj", {result});
// ---------------------------------------------------------------------------

#include "expander/Mesh.hpp"
#include "expander/MathUtils.hpp"
#include <vector>

namespace expander {

class AssemblyExpander {
public:
    struct Options {
        // Angle threshold for merging near-parallel face normals.
        double faceNormalMergeDeg = 20.0;

        // 凹形状対応（concavity 駆動の空間ボックス分割）のノブ。
        // 分割は concavityTol > 0 または maxConvexPieces > 1 で有効になる。
        // どちらもデフォルト（0 / 1）なら単一凸 = 従来挙動。
        //
        // concavityTol: 早期停止の閾値（ワールド単位）。最も concavity の大きい領域を
        //   二分割していき、全領域の concavity がこの値以下になった時点で止める。
        //   maxConvexPieces = 1 のまま concavityTol > 0 を指定した場合は、内部の
        //   既定上限（64 ピース）まで分割する。
        double concavityTol    = 0.0;
        // maxConvexPieces: 1 部品あたりのボックス（凸ピース）数の上限。
        //   > 1 で分割を有効化し、最大このピース数まで（または concavityTol 到達まで）分割する。
        int    maxConvexPieces = 1;
    };

    explicit AssemblyExpander(Options opts = {});

    // Expand each part independently via BoxExpander (削り出し法).
    // Returns one expanded Mesh per input part (index-aligned).
    // Empty input parts produce empty output Mesh entries.
    std::vector<Mesh> expand(const std::vector<Mesh>& parts, double d) const;

    // Expand all parts and concatenate into a single multi-body mesh.
    // Suitable for STL/OBJ export and visual inspection.
    // NOTE: the result is a concatenation of independent polytopes, NOT a single
    //       closed manifold. Do not compute volume on it via the divergence
    //       theorem (adjacent/overlapping faces do not cancel correctly); compute
    //       per-part volumes from expand() instead.
    Mesh expandMerged(const std::vector<Mesh>& parts, double d) const;

    // Merge parts that are contained within another part into that parent.
    // Absorbed parts have their geometry appended to the containing part and are
    // removed from the list.
    // Use case: CAD sub-features (holes, bosses) that should expand with the parent.
    // Containment requires BOTH the child's AABB to be inside the parent's AABB
    // (within tolerance) AND the child's centroid to lie inside the parent mesh
    // (point-in-mesh ray test). The centroid check prevents wrongly absorbing a
    // part that sits in a parent's hollow/cavity (e.g. inside an annulus or
    // C-shape) where the AABBs nest but the solids do not. Assumes closed meshes.
    static std::vector<Mesh> mergeContained(const std::vector<Mesh>& parts,
                                            double tolerance = 1e-6);

private:
    Options opts_;

    Mesh expandPart(const Mesh& part, double d) const;
    static Mesh mergeMeshes(const std::vector<Mesh>& meshes);

    // Append src's geometry to dst in place, offsetting face indices.
    static void appendMesh(Mesh& dst, const Mesh& src);
};

} // namespace expander
