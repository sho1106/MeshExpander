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
    };

    explicit AssemblyExpander(Options opts = {});

    // Expand each part independently via BoxExpander (削り出し法).
    // Returns one expanded Mesh per input part (index-aligned).
    // Empty input parts produce empty output Mesh entries.
    std::vector<Mesh> expand(const std::vector<Mesh>& parts, double d) const;

    // Expand all parts and concatenate into a single multi-body mesh.
    // Suitable for STL/OBJ export and visual inspection.
    Mesh expandMerged(const std::vector<Mesh>& parts, double d) const;

    // Merge parts whose bounding box is fully contained within another part's
    // bounding box (within tolerance). Absorbed parts have their geometry
    // appended to the containing part and are removed from the list.
    // Use case: CAD sub-features (holes, bosses) that should expand with the parent.
    static std::vector<Mesh> mergeContained(const std::vector<Mesh>& parts,
                                            double tolerance = 1e-6);

private:
    Options opts_;

    Mesh expandPart(const Mesh& part, double d) const;
    static Mesh mergeMeshes(const std::vector<Mesh>& meshes);
};

} // namespace expander
