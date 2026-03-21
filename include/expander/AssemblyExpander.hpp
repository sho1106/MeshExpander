#pragma once
// ---------------------------------------------------------------------------
// AssemblyExpander — conservative expansion for multi-part 3D assemblies
//
// Accepts a list of Mesh objects (one per part) and expands each independently
// using the best available algorithm:
//   - Convex parts  → ConservativeExpander  (single polytope, low polygon count)
//   - Concave parts → RobustSlicer          (multi-polytope, concavity-aware)
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
        // Resolution for RobustSlicer (concave parts).
        // Cell size = aabb.maxDim() / resolution  (adaptive per part).
        int    resolution         = 64;

        // If > 0, use this fixed voxel cell size (world units) instead of
        // adaptive resolution. Applies uniformly to all concave parts.
        double cellSizeWorld      = 0.0;

        // Angle threshold for merging near-parallel face normals.
        double faceNormalMergeDeg = 20.0;

        // Tolerance for isConvex() test: max allowed outward deviation (world units).
        double convexTol          = 1e-6;
    };

    explicit AssemblyExpander(Options opts = {});

    // ---------------------------------------------------------------------------
    // expand()
    // Expand each part independently. Returns one expanded Mesh per input part.
    // Convex parts → single polytope (ConservativeExpander).
    // Concave parts → merged multi-body mesh (RobustSlicer::expandMerged).
    // Empty input parts produce empty output Mesh entries (index-aligned).
    // ---------------------------------------------------------------------------
    std::vector<Mesh> expand(const std::vector<Mesh>& parts, double d) const;

    // ---------------------------------------------------------------------------
    // expandMerged()
    // Expand all parts and concatenate into a single multi-body mesh.
    // Suitable for STL/OBJ export and visual inspection.
    // ---------------------------------------------------------------------------
    Mesh expandMerged(const std::vector<Mesh>& parts, double d) const;

    // ---------------------------------------------------------------------------
    // mergeContained()  [static]
    // Merge parts whose bounding box is fully contained within another part's
    // bounding box (within tolerance). Absorbed parts have their geometry
    // appended to the containing part and are removed from the list.
    //
    // Use case: CAD sub-features (holes, bosses) that are structurally part of
    // a larger body and should be expanded together with it.
    //
    // Algorithm: O(n²), iterates until stable. Parts are processed smallest-first
    // so inner nesting is resolved transitively.
    // ---------------------------------------------------------------------------
    static std::vector<Mesh> mergeContained(const std::vector<Mesh>& parts,
                                            double tolerance = 1e-6);

    // ---------------------------------------------------------------------------
    // isConvex()  [static]
    // Returns true if mesh is approximately convex.
    // Test: for every face plane (outward normal n, offset d = n·v0),
    //       all vertices must satisfy  n·v <= d + tol.
    // O(faces × vertices). Suitable for typical CAD part counts.
    // ---------------------------------------------------------------------------
    static bool isConvex(const Mesh& mesh, double tol = 1e-6);

private:
    Options opts_;

    // Expand a single part using ConservativeExpander (convex) or
    // RobustSlicer (concave). Returns an empty Mesh for empty input.
    Mesh expandPart(const Mesh& part, double d) const;

    // Merge a flat list of Meshes into one multi-body Mesh.
    static Mesh mergeMeshes(const std::vector<Mesh>& meshes);
};

} // namespace expander
