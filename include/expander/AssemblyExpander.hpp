#pragma once
// ---------------------------------------------------------------------------
// AssemblyExpander — conservative expansion for multi-part 3D assemblies
//
// Accepts a list of Mesh objects (one per part) — typically loaded from a
// multi-body file (STEP/OBJ/FBX) via IModelLoader — and expands each part
// independently.
//
// Default behavior (useVoxelPartitioning = false):
//   Every part is expanded with ConservativeExpander regardless of convexity.
//   The part decomposition is driven solely by the file's mesh structure.
//   This is the recommended mode for machining clearance models where each
//   file mesh corresponds to one manufacturing component.
//
// Optional behavior (useVoxelPartitioning = true):
//   - Convex parts  → ConservativeExpander  (single polytope, low polygon count)
//   - Concave parts → RobustSlicer          (multi-polytope, voxel-partitioned)
//   Use this only when a single mesh part is itself highly concave and needs
//   internal voxel subdivision for tighter fit.
//
// This class has NO dependency on Assimp or any file I/O library.
// Use IModelLoader / IModelExporter (mesh_expander_io) for file loading/saving.
//
// Typical usage:
//   auto parts  = loader->load("assembly.stp");
//   auto merged = AssemblyExpander::mergeContained(parts);
//   AssemblyExpander expander;                       // voxel partitioning OFF
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
        // When false (default), all parts are expanded with ConservativeExpander.
        // Part boundaries come from the file's mesh structure (one mesh = one part).
        // When true, concave parts are further subdivided via RobustSlicer's
        // voxel partitioner for a tighter fit.
        bool   useVoxelPartitioning = false;

        // Resolution for RobustSlicer (used only when useVoxelPartitioning = true).
        // Cell size = aabb.maxDim() / resolution  (adaptive per part).
        int    resolution         = 64;

        // If > 0, use this fixed voxel cell size (world units) instead of
        // adaptive resolution. Used only when useVoxelPartitioning = true.
        double cellSizeWorld      = 0.0;

        // Angle threshold for merging near-parallel face normals.
        double faceNormalMergeDeg = 20.0;

        // Tolerance for isConvex() test (used only when useVoxelPartitioning = true).
        double convexTol          = 1e-6;
    };

    explicit AssemblyExpander(Options opts = {});

    // ---------------------------------------------------------------------------
    // expand()
    // Expand each part independently. Returns one expanded Mesh per input part.
    // Default: all parts via ConservativeExpander (one mesh = one part from file).
    // opt-in (useVoxelPartitioning): concave → RobustSlicer multi-polytope.
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

    // Expand a single part. Algorithm chosen based on useVoxelPartitioning.
    // Returns an empty Mesh for empty input.
    Mesh expandPart(const Mesh& part, double d) const;

    // Merge a flat list of Meshes into one multi-body Mesh.
    static Mesh mergeMeshes(const std::vector<Mesh>& meshes);
};

} // namespace expander
