#pragma once
// ---------------------------------------------------------------------------
// RobustSlicer — Conservative expansion for concave (non-convex) meshes
//
// Algorithm:
//   1. Compute voxel cell size:
//      - Resolution mode: cellSize = aabb.maxDim() / resolution  (adaptive)
//      - Fixed mode (withCellSize): cellSize = supplied world-space value
//   2. VoxelGrid::build()       Conservative triangle-AABB voxelization
//   3. VoxelGrid::greedyMerge() Decompose occupied voxels into non-overlapping
//                               axis-aligned boxes (preserving concavities)
//   4. Per-box local expansion:
//      a. Select mesh faces whose AABB overlaps the box (duplicates allowed)
//      b. Collect & merge face normals at faceNormalMergeDeg threshold
//      c. Build half-spaces: D_i = max(local_vertices . n_i) + d
//      d. ClippingEngine::clip(expandedBox, half-spaces)
//         (kSafetyMargin is added internally by ClippingEngine)
//   5. Return individual polytopes OR merged single mesh
//
// Conservativeness guarantee:
//   Every input vertex v lies inside at least one output polytope.
//   Proof: v is in a voxel -> covered by box B -> inside expandedBox ->
//   v.n_i <= D_i - d for every half-space (D_i = max + d >= v.n_i + d) ->
//   v is strictly inside clip(expandedBox, hs) with margin d.
// ---------------------------------------------------------------------------

#include "IExpander.hpp"
#include "MathUtils.hpp"

#include <Eigen/Geometry>
#include <vector>

namespace expander {

class RobustSlicer : public IExpander {
public:
    // resolution         : number of voxel cells along the longest AABB axis.
    //                      Cell size = aabb.maxDim() / resolution (adaptive).
    // faceNormalMergeDeg : merge threshold for near-parallel face normals.
    explicit RobustSlicer(int    resolution         = 64,
                          double faceNormalMergeDeg = 20.0);

    // Factory: use a fixed voxel cell size in world (mesh) units instead of
    // the adaptive resolution.  Useful when the physical cell size is known:
    //   auto slicer = RobustSlicer::withCellSize(5.0);  // 5 mm cells
    static RobustSlicer withCellSize(double cellSizeWorld,
                                     double faceNormalMergeDeg = 20.0);

    // Expand a possibly concave mesh into a set of closed convex polytopes.
    // The union of the output polytopes conservatively covers input + d.
    std::vector<Mesh> expandMulti(const Mesh& input, double d);

    // Expand and merge all polytopes into a single mesh.
    // The result is a multi-body mesh (each polytope is a separate connected
    // component with correct winding).  Suitable for STL export and visual
    // inspection; divergence-theorem volume computation is NOT valid on it.
    Mesh expandMerged(const Mesh& input, double d);

    // IExpander compatibility: returns the first non-empty polytope, or empty.
    // For concave shapes prefer expandMerged() or expandMulti().
    Mesh expand(const Mesh& input, double d) override;

    // Utility: merge a list of meshes into a single multi-body mesh.
    static Mesh merge(const std::vector<Mesh>& meshes);

private:
    int    resolution_;
    double faceNormalMergeDeg_;
    double cellSizeWorld_;   // > 0 -> use directly;  <= 0 -> use resolution

    // Returns indices of faces whose triangle AABB intersects box.
    static std::vector<int> collectFaces(const Mesh&                mesh,
                                          const Eigen::AlignedBox3d& box);
};

} // namespace expander
