#pragma once
// ---------------------------------------------------------------------------
// VoxelGrid -- Solid voxelization + Top-Down Recursive Partitioning
//
// Algorithm:
//   build():
//     1. Conservative triangle-AABB surface rasterization.
//     2. solidFill(): 6-connectivity BFS flood-fill from all unoccupied border
//        cells marks the exterior.  Any remaining unoccupied cell is interior
//        and is marked occupied (solid).
//
//   greedyMerge() -- top-down recursive partitioner:
//     - Root = full grid.
//     - All-empty   -> discard.
//     - All-occupied -> emit one axis-aligned box.
//     - Mixed       -> split on the axis/position that maximises pure-region
//                      cell count; midpoint fallback when no split creates purity.
//     - Recurse on both halves.
//
// Guarantees:
//   - Every occupied voxel is covered by exactly one output box.
//   - No two output boxes share interior volume.
//   - Solid interior regions produce one large box each; only the surface shell
//     is subdivided down to cell size.
// ---------------------------------------------------------------------------

#include "Mesh.hpp"

#include <Eigen/Geometry>
#include <vector>

namespace expander {

class VoxelGrid {
public:
    struct Box {
        Eigen::AlignedBox3d worldBounds;
    };

    // cellSizeWorld: side length of one voxel in world (mesh) units, e.g. 0.005 m
    explicit VoxelGrid(double cellSizeWorld);

    // Voxelize the mesh: surface rasterization then solid interior fill.
    void build(const Mesh& mesh);

    // Top-down recursive partition of occupied voxels into non-overlapping boxes.
    std::vector<Box> greedyMerge() const;

    // World-space AABB of voxel cell (x, y, z).
    Eigen::AlignedBox3d cellBounds(int x, int y, int z) const;

    // Grid dimensions.
    Eigen::Vector3i dims() const { return dims_; }

    // True if cell (x,y,z) is within grid bounds and occupied.
    bool occupied(int x, int y, int z) const;

private:
    double          cellSize_;
    Eigen::Vector3d origin_;
    Eigen::Vector3i dims_;
    std::vector<bool> cells_;  // flat [x + dims.x*(y + dims.y*z)]

    int flatIdx(int x, int y, int z) const;

    // BFS flood-fill from exterior; marks enclosed interior cells as occupied.
    void solidFill();

    // Count occupied cells in [x0..x1] x [y0..y1] x [z0..z1] (inclusive).
    int countOccupied(int x0, int y0, int z0, int x1, int y1, int z1) const;

    // Find the split on `axis` that maximises pure-region cell count.
    // Returns split index (first of right half); writes score (pure cells).
    // Falls back to midpoint when no split achieves score > 0.
    int findBestSplit(int x0, int y0, int z0, int x1, int y1, int z1,
                      int axis, int& score) const;

    // Recursively partition region and append boxes to out.
    void subdivide(int x0, int y0, int z0, int x1, int y1, int z1,
                   std::vector<Box>& out) const;
};

} // namespace expander
