#pragma once
// ---------------------------------------------------------------------------
// VoxelGrid — Conservative voxelization + Greedy Box Merging
//
// Usage:
//   VoxelGrid grid(cellSizeWorld);
//   grid.build(mesh);                    // mark occupied cells
//   auto boxes = grid.greedyMerge();     // get axis-aligned box set
//
// Voxelization is conservative: for each triangle the AABB of the triangle
// is fully marked occupied. This ensures no mesh surface is ever outside the
// union of returned boxes.
//
// Greedy merge scans voxels in (z,y,x) order and extends each seed into the
// largest axis-aligned box whose cells are all occupied and not yet claimed.
// The resulting box set covers every occupied cell with no overlaps.
// ---------------------------------------------------------------------------

#include "Mesh.hpp"

#include <Eigen/Geometry>
#include <vector>

namespace expander {

class VoxelGrid {
public:
    struct Box {
        Eigen::AlignedBox3d worldBounds;  // axis-aligned box in world coordinates
    };

    // cellSizeWorld: side length of one voxel in world (mesh) units, e.g. 5.0 mm
    explicit VoxelGrid(double cellSizeWorld);

    // Discretize the mesh into occupied voxels.
    // Each triangle is conservatively rasterized via its AABB.
    // A 1-cell margin is added around the mesh AABB as the grid domain.
    void build(const Mesh& mesh);

    // Return the greedy box decomposition of the occupied voxel set.
    // Each Box is non-overlapping; together they cover every occupied cell.
    std::vector<Box> greedyMerge() const;

    // World-space AABB of voxel cell (x, y, z).
    Eigen::AlignedBox3d cellBounds(int x, int y, int z) const;

    // Grid dimensions
    Eigen::Vector3i dims() const { return dims_; }

    // True if cell (x,y,z) is within grid bounds and occupied.
    bool occupied(int x, int y, int z) const;

private:
    double          cellSize_;
    Eigen::Vector3d origin_;   // world position of cell (0,0,0) corner
    Eigen::Vector3i dims_;
    std::vector<bool> cells_;  // flat [x + dims.x*(y + dims.y*z)]

    int flatIdx(int x, int y, int z) const;
};

} // namespace expander
