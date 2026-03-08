#include "expander/VoxelGrid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace expander {

// ---------------------------------------------------------------------------
// Ctor
// ---------------------------------------------------------------------------
VoxelGrid::VoxelGrid(double cellSizeWorld)
    : cellSize_(cellSizeWorld), origin_(Eigen::Vector3d::Zero()),
      dims_(Eigen::Vector3i::Zero())
{
    if (cellSizeWorld <= 0.0)
        throw std::invalid_argument("VoxelGrid: cellSizeWorld must be > 0");
}

// ---------------------------------------------------------------------------
// build()
// ---------------------------------------------------------------------------
void VoxelGrid::build(const Mesh& mesh) {
    if (mesh.vertices.rows() == 0) return;

    // Mesh AABB
    const Eigen::Vector3d vmin = mesh.vertices.colwise().minCoeff();
    const Eigen::Vector3d vmax = mesh.vertices.colwise().maxCoeff();

    // Extend domain by one cell on every side (conservative border)
    const Eigen::Vector3d border = Eigen::Vector3d::Constant(cellSize_);
    origin_ = vmin - border;
    const Eigen::Vector3d domainSize = (vmax + border) - origin_;

    // Compute grid dimensions (ceiling so every occupied cell is within bounds)
    for (int ax = 0; ax < 3; ++ax)
        dims_[ax] = static_cast<int>(std::ceil(domainSize[ax] / cellSize_)) + 1;

    cells_.assign(dims_.x() * dims_.y() * dims_.z(), false);

    // Rasterize each triangle conservatively via its AABB
    for (const auto& f : mesh.faces) {
        const Eigen::Vector3d p0 = mesh.vertices.row(f[0]);
        const Eigen::Vector3d p1 = mesh.vertices.row(f[1]);
        const Eigen::Vector3d p2 = mesh.vertices.row(f[2]);

        const Eigen::Vector3d tmin = p0.cwiseMin(p1).cwiseMin(p2);
        const Eigen::Vector3d tmax = p0.cwiseMax(p1).cwiseMax(p2);

        // Voxel index range that overlaps with triangle AABB
        Eigen::Vector3i imin, imax;
        for (int ax = 0; ax < 3; ++ax) {
            imin[ax] = static_cast<int>(std::floor((tmin[ax] - origin_[ax]) / cellSize_));
            imax[ax] = static_cast<int>(std::floor((tmax[ax] - origin_[ax]) / cellSize_));
            imin[ax] = std::max(imin[ax], 0);
            imax[ax] = std::min(imax[ax], dims_[ax] - 1);
        }

        for (int z = imin.z(); z <= imax.z(); ++z)
            for (int y = imin.y(); y <= imax.y(); ++y)
                for (int x = imin.x(); x <= imax.x(); ++x)
                    cells_[flatIdx(x, y, z)] = true;
    }
}

// ---------------------------------------------------------------------------
// greedyMerge()
// Scan voxels in (z,y,x) order.  For each unmerged occupied voxel, extend a
// box maximally in x, then y, then z.  Mark all contained cells as merged.
// ---------------------------------------------------------------------------
std::vector<VoxelGrid::Box> VoxelGrid::greedyMerge() const {
    std::vector<bool> merged(cells_.size(), false);
    std::vector<Box>  boxes;

    for (int z = 0; z < dims_.z(); ++z) {
        for (int y = 0; y < dims_.y(); ++y) {
            for (int x = 0; x < dims_.x(); ++x) {
                if (!occupied(x, y, z) || merged[flatIdx(x, y, z)]) continue;

                // Extend in X
                int x2 = x;
                while (x2 + 1 < dims_.x()
                       && occupied(x2 + 1, y, z)
                       && !merged[flatIdx(x2 + 1, y, z)])
                    ++x2;

                // Extend in Y (all x in [x..x2] must be free)
                int y2 = y;
                while (y2 + 1 < dims_.y()) {
                    bool ok = true;
                    for (int xi = x; xi <= x2 && ok; ++xi)
                        ok = occupied(xi, y2 + 1, z) && !merged[flatIdx(xi, y2 + 1, z)];
                    if (!ok) break;
                    ++y2;
                }

                // Extend in Z (all x,y in [x..x2] × [y..y2] must be free)
                int z2 = z;
                while (z2 + 1 < dims_.z()) {
                    bool ok = true;
                    for (int yi = y; yi <= y2 && ok; ++yi)
                        for (int xi = x; xi <= x2 && ok; ++xi)
                            ok = occupied(xi, yi, z2 + 1) && !merged[flatIdx(xi, yi, z2 + 1)];
                    if (!ok) break;
                    ++z2;
                }

                // Mark all cells in this box as merged
                for (int zi = z; zi <= z2; ++zi)
                    for (int yi = y; yi <= y2; ++yi)
                        for (int xi = x; xi <= x2; ++xi)
                            merged[flatIdx(xi, yi, zi)] = true;

                // Compute world-space AABB: from corner of (x,y,z) to far corner of (x2,y2,z2)
                boxes.push_back({cellBounds(x, y, z).merged(cellBounds(x2, y2, z2))});
            }
        }
    }
    return boxes;
}

// ---------------------------------------------------------------------------
// cellBounds()
// ---------------------------------------------------------------------------
Eigen::AlignedBox3d VoxelGrid::cellBounds(int x, int y, int z) const {
    const Eigen::Vector3d lo = origin_ + Eigen::Vector3d(x * cellSize_,
                                                          y * cellSize_,
                                                          z * cellSize_);
    return Eigen::AlignedBox3d(lo, lo + Eigen::Vector3d::Constant(cellSize_));
}

// ---------------------------------------------------------------------------
// occupied()
// ---------------------------------------------------------------------------
bool VoxelGrid::occupied(int x, int y, int z) const {
    if (x < 0 || y < 0 || z < 0
        || x >= dims_.x() || y >= dims_.y() || z >= dims_.z())
        return false;
    return cells_[flatIdx(x, y, z)];
}

// ---------------------------------------------------------------------------
// flatIdx()
// ---------------------------------------------------------------------------
int VoxelGrid::flatIdx(int x, int y, int z) const {
    return x + dims_.x() * (y + dims_.y() * z);
}

} // namespace expander
