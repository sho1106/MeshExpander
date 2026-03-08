#include "expander/VoxelGrid.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
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
// build()  --  surface rasterisation + solid interior fill
// ---------------------------------------------------------------------------
void VoxelGrid::build(const Mesh& mesh) {
    if (mesh.vertices.rows() == 0) return;

    const Eigen::Vector3d vmin = mesh.vertices.colwise().minCoeff();
    const Eigen::Vector3d vmax = mesh.vertices.colwise().maxCoeff();

    // Extend domain by one cell on every side (conservative border)
    const Eigen::Vector3d border = Eigen::Vector3d::Constant(cellSize_);
    origin_ = vmin - border;
    const Eigen::Vector3d domainSize = (vmax + border) - origin_;

    for (int ax = 0; ax < 3; ++ax)
        dims_[ax] = static_cast<int>(std::ceil(domainSize[ax] / cellSize_)) + 1;

    cells_.assign(dims_.x() * dims_.y() * dims_.z(), false);

    // 1. Conservative surface rasterisation via triangle AABB
    for (const auto& f : mesh.faces) {
        const Eigen::Vector3d p0 = mesh.vertices.row(f[0]);
        const Eigen::Vector3d p1 = mesh.vertices.row(f[1]);
        const Eigen::Vector3d p2 = mesh.vertices.row(f[2]);

        const Eigen::Vector3d tmin = p0.cwiseMin(p1).cwiseMin(p2);
        const Eigen::Vector3d tmax = p0.cwiseMax(p1).cwiseMax(p2);

        Eigen::Vector3i imin, imax;
        for (int ax = 0; ax < 3; ++ax) {
            imin[ax] = std::max(0,
                static_cast<int>(std::floor((tmin[ax] - origin_[ax]) / cellSize_)));
            imax[ax] = std::min(dims_[ax] - 1,
                static_cast<int>(std::floor((tmax[ax] - origin_[ax]) / cellSize_)));
        }

        for (int z = imin.z(); z <= imax.z(); ++z)
            for (int y = imin.y(); y <= imax.y(); ++y)
                for (int x = imin.x(); x <= imax.x(); ++x)
                    cells_[flatIdx(x, y, z)] = true;
    }

    // 2. Solid fill: flood-fill exterior, mark remaining interior cells occupied
    if (!mesh.faces.empty())
        solidFill();
}

// ---------------------------------------------------------------------------
// solidFill()  --  BFS flood-fill from exterior border cells
// ---------------------------------------------------------------------------
void VoxelGrid::solidFill() {
    const int N = static_cast<int>(cells_.size());
    std::vector<bool> outside(N, false);
    std::queue<int>   q;

    static const int dx[] = {1,-1, 0, 0, 0, 0};
    static const int dy[] = {0, 0, 1,-1, 0, 0};
    static const int dz[] = {0, 0, 0, 0, 1,-1};

    auto tryEnqueue = [&](int x, int y, int z) {
        if (x < 0 || y < 0 || z < 0 ||
            x >= dims_.x() || y >= dims_.y() || z >= dims_.z()) return;
        const int idx = flatIdx(x, y, z);
        if (!cells_[idx] && !outside[idx]) {
            outside[idx] = true;
            q.push(idx);
        }
    };

    // Seed all unoccupied cells on the 6 boundary faces
    for (int z = 0; z < dims_.z(); ++z)
        for (int y = 0; y < dims_.y(); ++y) {
            tryEnqueue(0,           y, z);
            tryEnqueue(dims_.x()-1, y, z);
        }
    for (int z = 0; z < dims_.z(); ++z)
        for (int x = 0; x < dims_.x(); ++x) {
            tryEnqueue(x, 0,           z);
            tryEnqueue(x, dims_.y()-1, z);
        }
    for (int y = 0; y < dims_.y(); ++y)
        for (int x = 0; x < dims_.x(); ++x) {
            tryEnqueue(x, y, 0          );
            tryEnqueue(x, y, dims_.z()-1);
        }

    // BFS flood-fill through unoccupied cells (6-connectivity)
    while (!q.empty()) {
        const int idx = q.front(); q.pop();
        const int z =  idx / (dims_.x() * dims_.y());
        const int y = (idx /  dims_.x()) % dims_.y();
        const int x =  idx %  dims_.x();
        for (int d = 0; d < 6; ++d)
            tryEnqueue(x + dx[d], y + dy[d], z + dz[d]);
    }

    // Any cell not reached from outside and not surface -> solid interior
    for (int i = 0; i < N; ++i)
        if (!outside[i] && !cells_[i])
            cells_[i] = true;
}

// ---------------------------------------------------------------------------
// greedyMerge()  --  public entry point (top-down partitioner)
// ---------------------------------------------------------------------------
std::vector<VoxelGrid::Box> VoxelGrid::greedyMerge() const {
    if (dims_.prod() == 0) return {};
    std::vector<Box> boxes;
    subdivide(0, 0, 0, dims_.x()-1, dims_.y()-1, dims_.z()-1, boxes);
    return boxes;
}

// ---------------------------------------------------------------------------
// countOccupied()
// ---------------------------------------------------------------------------
int VoxelGrid::countOccupied(int x0, int y0, int z0,
                               int x1, int y1, int z1) const {
    int n = 0;
    for (int z = z0; z <= z1; ++z)
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x)
                n += cells_[flatIdx(x, y, z)] ? 1 : 0;
    return n;
}

// ---------------------------------------------------------------------------
// findBestSplit()
// For each candidate split on `axis`, compute purity score = cells ending up
// in an all-occupied or all-empty sub-region.  Returns the split index (first
// index of right half) with the highest score.  Falls back to midpoint when
// no split achieves score > 0.
// ---------------------------------------------------------------------------
int VoxelGrid::findBestSplit(int x0, int y0, int z0,
                              int x1, int y1, int z1,
                              int axis, int& outScore) const {
    const int lo = (axis == 0) ? x0 : (axis == 1) ? y0 : z0;
    const int hi = (axis == 0) ? x1 : (axis == 1) ? y1 : z1;
    const int n  = hi - lo + 1;

    if (n <= 1) { outScore = -1; return lo; }

    const int slabSize =
        (axis == 0) ? (y1-y0+1)*(z1-z0+1) :
        (axis == 1) ? (x1-x0+1)*(z1-z0+1) :
                      (x1-x0+1)*(y1-y0+1);

    // Prefix-sum of occupied cells per slab
    std::vector<int> prefix(n + 1, 0);
    for (int i = 0; i < n; ++i) {
        int count = 0;
        if (axis == 0) {
            for (int z = z0; z <= z1; ++z)
                for (int y = y0; y <= y1; ++y)
                    count += cells_[flatIdx(lo+i, y, z)] ? 1 : 0;
        } else if (axis == 1) {
            for (int z = z0; z <= z1; ++z)
                for (int x = x0; x <= x1; ++x)
                    count += cells_[flatIdx(x, lo+i, z)] ? 1 : 0;
        } else {
            for (int y = y0; y <= y1; ++y)
                for (int x = x0; x <= x1; ++x)
                    count += cells_[flatIdx(x, y, lo+i)] ? 1 : 0;
        }
        prefix[i+1] = prefix[i] + count;
    }
    const int totalOcc = prefix[n];

    // Midpoint fallback; used when no split achieves score > 0
    int bestPos   = lo + n / 2;
    int bestScore = 0;

    for (int s = 1; s < n; ++s) {
        const int leftOcc    = prefix[s];
        const int leftTotal  = s * slabSize;
        const int rightOcc   = totalOcc - leftOcc;
        const int rightTotal = (n - s) * slabSize;

        const int pureLeft  = (leftOcc  == 0 || leftOcc  == leftTotal ) ? leftTotal  : 0;
        const int pureRight = (rightOcc == 0 || rightOcc == rightTotal) ? rightTotal : 0;
        const int score     = pureLeft + pureRight;

        if (score > bestScore) {
            bestScore = score;
            bestPos   = lo + s;
        }
    }

    outScore = bestScore;
    return bestPos;
}

// ---------------------------------------------------------------------------
// subdivide()  --  recursive partitioner
// ---------------------------------------------------------------------------
void VoxelGrid::subdivide(int x0, int y0, int z0,
                           int x1, int y1, int z1,
                           std::vector<Box>& out) const {
    const int total = (x1-x0+1) * (y1-y0+1) * (z1-z0+1);
    const int occ   = countOccupied(x0, y0, z0, x1, y1, z1);

    if (occ == 0)     return;   // all empty -- discard

    if (occ == total) {         // all occupied -- emit one large box
        out.push_back({cellBounds(x0,y0,z0).merged(cellBounds(x1,y1,z1))});
        return;
    }

    if (total == 1) {           // single cell -- guard (shouldn't be mixed)
        out.push_back({cellBounds(x0,y0,z0)});
        return;
    }

    // Find best split axis/position
    int bestAxis = -1, bestPos = -1, bestScore = -1;
    for (int ax = 0; ax < 3; ++ax) {
        int score = 0;
        const int pos = findBestSplit(x0, y0, z0, x1, y1, z1, ax, score);
        if (score < 0) continue;  // axis has size 1, can't split
        if (score > bestScore) {
            bestScore = score;
            bestAxis  = ax;
            bestPos   = pos;
        }
    }

    if (bestAxis < 0) {         // all axes have size 1 -- unreachable for bool grid
        out.push_back({cellBounds(x0,y0,z0).merged(cellBounds(x1,y1,z1))});
        return;
    }

    // Split: left = [lo..bestPos-1], right = [bestPos..hi]
    int lx0=x0,ly0=y0,lz0=z0, lx1=x1,ly1=y1,lz1=z1;
    int rx0=x0,ry0=y0,rz0=z0, rx1=x1,ry1=y1,rz1=z1;
    if      (bestAxis == 0) { lx1 = bestPos-1; rx0 = bestPos; }
    else if (bestAxis == 1) { ly1 = bestPos-1; ry0 = bestPos; }
    else                    { lz1 = bestPos-1; rz0 = bestPos; }

    subdivide(lx0,ly0,lz0, lx1,ly1,lz1, out);
    subdivide(rx0,ry0,rz0, rx1,ry1,rz1, out);
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
    if (x < 0 || y < 0 || z < 0 ||
        x >= dims_.x() || y >= dims_.y() || z >= dims_.z())
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
