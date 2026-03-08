#include "expander/RobustSlicer.hpp"
#include "expander/ClippingEngine.hpp"
#include "expander/VoxelGrid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace expander {

// ---------------------------------------------------------------------------
// Ctor
// ---------------------------------------------------------------------------
RobustSlicer::RobustSlicer(int resolution, double faceNormalMergeDeg)
    : resolution_(std::max(resolution, 1))
    , faceNormalMergeDeg_(faceNormalMergeDeg)
    , cellSizeWorld_(0.0)
{}

// ---------------------------------------------------------------------------
// Factory: fixed world-space cell size
// ---------------------------------------------------------------------------
RobustSlicer RobustSlicer::withCellSize(double cellSizeWorld,
                                         double faceNormalMergeDeg)
{
    RobustSlicer r(1, faceNormalMergeDeg);
    r.cellSizeWorld_ = cellSizeWorld;
    return r;
}

// ---------------------------------------------------------------------------
// collectFaces()
// Return indices of faces whose triangle AABB intersects the query box.
// ---------------------------------------------------------------------------
std::vector<int> RobustSlicer::collectFaces(const Mesh&                mesh,
                                              const Eigen::AlignedBox3d& box)
{
    std::vector<int> result;
    result.reserve(64);
    for (int i = 0; i < static_cast<int>(mesh.faces.size()); ++i) {
        const auto& f = mesh.faces[i];
        Eigen::AlignedBox3d triBox;
        triBox.extend(mesh.vertices.row(f[0]).transpose().eval());
        triBox.extend(mesh.vertices.row(f[1]).transpose().eval());
        triBox.extend(mesh.vertices.row(f[2]).transpose().eval());
        if (box.intersects(triBox))
            result.push_back(i);
    }
    return result;
}

// ---------------------------------------------------------------------------
// expandMulti()
// ---------------------------------------------------------------------------
std::vector<Mesh> RobustSlicer::expandMulti(const Mesh& input, double d)
{
    if (input.empty() || input.faces.empty()) return {};

    // 1. Determine cell size
    double cellSize = cellSizeWorld_;
    if (cellSize <= math::kEpsilon) {
        // Resolution mode: adaptive to mesh AABB
        const Eigen::Vector3d vmin = input.vertices.colwise().minCoeff();
        const Eigen::Vector3d vmax = input.vertices.colwise().maxCoeff();
        const double aabbMaxDim = (vmax - vmin).maxCoeff();
        cellSize = (aabbMaxDim > math::kEpsilon)
                       ? aabbMaxDim / resolution_
                       : 1.0;
    }

    // 2. Voxelize
    VoxelGrid grid(cellSize);
    grid.build(input);

    // 3. Greedy box decomposition
    const auto boxes = grid.greedyMerge();
    if (boxes.empty()) return {};

    std::vector<Mesh> result;
    result.reserve(boxes.size());

    for (const auto& box : boxes) {
        // Expand the box by d to form the initial polytope boundary
        const Eigen::AlignedBox3d expandedBox(
            box.worldBounds.min() - Eigen::Vector3d::Constant(d),
            box.worldBounds.max() + Eigen::Vector3d::Constant(d));

        // 4a. Select faces whose AABB overlaps with the original box
        const auto faceIdxs = collectFaces(input, box.worldBounds);

        if (faceIdxs.empty()) {
            // No mesh surface in this box — emit the expanded box as-is
            Mesh m = ClippingEngine::clip(expandedBox, {});
            if (m.numFaces() > 0) result.push_back(std::move(m));
            continue;
        }

        // 4b. Collect face normals
        std::vector<Eigen::Vector3d> normals;
        normals.reserve(faceIdxs.size());
        for (int fi : faceIdxs) {
            const auto& f  = input.faces[fi];
            const Eigen::Vector3d v0 = input.vertices.row(f[0]).transpose();
            const Eigen::Vector3d v1 = input.vertices.row(f[1]).transpose();
            const Eigen::Vector3d v2 = input.vertices.row(f[2]).transpose();
            const Eigen::Vector3d n  = (v1 - v0).cross(v2 - v0);
            const double len = n.norm();
            if (len > math::kEpsilon) normals.push_back(n / len);
        }

        // 4c. Merge near-parallel normals
        const auto merged = math::mergeDirections(normals, faceNormalMergeDeg_);

        // 4d. Build half-spaces using LOCAL vertex support values.
        //     D_i = max(local_vertices . n_i) + d
        std::vector<math::HalfSpace> hs;
        hs.reserve(merged.size());
        for (const auto& n : merged) {
            double maxDot = -std::numeric_limits<double>::infinity();
            for (int fi : faceIdxs) {
                const auto& f = input.faces[fi];
                for (int k = 0; k < 3; ++k)
                    maxDot = std::max(maxDot,
                                      input.vertices.row(f[k]).dot(n));
            }
            hs.push_back({n, maxDot + d});
        }

        // 4e. Clip: expandedBox cut by local half-spaces
        Mesh m = ClippingEngine::clip(expandedBox, hs);
        if (m.numFaces() > 0) result.push_back(std::move(m));
    }

    return result;
}

// ---------------------------------------------------------------------------
// expand() — IExpander compatibility
// ---------------------------------------------------------------------------
Mesh RobustSlicer::expand(const Mesh& input, double d)
{
    auto meshes = expandMulti(input, d);
    if (meshes.empty()) return {};
    return std::move(meshes.front());
}

// ---------------------------------------------------------------------------
// merge() — concatenate multiple meshes into a single multi-body mesh
// ---------------------------------------------------------------------------
Mesh RobustSlicer::merge(const std::vector<Mesh>& meshes)
{
    int totalVerts = 0, totalFaces = 0;
    for (const auto& m : meshes) {
        totalVerts += m.numVertices();
        totalFaces += m.numFaces();
    }
    if (totalVerts == 0) return {};

    Mesh result;
    result.vertices.resize(totalVerts, 3);
    result.faces.reserve(totalFaces);

    int vOffset = 0;
    for (const auto& m : meshes) {
        result.vertices.middleRows(vOffset, m.numVertices()) = m.vertices;
        for (const auto& f : m.faces)
            result.faces.push_back({f[0] + vOffset, f[1] + vOffset, f[2] + vOffset});
        vOffset += m.numVertices();
    }
    return result;
}

// ---------------------------------------------------------------------------
// expandMerged()
// ---------------------------------------------------------------------------
Mesh RobustSlicer::expandMerged(const Mesh& input, double d)
{
    return merge(expandMulti(input, d));
}

} // namespace expander
