#include "expander/BoxExpander.hpp"
#include "expander/ClippingEngine.hpp"

#include <limits>

namespace expander {

BoxExpander::BoxExpander(double faceNormalMergeDeg)
    : faceNormalMergeDeg_(faceNormalMergeDeg)
{}

// ---------------------------------------------------------------------------
// collectFaces()
// Return indices of faces whose triangle AABB intersects the query box.
// ---------------------------------------------------------------------------
std::vector<int> BoxExpander::collectFaces(const Mesh&                mesh,
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
// faceNormals() — unit normals of the given faces (degenerate skipped)
// ---------------------------------------------------------------------------
std::vector<Eigen::Vector3d> BoxExpander::faceNormals(
    const Mesh&             mesh,
    const std::vector<int>& faceIdx,
    std::vector<int>*       outValidFaces)
{
    std::vector<Eigen::Vector3d> normals;
    normals.reserve(faceIdx.size());
    if (outValidFaces) { outValidFaces->clear(); outValidFaces->reserve(faceIdx.size()); }

    for (int fi : faceIdx) {
        const auto& f = mesh.faces[fi];
        const Eigen::Vector3d n = math::triangleNormal(
            mesh.vertices.row(f[0]).transpose(),
            mesh.vertices.row(f[1]).transpose(),
            mesh.vertices.row(f[2]).transpose());
        if (n.squaredNorm() > 0.5) {   // skip degenerate (zero) normals
            normals.push_back(n);
            if (outValidFaces) outValidFaces->push_back(fi);
        }
    }
    return normals;
}

// ---------------------------------------------------------------------------
// maxSupport() — max projection of the face set's vertices onto n
// ---------------------------------------------------------------------------
double BoxExpander::maxSupport(const Mesh&             mesh,
                               const std::vector<int>& faceIdx,
                               const Eigen::Vector3d&  n)
{
    double maxDot = -std::numeric_limits<double>::infinity();
    for (int fi : faceIdx) {
        const auto& f = mesh.faces[fi];
        for (int k = 0; k < 3; ++k)
            maxDot = std::max(maxDot, mesh.vertices.row(f[k]).dot(n));
    }
    return maxDot;
}

// ---------------------------------------------------------------------------
// expand(box, mesh, d) — core carving function
// ---------------------------------------------------------------------------
Mesh BoxExpander::expand(const Eigen::AlignedBox3d& box,
                          const Mesh&                mesh,
                          double                     d) const
{
    if (mesh.empty() || mesh.faces.empty()) return {};

    // 1. Expand the box by d to form the initial polytope boundary
    const Eigen::AlignedBox3d expandedBox(
        box.min() - Eigen::Vector3d::Constant(d),
        box.max() + Eigen::Vector3d::Constant(d));

    // 2. Select faces whose AABB overlaps with the original box
    const auto faceIdxs = collectFaces(mesh, box);

    if (faceIdxs.empty()) {
        // No mesh surface in this box — emit the expanded box as-is
        return ClippingEngine::clip(expandedBox, {});
    }

    // 3. Collect face normals
    const auto normals = faceNormals(mesh, faceIdxs);

    // 4. Merge near-parallel normals
    const auto merged = math::mergeDirections(normals, faceNormalMergeDeg_);

    // 5. Build half-spaces: D_i = max(local_vertices . n_i) + d
    std::vector<math::HalfSpace> hs;
    hs.reserve(merged.size());
    for (const auto& n : merged)
        hs.push_back({n, maxSupport(mesh, faceIdxs, n) + d});

    // 6. Clip: expandedBox carved by local half-spaces → single convex polytope
    return ClippingEngine::clip(expandedBox, hs);
}

// ---------------------------------------------------------------------------
// expand(mesh, d) — convenience: use mesh's own AABB as the box
// ---------------------------------------------------------------------------
Mesh BoxExpander::expand(const Mesh& mesh, double d)
{
    if (mesh.empty() || mesh.faces.empty()) return {};
    const Eigen::AlignedBox3d box(
        mesh.vertices.colwise().minCoeff().transpose(),
        mesh.vertices.colwise().maxCoeff().transpose());
    return expand(box, mesh, d);
}

} // namespace expander
