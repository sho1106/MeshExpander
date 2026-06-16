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
    return expand(box, mesh, Eigen::Vector3d::Constant(d));
}

Mesh BoxExpander::expand(const Eigen::AlignedBox3d& box,
                          const Mesh&                mesh,
                          const Eigen::Vector3d&     d) const
{
    if (mesh.empty() || mesh.faces.empty()) return {};

    // 1. Expand the box by d (per-axis) to form the initial polytope boundary
    const Eigen::AlignedBox3d expandedBox(
        box.min() - d,
        box.max() + d);

    // Normalize to a canonical size before clipping. ClippingEngine uses absolute
    // tolerances (kOnPlaneEps, kSafetyMargin); without this, a micron-scale model
    // would collapse to nothing and a kilometre-scale one would be needlessly
    // coarse. We map the expanded box to roughly [-1, 1]^3, clip there, then map
    // the result back — the polytope is identical, only the tolerances become
    // scale-relative. (A point transforms as x_n = (x - center) * s.)
    const Eigen::Vector3d center = expandedBox.center();
    const double extent = (expandedBox.max() - expandedBox.min()).maxCoeff();
    const double s      = (extent > 0.0) ? (2.0 / extent) : 1.0;

    const Eigen::AlignedBox3d boxN(
        (expandedBox.min() - center) * s,
        (expandedBox.max() - center) * s);

    // 2. Select faces whose AABB overlaps with the original box
    const auto faceIdxs = collectFaces(mesh, box);

    std::vector<math::HalfSpace> hs;
    if (!faceIdxs.empty()) {
        // 3-5. Collect normals, merge near-parallel ones, build half-spaces.
        // Half-space {x : n·x ≤ D} maps to {x_n : n·x_n ≤ (D − n·center)·s}.
        const auto normals = faceNormals(mesh, faceIdxs);
        const auto merged  = math::mergeDirections(normals, faceNormalMergeDeg_);
        hs.reserve(merged.size());
        for (const auto& n : merged) {
            // Effective offset along unit normal n is the support of an ellipsoid
            // with semi-axes (dx, dy, dz):  off = sqrt((n_x·dx)^2 + (n_y·dy)^2 +
            // (n_z·dz)^2). An axis-aligned face is offset by exactly its component
            // (n = +X → dx); a uniform d = (d,d,d) reduces to off = d for every
            // unit normal (identical to the isotropic ball expansion). off >= 0,
            // so conservativeness is preserved.
            const double off = n.cwiseProduct(d).norm();
            const double D   = maxSupport(mesh, faceIdxs, n) + off;  // world units
            hs.push_back({n, (D - n.dot(center)) * s});             // normalized
        }
    }
    // (faceIdxs empty → no carving half-spaces; the expanded box is emitted as-is)

    // 6. Clip in normalized space, then denormalize: x = x_n / s + center.
    Mesh out = ClippingEngine::clip(boxN, hs);
    for (int i = 0; i < out.numVertices(); ++i)
        out.vertices.row(i) =
            (out.vertices.row(i).transpose() / s + center).transpose();
    return out;
}

// ---------------------------------------------------------------------------
// expand(mesh, d) — convenience: use mesh's own AABB as the box
// ---------------------------------------------------------------------------
Mesh BoxExpander::expand(const Mesh& mesh, double d)
{
    return expand(mesh, Eigen::Vector3d::Constant(d));
}

Mesh BoxExpander::expand(const Mesh& mesh, const Eigen::Vector3d& d) const
{
    if (mesh.empty() || mesh.faces.empty()) return {};
    const Eigen::AlignedBox3d box(
        mesh.vertices.colwise().minCoeff().transpose(),
        mesh.vertices.colwise().maxCoeff().transpose());
    return expand(box, mesh, d);
}

} // namespace expander
