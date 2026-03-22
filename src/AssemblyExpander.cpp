#include "expander/AssemblyExpander.hpp"
#include "expander/ConservativeExpander.hpp"
#include "expander/RobustSlicer.hpp"

#include <algorithm>
#include <limits>
#include <numeric>

namespace expander {

// ---------------------------------------------------------------------------
// Ctor
// ---------------------------------------------------------------------------
AssemblyExpander::AssemblyExpander(Options opts)
    : opts_(opts)
{}

// ---------------------------------------------------------------------------
// isConvex()
// For each face plane, all vertices must lie on the inner side (n·v <= d+tol).
// ---------------------------------------------------------------------------
bool AssemblyExpander::isConvex(const Mesh& mesh, double tol)
{
    if (mesh.faces.empty() || mesh.numVertices() < 4) return true;

    for (const auto& f : mesh.faces) {
        const Eigen::Vector3d v0 = mesh.vertices.row(f[0]).transpose();
        const Eigen::Vector3d v1 = mesh.vertices.row(f[1]).transpose();
        const Eigen::Vector3d v2 = mesh.vertices.row(f[2]).transpose();
        const Eigen::Vector3d n  = (v1 - v0).cross(v2 - v0);
        const double len = n.norm();
        if (len < math::kEpsilon) continue;  // degenerate face — skip

        const Eigen::Vector3d nHat = n / len;
        const double d = nHat.dot(v0);

        // All vertices must be on the inside of this face plane
        for (int vi = 0; vi < mesh.numVertices(); ++vi) {
            if (nHat.dot(mesh.vertices.row(vi).transpose()) > d + tol)
                return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// mergeContained()
// ---------------------------------------------------------------------------
std::vector<Mesh> AssemblyExpander::mergeContained(const std::vector<Mesh>& parts,
                                                    double tolerance)
{
    const int n = static_cast<int>(parts.size());
    if (n <= 1) return parts;

    // Compute bounding boxes and volumes for each part
    struct BB { Eigen::Vector3d lo, hi; double vol; };
    std::vector<BB> bbs(n);
    for (int i = 0; i < n; ++i) {
        if (parts[i].empty()) {
            bbs[i] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0};
            continue;
        }
        bbs[i].lo = parts[i].vertices.colwise().minCoeff().transpose();
        bbs[i].hi = parts[i].vertices.colwise().maxCoeff().transpose();
        const Eigen::Vector3d dim = bbs[i].hi - bbs[i].lo;
        bbs[i].vol = dim.x() * dim.y() * dim.z();
    }

    // Process from smallest to largest so inner nesting resolves transitively
    std::vector<int> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
              [&](int a, int b) { return bbs[a].vol < bbs[b].vol; });

    // Work on mutable copies
    std::vector<Mesh> result = parts;
    std::vector<bool> absorbed(n, false);

    for (int oi = 0; oi < n; ++oi) {
        const int j = order[oi];
        if (absorbed[j] || result[j].empty()) continue;

        // Find smallest container for j
        int bestContainer = -1;
        double bestVol    = std::numeric_limits<double>::infinity();

        for (int i = 0; i < n; ++i) {
            if (i == j || absorbed[i] || result[i].empty()) continue;
            if (bbs[i].vol <= bbs[j].vol) continue;  // i must be larger

            // Check: BB(j) ⊆ BB(i) with tolerance
            const Eigen::Vector3d& lo_i = bbs[i].lo;
            const Eigen::Vector3d& hi_i = bbs[i].hi;
            const Eigen::Vector3d& lo_j = bbs[j].lo;
            const Eigen::Vector3d& hi_j = bbs[j].hi;

            bool contained =
                (lo_i.array() <= lo_j.array() + tolerance).all() &&
                (hi_i.array() >= hi_j.array() - tolerance).all();

            if (contained && bbs[i].vol < bestVol) {
                bestContainer = i;
                bestVol = bbs[i].vol;
            }
        }

        if (bestContainer < 0) continue;

        // Absorb j into bestContainer: append vertices and faces
        Mesh& dst = result[bestContainer];
        const Mesh& src = result[j];
        const int vOffset = dst.numVertices();

        Eigen::MatrixXd newV(vOffset + src.numVertices(), 3);
        newV.topRows(vOffset)    = dst.vertices;
        newV.bottomRows(src.numVertices()) = src.vertices;
        dst.vertices = std::move(newV);

        dst.faces.reserve(dst.numFaces() + src.numFaces());
        for (const auto& f : src.faces)
            dst.faces.push_back({f[0] + vOffset, f[1] + vOffset, f[2] + vOffset});

        absorbed[j] = true;
    }

    // Return non-absorbed parts only
    std::vector<Mesh> out;
    out.reserve(n);
    for (int i = 0; i < n; ++i)
        if (!absorbed[i]) out.push_back(std::move(result[i]));
    return out;
}

// ---------------------------------------------------------------------------
// expandPart() — choose algorithm based on options
// ---------------------------------------------------------------------------
Mesh AssemblyExpander::expandPart(const Mesh& part, double d) const
{
    if (part.empty() || part.faces.empty()) return {};

    if (!opts_.useVoxelPartitioning) {
        // Default: ConservativeExpander for all parts.
        // Part boundaries are driven by the file's mesh structure.
        ConservativeExpander exp({}, opts_.faceNormalMergeDeg);
        return exp.expand(part, d);
    }

    // Opt-in voxel partitioning: use RobustSlicer for concave parts.
    if (isConvex(part, opts_.convexTol)) {
        ConservativeExpander exp({}, opts_.faceNormalMergeDeg);
        return exp.expand(part, d);
    } else {
        RobustSlicer slicer = (opts_.cellSizeWorld > math::kEpsilon)
            ? RobustSlicer::withCellSize(opts_.cellSizeWorld, opts_.faceNormalMergeDeg)
            : RobustSlicer(opts_.resolution, opts_.faceNormalMergeDeg);
        return slicer.expandMerged(part, d);
    }
}

// ---------------------------------------------------------------------------
// mergeMeshes() — concatenate mesh list into a single multi-body mesh
// ---------------------------------------------------------------------------
Mesh AssemblyExpander::mergeMeshes(const std::vector<Mesh>& meshes)
{
    int totalV = 0, totalF = 0;
    for (const auto& m : meshes) { totalV += m.numVertices(); totalF += m.numFaces(); }
    if (totalV == 0) return {};

    Mesh out;
    out.vertices.resize(totalV, 3);
    out.faces.reserve(totalF);

    int vOff = 0;
    for (const auto& m : meshes) {
        out.vertices.middleRows(vOff, m.numVertices()) = m.vertices;
        for (const auto& f : m.faces)
            out.faces.push_back({f[0] + vOff, f[1] + vOff, f[2] + vOff});
        vOff += m.numVertices();
    }
    return out;
}

// ---------------------------------------------------------------------------
// expand()
// ---------------------------------------------------------------------------
std::vector<Mesh> AssemblyExpander::expand(const std::vector<Mesh>& parts,
                                           double d) const
{
    std::vector<Mesh> result;
    result.reserve(parts.size());
    for (const auto& p : parts)
        result.push_back(expandPart(p, d));
    return result;
}

// ---------------------------------------------------------------------------
// expandMerged()
// ---------------------------------------------------------------------------
Mesh AssemblyExpander::expandMerged(const std::vector<Mesh>& parts, double d) const
{
    return mergeMeshes(expand(parts, d));
}

} // namespace expander
