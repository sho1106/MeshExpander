#include "expander/ConservativeExpander.hpp"

#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace expander {

using math::HalfSpace;
using math::kEpsilon;
using math::kOnPlaneEps;
using math::kInsideEps;

// ---------------------------------------------------------------------------
// Ctor
// ---------------------------------------------------------------------------
ConservativeExpander::ConservativeExpander(std::vector<Eigen::Vector3d> dirs,
                                           double faceNormalMergeDeg)
    : dirs_(std::move(dirs))
    , faceNormalMergeDeg_(faceNormalMergeDeg) {}

// ---------------------------------------------------------------------------
// expand()
// ---------------------------------------------------------------------------
Mesh ConservativeExpander::expand(const Mesh& input, double expansionDist) {
    if (input.empty()) return {};

    // 1. Uniform normalization
    const auto info    = math::computeNormalization(input.vertices);
    const auto Vn      = math::applyNormalization(input.vertices, info);
    const double dSc   = expansionDist * info.scale;

    // 2. Half-space generation
    //    Face-normal mode: use mesh face normals (shape-adaptive, no apex dominance).
    //    Fallback mode:    use constructor-provided fixed directions (default: 26-dir).
    std::vector<math::HalfSpace> hs;
    if (!input.faces.empty()) {
        // Collect face normals in normalized space
        std::vector<Eigen::Vector3d> faceNormals;
        faceNormals.reserve(input.faces.size());
        for (const auto& f : input.faces) {
            const Eigen::Vector3d v0 = Vn.row(f[0]).transpose();
            const Eigen::Vector3d v1 = Vn.row(f[1]).transpose();
            const Eigen::Vector3d v2 = Vn.row(f[2]).transpose();
            const Eigen::Vector3d n  = (v1 - v0).cross(v2 - v0);
            const double len = n.norm();
            if (len > kEpsilon) faceNormals.push_back(n / len);
        }
        // Merge near-duplicate directions then build half-spaces
        const auto merged = math::mergeDirections(faceNormals, faceNormalMergeDeg_);
        hs.reserve(merged.size());
        for (const auto& n : merged) {
            double maxDot = (Vn * n).maxCoeff();
            hs.push_back({n, maxDot + dSc});
        }
    } else {
        // No face data — fall back to fixed directions
        hs = math::buildHalfSpaces(Vn, dirs_, dSc);
    }
    if (hs.size() < 3) return {};

    // 3. Polytope vertex extraction
    const auto verts = intersectHalfSpaces(hs);
    if (verts.empty()) return {};

    // 4. Mesh assembly (in normalized space)
    Mesh result = buildMesh(verts, hs);

    // 5. Denormalize
    result.vertices = math::applyDenormalization(result.vertices, info);
    return result;
}

// ---------------------------------------------------------------------------
// intersectHalfSpaces()
// Enumerate all C(n,3) triples of planes, solve the 3x3 system with
// ColPivHouseholderQR, and keep points that lie inside every half-space.
// ---------------------------------------------------------------------------
std::vector<Eigen::Vector3d> ConservativeExpander::intersectHalfSpaces(
    const std::vector<HalfSpace>& hs) const
{
    const int n = static_cast<int>(hs.size());
    std::vector<Eigen::Vector3d> candidates;
    candidates.reserve(n * n);  // rough upper bound

    for (int i = 0; i < n - 2; ++i) {
        for (int j = i + 1; j < n - 1; ++j) {
            for (int k = j + 1; k < n; ++k) {
                Eigen::Matrix3d A;
                A.row(0) = hs[i].normal.transpose();
                A.row(1) = hs[j].normal.transpose();
                A.row(2) = hs[k].normal.transpose();

                Eigen::Vector3d b(hs[i].distance,
                                  hs[j].distance,
                                  hs[k].distance);

                // Rank-revealing decomposition — handles near-parallel planes
                Eigen::ColPivHouseholderQR<Eigen::Matrix3d> qr(A);
                if (qr.rank() < 3) continue;

                const Eigen::Vector3d pt = qr.solve(b);
                if (!pt.allFinite()) continue;

                if (math::isInsideAll(pt, hs, kInsideEps))
                    candidates.push_back(pt);
            }
        }
    }

    // Deduplicate (floating-point vertices from different triples may coincide)
    const double dedupEps = kOnPlaneEps;
    std::vector<Eigen::Vector3d> unique;
    unique.reserve(candidates.size());
    for (const auto& c : candidates) {
        bool dup = false;
        for (const auto& u : unique) {
            if ((c - u).squaredNorm() < dedupEps * dedupEps) { dup = true; break; }
        }
        if (!dup) unique.push_back(c);
    }
    return unique;
}

// ---------------------------------------------------------------------------
// buildMesh()
// For each half-space face, collect boundary vertices, sort them in
// CCW order (when viewed from outside), fan-triangulate.
// ---------------------------------------------------------------------------
Mesh ConservativeExpander::buildMesh(const std::vector<Eigen::Vector3d>& verts,
                                      const std::vector<HalfSpace>&        hs) const
{
    // Pack into Eigen matrix
    Eigen::MatrixXd V(static_cast<int>(verts.size()), 3);
    for (int i = 0; i < static_cast<int>(verts.size()); ++i)
        V.row(i) = verts[i].transpose();

    std::vector<Eigen::Vector3i> faces;

    for (const auto& h : hs) {
        // Vertices whose support value equals D_i (they lie on the face plane)
        std::vector<int> onFace;
        for (int i = 0; i < static_cast<int>(verts.size()); ++i) {
            double val = h.normal.dot(verts[i]);
            if (std::abs(val - h.distance) < kOnPlaneEps)
                onFace.push_back(i);
        }
        if (static_cast<int>(onFace.size()) < 3) continue;

        // Centroid of this face polygon
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (int idx : onFace) centroid += verts[idx];
        centroid /= static_cast<double>(onFace.size());

        // Local 2-D basis on the plane (orthonormal, consistent with h.normal)
        Eigen::Vector3d ref = (verts[onFace[0]] - centroid);
        if (ref.squaredNorm() < kEpsilon) {
            // All points coincide with centroid — degenerate face, skip
            continue;
        }
        ref.normalize();
        Eigen::Vector3d tan2 = h.normal.cross(ref).normalized();

        // Sort vertices by polar angle in the local frame → CCW from outside
        std::vector<std::pair<double, int>> angleVtx;
        angleVtx.reserve(onFace.size());
        for (int idx : onFace) {
            Eigen::Vector3d dv = verts[idx] - centroid;
            double angle = std::atan2(dv.dot(tan2), dv.dot(ref));
            angleVtx.emplace_back(angle, idx);
        }
        std::sort(angleVtx.begin(), angleVtx.end());

        // Fan triangulation from first vertex
        const int v0 = angleVtx[0].second;
        for (int t = 1; t + 1 < static_cast<int>(angleVtx.size()); ++t) {
            const int v1 = angleVtx[t].second;
            const int v2 = angleVtx[t + 1].second;

            // Ensure winding is consistent with outward normal
            Eigen::Vector3d faceN = (verts[v1] - verts[v0])
                                    .cross(verts[v2] - verts[v0]);
            if (faceN.dot(h.normal) >= 0.0)
                faces.push_back({v0, v1, v2});
            else
                faces.push_back({v0, v2, v1});
        }
    }

    return {V, faces};
}

} // namespace expander
