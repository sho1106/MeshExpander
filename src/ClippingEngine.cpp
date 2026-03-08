#include "expander/ClippingEngine.hpp"

#include <Eigen/QR>
#include <algorithm>
#include <cmath>

namespace expander {

using math::HalfSpace;
using math::kEpsilon;
using math::kOnPlaneEps;
using math::kInsideEps;
using math::kSafetyMargin;

// ---------------------------------------------------------------------------
// clip() — public entry point
// ---------------------------------------------------------------------------
Mesh ClippingEngine::clip(const Eigen::AlignedBox3d&          initBox,
                           const std::vector<math::HalfSpace>& halfSpaces)
{
    // Start with box face planes (hard boundary, no safety margin needed).
    auto hs = boxFaceHalfSpaces(initBox);

    // Merge user half-spaces into the list.
    // If a user half-space is nearly parallel to an existing one (dot > 1-1e-8),
    // replace the existing distance with the larger (outer) value so only ONE
    // plane per direction is kept.  This prevents assembleMesh from seeing two
    // nearly-coincident faces (e.g. box top at D and user top at D+kSafetyMargin)
    // and triangulating the same cap twice, which corrupts the volume computation.
    hs.reserve(hs.size() + halfSpaces.size());
    for (const auto& h : halfSpaces) {
        const double D = h.distance + kSafetyMargin;
        bool merged = false;
        for (auto& existing : hs) {
            if (h.normal.dot(existing.normal) > 1.0 - 1e-8) {
                // Keep the TIGHTER (smaller D) constraint so only one plane per
                // direction exists in hs.  This prevents assembleMesh from seeing
                // two nearly-coincident parallel faces (e.g. box top at D=1.1 and
                // user top at D=1.1+kSafetyMargin) and triangulating the cap twice.
                if (D < existing.distance)
                    existing.distance = D;
                merged = true;
                break;
            }
        }
        if (!merged)
            hs.push_back({h.normal, D});
    }

    // Find all polytope vertices
    const auto verts = computeVertices(hs);
    if (verts.size() < 4) return {};   // degenerate (< tetrahedron)

    return assembleMesh(verts, hs);
}

// ---------------------------------------------------------------------------
// boxFaceHalfSpaces()
// ---------------------------------------------------------------------------
std::vector<HalfSpace> ClippingEngine::boxFaceHalfSpaces(
    const Eigen::AlignedBox3d& box)
{
    const Eigen::Vector3d mn = box.min();
    const Eigen::Vector3d mx = box.max();
    return {
        { Eigen::Vector3d( 1, 0, 0),  mx.x() },
        { Eigen::Vector3d(-1, 0, 0), -mn.x() },
        { Eigen::Vector3d( 0, 1, 0),  mx.y() },
        { Eigen::Vector3d( 0,-1, 0), -mn.y() },
        { Eigen::Vector3d( 0, 0, 1),  mx.z() },
        { Eigen::Vector3d( 0, 0,-1), -mn.z() },
    };
}

// ---------------------------------------------------------------------------
// computeVertices()
// Enumerate all C(n,3) plane triples, solve with ColPivHouseholderQR,
// keep points satisfying all half-spaces, deduplicate.
// ---------------------------------------------------------------------------
std::vector<Eigen::Vector3d> ClippingEngine::computeVertices(
    const std::vector<HalfSpace>& hs)
{
    const int n = static_cast<int>(hs.size());
    std::vector<Eigen::Vector3d> candidates;
    candidates.reserve(n * n);

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

                Eigen::ColPivHouseholderQR<Eigen::Matrix3d> qr(A);
                if (qr.rank() < 3) continue;

                const Eigen::Vector3d pt = qr.solve(b);
                if (!pt.allFinite()) continue;

                // Keep point only if it satisfies ALL half-spaces
                // (with a small inward tolerance to accept near-boundary vertices)
                if (math::isInsideAll(pt, hs, kInsideEps))
                    candidates.push_back(pt);
            }
        }
    }

    // Deduplicate: merge vertices closer than kOnPlaneEps
    std::vector<Eigen::Vector3d> unique;
    unique.reserve(candidates.size());
    for (const auto& c : candidates) {
        bool dup = false;
        for (const auto& u : unique)
            if ((c - u).squaredNorm() < kOnPlaneEps * kOnPlaneEps) { dup = true; break; }
        if (!dup) unique.push_back(c);
    }
    return unique;
}

// ---------------------------------------------------------------------------
// assembleMesh()
// For each half-space face, collect on-plane vertices, sort CCW, fan-triangulate.
// ---------------------------------------------------------------------------
Mesh ClippingEngine::assembleMesh(const std::vector<Eigen::Vector3d>& verts,
                                   const std::vector<HalfSpace>&        hs)
{
    // Pack vertex cloud into Eigen matrix
    Eigen::MatrixXd V(static_cast<int>(verts.size()), 3);
    for (int i = 0; i < static_cast<int>(verts.size()); ++i)
        V.row(i) = verts[i].transpose();

    std::vector<Eigen::Vector3i> faces;

    for (const auto& h : hs) {
        // Collect vertices that lie on this face (within kOnPlaneEps of the plane)
        std::vector<int> onFace;
        for (int i = 0; i < static_cast<int>(verts.size()); ++i) {
            if (std::abs(h.normal.dot(verts[i]) - h.distance) < kOnPlaneEps)
                onFace.push_back(i);
        }
        if (static_cast<int>(onFace.size()) < 3) continue;

        // Centroid of the face polygon
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (int idx : onFace) centroid += verts[idx];
        centroid /= static_cast<double>(onFace.size());

        // Orthonormal basis on the face plane
        Eigen::Vector3d ref = (verts[onFace[0]] - centroid);
        if (ref.squaredNorm() < kEpsilon) continue;  // degenerate face
        ref.normalize();
        Eigen::Vector3d tan2 = h.normal.cross(ref).normalized();

        // Sort vertices by polar angle (CCW when viewed from outside)
        std::vector<std::pair<double, int>> angleVtx;
        angleVtx.reserve(onFace.size());
        for (int idx : onFace) {
            Eigen::Vector3d dv = verts[idx] - centroid;
            angleVtx.emplace_back(std::atan2(dv.dot(tan2), dv.dot(ref)), idx);
        }
        std::sort(angleVtx.begin(), angleVtx.end());

        // Fan triangulation from first vertex
        const int v0 = angleVtx[0].second;
        for (int t = 1; t + 1 < static_cast<int>(angleVtx.size()); ++t) {
            const int v1 = angleVtx[t].second;
            const int v2 = angleVtx[t + 1].second;

            // Enforce outward winding
            Eigen::Vector3d faceN = (verts[v1] - verts[v0]).cross(verts[v2] - verts[v0]);
            if (faceN.dot(h.normal) >= 0.0)
                faces.push_back({v0, v1, v2});
            else
                faces.push_back({v0, v2, v1});
        }
    }

    return {V, faces};
}

} // namespace expander
