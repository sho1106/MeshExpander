#include "expander/ClippingEngine.hpp"

#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>

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

    // Vertex enumeration is O(n^3) (every plane triple). In normal use n is the
    // 6 box faces plus a handful of merged face-normal directions (typically
    // < 80, since mergeDirections collapses normals within ~20°). Guard against
    // pathological inputs that would make this explode into minutes / OOM.
    static constexpr int kMaxHalfSpaces = 256;
    if (n > kMaxHalfSpaces)
        throw std::runtime_error(
            "ClippingEngine: too many half-spaces (" + std::to_string(n) +
            " > " + std::to_string(kMaxHalfSpaces) + "); vertex enumeration is "
            "O(n^3). Increase the face-normal merge angle to reduce direction count.");

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

    // Deduplicate: merge vertices closer than kOnPlaneEps.
    // Spatial hash with cell size kOnPlaneEps: two points within kOnPlaneEps
    // differ by at most one cell per axis, so scanning the 3x3x3 neighborhood
    // finds every potential duplicate in O(candidates) instead of O(n^2).
    using Cell = std::tuple<std::int64_t, std::int64_t, std::int64_t>;
    struct CellHash {
        std::size_t operator()(const Cell& c) const {
            std::size_t h = std::hash<std::int64_t>{}(std::get<0>(c));
            h ^= std::hash<std::int64_t>{}(std::get<1>(c)) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
            h ^= std::hash<std::int64_t>{}(std::get<2>(c)) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
            return h;
        }
    };

    std::vector<Eigen::Vector3d> unique;
    unique.reserve(candidates.size());
    std::unordered_map<Cell, std::vector<int>, CellHash> grid;
    grid.reserve(candidates.size() * 2);

    const double inv = 1.0 / kOnPlaneEps;
    const double eps2 = kOnPlaneEps * kOnPlaneEps;
    auto cellOf = [&](const Eigen::Vector3d& p) {
        return Cell{ static_cast<std::int64_t>(std::floor(p.x() * inv)),
                     static_cast<std::int64_t>(std::floor(p.y() * inv)),
                     static_cast<std::int64_t>(std::floor(p.z() * inv)) };
    };

    for (const auto& c : candidates) {
        const Cell base = cellOf(c);
        bool dup = false;
        for (int dx = -1; dx <= 1 && !dup; ++dx)
            for (int dy = -1; dy <= 1 && !dup; ++dy)
                for (int dz = -1; dz <= 1 && !dup; ++dz) {
                    const Cell nb{ std::get<0>(base) + dx,
                                   std::get<1>(base) + dy,
                                   std::get<2>(base) + dz };
                    auto it = grid.find(nb);
                    if (it == grid.end()) continue;
                    for (int ui : it->second)
                        if ((c - unique[ui]).squaredNorm() < eps2) { dup = true; break; }
                }
        if (!dup) {
            grid[base].push_back(static_cast<int>(unique.size()));
            unique.push_back(c);
        }
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
