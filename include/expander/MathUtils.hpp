#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace expander {
namespace math {

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
constexpr double kEpsilon         = 1e-9;
constexpr double kOnPlaneEps      = 1e-5;  // tolerance for "vertex lies on plane"
constexpr double kInsideEps       = 1e-6;  // tolerance for half-space inclusion check
constexpr double kMergeAngleDeg   = 5.0;   // merge directions closer than this
constexpr double kSafetyMargin    = 1e-6;  // outward padding added to every clipping plane
                                           //   ensures float errors stay on the safe (outer) side

// ---------------------------------------------------------------------------
// Half-space: { x in R3 | normal . x <= distance }
// ---------------------------------------------------------------------------
struct HalfSpace {
    Eigen::Vector3d normal;   // unit vector (outward)
    double          distance; // support value + expansion offset
};

// ---------------------------------------------------------------------------
// Normalization
// ---------------------------------------------------------------------------
struct NormalizationInfo {
    Eigen::Vector3d center;
    double          scale;     // world -> normalized:  x_n = (x_w - center) * scale
    double          scaleInv;  // normalized -> world:  x_w = x_n * scaleInv + center
};

inline NormalizationInfo computeNormalization(const Eigen::MatrixXd& V) {
    NormalizationInfo info;
    Eigen::Vector3d vmin = V.colwise().minCoeff();
    Eigen::Vector3d vmax = V.colwise().maxCoeff();
    info.center   = 0.5 * (vmin + vmax);

    // Uniform scale: longest AABB edge maps to [-1, 1]
    double maxExtent = (vmax - vmin).maxCoeff();
    info.scale    = (maxExtent > kEpsilon) ? 2.0 / maxExtent : 1.0;
    info.scaleInv = 1.0 / info.scale;
    return info;
}

inline Eigen::MatrixXd applyNormalization(const Eigen::MatrixXd& V,
                                           const NormalizationInfo& info) {
    return (V.rowwise() - info.center.transpose()) * info.scale;
}

inline Eigen::MatrixXd applyDenormalization(const Eigen::MatrixXd& V,
                                              const NormalizationInfo& info) {
    return (V * info.scaleInv).rowwise() + info.center.transpose();
}

// ---------------------------------------------------------------------------
// Direction sets
// ---------------------------------------------------------------------------

// 26 directions from 3D grid neighbors (all (i,j,k) in {-1,0,1}^3 \ {0})
inline std::vector<Eigen::Vector3d> generate26Directions() {
    std::vector<Eigen::Vector3d> dirs;
    dirs.reserve(26);
    for (int x = -1; x <= 1; ++x)
        for (int y = -1; y <= 1; ++y)
            for (int z = -1; z <= 1; ++z) {
                if (x == 0 && y == 0 && z == 0) continue;
                dirs.push_back(Eigen::Vector3d(x, y, z).normalized());
            }
    return dirs;
}

// Remove directions within angleThresholdDeg of an already-accepted direction.
// Anti-parallel directions (dot < 0) are treated as DISTINCT (they constrain
// opposite sides of the polytope).
inline std::vector<Eigen::Vector3d> mergeDirections(
    const std::vector<Eigen::Vector3d>& dirs,
    double angleThresholdDeg = kMergeAngleDeg)
{
    const double cosThresh = std::cos(angleThresholdDeg * (M_PI / 180.0));
    std::vector<Eigen::Vector3d> result;
    for (const auto& d : dirs) {
        bool duplicate = false;
        for (const auto& r : result) {
            if (d.dot(r) > cosThresh) {   // same hemisphere, nearly parallel
                duplicate = true;
                break;
            }
        }
        if (!duplicate) result.push_back(d);
    }
    return result;
}

// ---------------------------------------------------------------------------
// Half-space helpers
// ---------------------------------------------------------------------------

// Build one half-space per direction from a normalized vertex set.
// D_i = max_{v in V_n} (v . n_i) + d_scaled
inline std::vector<HalfSpace> buildHalfSpaces(
    const Eigen::MatrixXd&             Vn,
    const std::vector<Eigen::Vector3d>& dirs,
    double                              dScaled)
{
    auto merged = mergeDirections(dirs);
    std::vector<HalfSpace> hs;
    hs.reserve(merged.size());
    for (const auto& n : merged) {
        double maxDot = (Vn * n).maxCoeff();
        hs.push_back({n, maxDot + dScaled});
    }
    return hs;
}

// Test whether point pt satisfies ALL half-spaces (with tolerance eps).
inline bool isInsideAll(const Eigen::Vector3d&          pt,
                         const std::vector<HalfSpace>& hs,
                         double                         eps = kInsideEps)
{
    for (const auto& h : hs)
        if (h.normal.dot(pt) > h.distance + eps) return false;
    return true;
}

} // namespace math
} // namespace expander
