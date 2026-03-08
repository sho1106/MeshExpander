#pragma once
// ---------------------------------------------------------------------------
// ClippingEngine — Conservative convex-polytope clipping
//
// Algorithm:
//   1. Start with the given axis-aligned box as the initial polytope
//      (represented as 6 bounding half-spaces).
//   2. Append the caller-supplied half-spaces, each padded by kSafetyMargin
//      so that floating-point errors always stay on the safe (outer) side.
//   3. Enumerate all C(n,3) triples of planes, solve with ColPivHouseholderQR,
//      keep vertices that satisfy every half-space → polytope vertex set.
//   4. Assemble a closed triangle mesh via per-face angle-sort + fan triangulation.
//
// The Safety Margin guarantees the absolute-inclusion property: no original
// mesh vertex that is within `d` of the box region can fall outside the
// clipped polytope.
// ---------------------------------------------------------------------------

#include "Mesh.hpp"
#include "MathUtils.hpp"

#include <Eigen/Dense>
#include <vector>

namespace expander {

class ClippingEngine {
public:
    // Clip initBox by the given half-spaces.
    //   initBox     : axis-aligned box defining the initial polytope boundary
    //   halfSpaces  : caller-supplied planes, typically {n, max(V·n) + d}
    //                 kSafetyMargin is added to every distance internally
    // Returns a closed triangle mesh, or an empty Mesh if the result is degenerate.
    static Mesh clip(const Eigen::AlignedBox3d&          initBox,
                     const std::vector<math::HalfSpace>& halfSpaces);

private:
    // Convert an AlignedBox3d to its 6 bounding half-spaces.
    static std::vector<math::HalfSpace> boxFaceHalfSpaces(
        const Eigen::AlignedBox3d& box);

    // Find all vertices of the convex polytope defined by the half-space set.
    // Uses C(n,3) plane-triple intersection + ColPivHouseholderQR + dedup.
    static std::vector<Eigen::Vector3d> computeVertices(
        const std::vector<math::HalfSpace>& hs);

    // Build a closed triangle mesh from the polytope vertex cloud + half-spaces.
    // Per-face: collect on-plane vertices, sort by polar angle, fan-triangulate.
    static Mesh assembleMesh(const std::vector<Eigen::Vector3d>& verts,
                              const std::vector<math::HalfSpace>& hs);
};

} // namespace expander
