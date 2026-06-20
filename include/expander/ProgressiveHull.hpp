#pragma once
// ---------------------------------------------------------------------------
// ProgressiveHull — Conservative progressive-hull simplifier
//
// A carved polytope is the intersection of half-spaces.  REMOVING a half-space
// only ENLARGES the polytope, so the result still CONTAINS the original — the
// simplification stays conservative and volume only ever grows.
//
// Algorithm (greedy, volume-guided):
//   1. Start from the full half-space set; compute the carved polytope volume V0
//      (divergence theorem on the single closed convex polytope).
//   2. Each step, for every removable half-space, re-clip WITHOUT it and compute
//      the resulting volume.  Pick the removal that grows the volume the least.
//   3. Apply that removal if it stays within the volume budget; repeat until the
//      target face count is reached or no removal fits the budget.
//
// Removals that make the polytope degenerate / unbounded (clip returns < 4
// vertices or a non-finite volume) are skipped, so the result is always a valid
// closed polytope.  The 6 AABB box faces are supplied to ClippingEngine::clip()
// separately and are never candidates for removal, guaranteeing boundedness.
// ---------------------------------------------------------------------------

#include "Mesh.hpp"
#include "MathUtils.hpp"
#include "ClippingEngine.hpp"

#include <Eigen/Geometry>
#include <vector>

namespace expander {

class ProgressiveHull {
public:
    struct Options {
        // Stop once the polytope has at most this many faces (<= 0 disables the
        // face-count criterion). The greedy loop ends when either criterion fires.
        int maxFaces = 0;

        // Stop before the volume exceeds (1 + maxVolumeGrowth) * V0, where V0 is
        // the volume of the original (un-simplified) carved polytope.
        // <= 0 disables the volume-budget criterion (face count alone then stops).
        double maxVolumeGrowth = 0.10;
    };

    // Simplify the polytope { x | box-faces AND halfSpaces }.
    //   box        : axis-aligned box passed straight to ClippingEngine::clip;
    //                its 6 faces bound the polytope and are never removed.
    //   halfSpaces : caller-supplied carving planes (the removal candidates).
    //   opt        : stop criteria (see Options).
    // Returns the simplified closed convex polytope as a Mesh. The result always
    // contains the original carved polytope (conservative); its volume is >= the
    // original volume.
    static Mesh simplify(const Eigen::AlignedBox3d&          box,
                         const std::vector<math::HalfSpace>& halfSpaces,
                         const Options&                      opt);

    // Volume of a single closed (convex) polytope mesh via the divergence
    // theorem:  V = (1/6) * |sum over faces of a . (b x c)|.
    // Returns 0 for an empty / degenerate mesh.
    static double volume(const Mesh& m);
};

} // namespace expander
