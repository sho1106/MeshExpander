#include "expander/ProgressiveHull.hpp"

#include <cmath>
#include <limits>

namespace expander {

// ---------------------------------------------------------------------------
// volume() — divergence theorem on a single closed polytope
//   V = (1/6) * |sum_faces a . (b x c)|
// Valid only for a single closed mesh (the polytope ClippingEngine::clip emits).
// ---------------------------------------------------------------------------
double ProgressiveHull::volume(const Mesh& m)
{
    if (m.empty() || m.faces.empty()) return 0.0;

    double six_v = 0.0;
    for (const auto& f : m.faces) {
        const Eigen::Vector3d a = m.vertices.row(f[0]).transpose();
        const Eigen::Vector3d b = m.vertices.row(f[1]).transpose();
        const Eigen::Vector3d c = m.vertices.row(f[2]).transpose();
        six_v += a.dot(b.cross(c));
    }
    return std::abs(six_v) / 6.0;
}

// ---------------------------------------------------------------------------
// simplify() — greedy volume-guided half-space removal
// ---------------------------------------------------------------------------
Mesh ProgressiveHull::simplify(const Eigen::AlignedBox3d&          box,
                               const std::vector<math::HalfSpace>& halfSpaces,
                               const Options&                      opt)
{
    // Active set of candidate half-spaces (box faces are added by clip()).
    std::vector<math::HalfSpace> active = halfSpaces;

    // Baseline: original carved polytope and its volume.
    Mesh current = ClippingEngine::clip(box, active);
    if (current.empty()) return current;          // degenerate input, nothing to do

    const double v0 = volume(current);
    if (!(v0 > 0.0)) return current;              // non-finite / zero baseline

    // Volume ceiling: stop before the simplified volume would exceed this.
    const double volBudget = (opt.maxVolumeGrowth > 0.0)
        ? v0 * (1.0 + opt.maxVolumeGrowth)
        : std::numeric_limits<double>::infinity();

    // Greedy loop: each iteration removes at most one half-space.
    while (!active.empty()) {
        // Face-count stop criterion.
        if (opt.maxFaces > 0 && current.numFaces() <= opt.maxFaces)
            break;

        // Find the removal that grows the volume the least, among removals that
        // keep the polytope valid (>= 4 vertices, finite volume) and within budget.
        int    bestIdx  = -1;
        double bestVol  = std::numeric_limits<double>::infinity();
        Mesh   bestMesh;

        for (int i = 0; i < static_cast<int>(active.size()); ++i) {
            // Build the candidate set with half-space i removed.
            std::vector<math::HalfSpace> trial;
            trial.reserve(active.size() - 1);
            for (int j = 0; j < static_cast<int>(active.size()); ++j)
                if (j != i) trial.push_back(active[j]);

            Mesh cand = ClippingEngine::clip(box, trial);
            if (cand.empty() || cand.numVertices() < 4) continue;  // unbounded / degenerate

            const double vol = volume(cand);
            if (!std::isfinite(vol) || vol < v0) continue;         // guard non-finite / shrink

            if (vol < bestVol) {
                bestVol  = vol;
                bestIdx  = i;
                bestMesh = std::move(cand);
            }
        }

        // No valid removal, or the cheapest removal exceeds the volume budget → stop.
        if (bestIdx < 0 || bestVol > volBudget)
            break;

        // Commit the best removal.
        active.erase(active.begin() + bestIdx);
        current = std::move(bestMesh);
    }

    return current;
}

} // namespace expander
