#include <gtest/gtest.h>
#include "expander/ProgressiveHull.hpp"
#include "expander/ClippingEngine.hpp"
#include "expander/BoxExpander.hpp"
#include "expander/MathUtils.hpp"

#include <cmath>
#include <vector>

using namespace expander;
using namespace expander::math;

// ---------------------------------------------------------------------------
// Build a dense set of half-spaces tangent to a sphere of radius r centred at
// the origin: { n . x <= r } for many unit directions n. The intersection is a
// fine polyhedral approximation of the ball — a good "carved convex polytope"
// stand-in with many removable faces. A Fibonacci sphere gives near-uniform n.
// ---------------------------------------------------------------------------
static std::vector<HalfSpace> sphereHalfSpaces(int count, double r) {
    std::vector<HalfSpace> hs;
    hs.reserve(count);
    const double golden = M_PI * (3.0 - std::sqrt(5.0));  // golden angle
    for (int i = 0; i < count; ++i) {
        const double z   = 1.0 - 2.0 * (i + 0.5) / count;  // (-1, 1)
        const double rho = std::sqrt(std::max(0.0, 1.0 - z * z));
        const double th  = golden * i;
        Eigen::Vector3d n(rho * std::cos(th), rho * std::sin(th), z);
        hs.push_back({n.normalized(), r});
    }
    return hs;
}

// Box large enough that its 6 faces never clip the sphere (radius r).
static Eigen::AlignedBox3d enclosingBox(double r) {
    const double h = r * 2.0;
    return Eigen::AlignedBox3d(Eigen::Vector3d::Constant(-h),
                               Eigen::Vector3d::Constant( h));
}

// ---------------------------------------------------------------------------
// Core test: carve a many-direction polytope, simplify to a face-count target,
// and assert conservativeness + monotone volume + reduced face count.
// ---------------------------------------------------------------------------
TEST(ProgressiveHull, ConservativeSimplificationToFaceTarget) {
    const double r = 1.5;
    const auto   box = enclosingBox(r);
    // Small input keeps this fast (the greedy re-clips per candidate, O(n^3));
    // the large-scale 228->48 reduction is exercised by the docs benchmark.
    const auto   hs  = sphereHalfSpaces(24, r);

    // Original carved polytope and its volume.
    Mesh original = ClippingEngine::clip(box, hs);
    ASSERT_FALSE(original.empty());
    const double v0 = ProgressiveHull::volume(original);
    ASSERT_GT(v0, 0.0);

    ProgressiveHull::Options opt;
    opt.maxFaces        = 20;    // aim well below the original face count
    opt.maxVolumeGrowth = 0.30;  // allow up to +30% volume

    Mesh simplified = ProgressiveHull::simplify(box, hs, opt);
    ASSERT_FALSE(simplified.empty());
    const double v1 = ProgressiveHull::volume(simplified);

    // (3) Face count strictly reduced.
    EXPECT_LT(simplified.numFaces(), original.numFaces());

    // (2) Volume monotone / conservative: simplified encloses original.
    EXPECT_GE(v1, v0 - 1e-9);
    // And it respected the growth budget.
    EXPECT_LE(v1, v0 * (1.0 + opt.maxVolumeGrowth) + 1e-9);

    // (1) Conservative: every ORIGINAL polytope vertex is still inside the
    // simplified polytope. We test against the simplified mesh's own face
    // planes (point-in-convex), reconstructed from its triangles.
    // Build outward half-spaces from the simplified mesh faces.
    std::vector<HalfSpace> simpHs;
    simpHs.reserve(simplified.faces.size());
    for (const auto& f : simplified.faces) {
        const Eigen::Vector3d a = simplified.vertices.row(f[0]).transpose();
        const Eigen::Vector3d b = simplified.vertices.row(f[1]).transpose();
        const Eigen::Vector3d c = simplified.vertices.row(f[2]).transpose();
        Eigen::Vector3d n = (b - a).cross(c - a);
        if (n.norm() < kEpsilon) continue;
        n.normalize();
        simpHs.push_back({n, n.dot(a)});
    }

    int outside = 0;
    for (int i = 0; i < original.numVertices(); ++i) {
        const Eigen::Vector3d v = original.vertices.row(i).transpose();
        // Generous tolerance: face planes from triangulation + safety margins.
        if (!isInsideAll(v, simpHs, 1e-3 * r)) ++outside;
    }
    EXPECT_EQ(outside, 0)
        << outside << " original vertices fell outside the simplified polytope";

    // Console summary (visible with --gtest_filter under ctest -V).
    std::printf("[ProgressiveHull] faces %d -> %d, volume %.6f -> %.6f (+%.2f%%)\n",
                original.numFaces(), simplified.numFaces(),
                v0, v1, 100.0 * (v1 - v0) / v0);
}

// Volume-budget criterion alone (no face target): stops before exceeding budget.
TEST(ProgressiveHull, VolumeBudgetIsRespected) {
    const double r = 1.0;
    const auto   box = enclosingBox(r);
    const auto   hs  = sphereHalfSpaces(20, r);

    Mesh original = ClippingEngine::clip(box, hs);
    ASSERT_FALSE(original.empty());
    const double v0 = ProgressiveHull::volume(original);

    ProgressiveHull::Options opt;
    opt.maxFaces        = 0;     // disable face-count criterion
    opt.maxVolumeGrowth = 0.05;  // tight 5% budget

    Mesh simplified = ProgressiveHull::simplify(box, hs, opt);
    ASSERT_FALSE(simplified.empty());
    const double v1 = ProgressiveHull::volume(simplified);

    EXPECT_GE(v1, v0 - 1e-9);
    EXPECT_LE(v1, v0 * 1.05 + 1e-9);
    EXPECT_LE(simplified.numFaces(), original.numFaces());
}

// A coarse tessellated sphere mesh (input geometry for the convenience API).
static Mesh uvSphere(double R, int nlat, int nlon) {
    Mesh m;
    std::vector<Eigen::Vector3d> V = {{0, 0, R}, {0, 0, -R}};
    auto idx = [&](int i, int j) { return 2 + (i - 1) * nlon + (j % nlon); };
    for (int i = 1; i < nlat; ++i) {
        const double th = M_PI * i / nlat;
        for (int j = 0; j < nlon; ++j) {
            const double ph = 2 * M_PI * j / nlon;
            V.push_back({R * std::sin(th) * std::cos(ph),
                         R * std::sin(th) * std::sin(ph), R * std::cos(th)});
        }
    }
    m.vertices.resize((int)V.size(), 3);
    for (int i = 0; i < (int)V.size(); ++i) m.vertices.row(i) = V[i].transpose();
    for (int j = 0; j < nlon; ++j) {
        m.faces.push_back({0, idx(1, j), idx(1, j + 1)});
        m.faces.push_back({1, idx(nlat - 1, j + 1), idx(nlat - 1, j)});
    }
    for (int i = 1; i < nlat - 1; ++i)
        for (int j = 0; j < nlon; ++j) {
            m.faces.push_back({idx(i, j), idx(i, j + 1), idx(i + 1, j + 1)});
            m.faces.push_back({idx(i, j), idx(i + 1, j + 1), idx(i + 1, j)});
        }
    return m;
}

// BoxExpander::expandSimplified is callable end-to-end: fewer faces than a plain
// expand, and still conservative (every input vertex inside the result).
TEST(ProgressiveHull, BoxExpanderConvenienceIsConservativeAndSmaller) {
    Mesh sphere = uvSphere(5.0, 5, 8);    // coarse: keeps the greedy loop quick
    BoxExpander exp;                       // 20-degree merge

    Mesh full = exp.expand(sphere, 1.0);
    ASSERT_FALSE(full.empty());

    ProgressiveHull::Options opt;
    opt.maxFaces        = 20;
    opt.maxVolumeGrowth = 0.5;
    Mesh simp = exp.expandSimplified(sphere, 1.0, opt);
    ASSERT_FALSE(simp.empty());

    EXPECT_LE(simp.numFaces(), full.numFaces());

    std::vector<HalfSpace> simpHs;
    for (const auto& f : simp.faces) {
        const Eigen::Vector3d a = simp.vertices.row(f[0]).transpose();
        const Eigen::Vector3d b = simp.vertices.row(f[1]).transpose();
        const Eigen::Vector3d c = simp.vertices.row(f[2]).transpose();
        Eigen::Vector3d n = (b - a).cross(c - a);
        if (n.norm() < kEpsilon) continue;
        n.normalize();
        simpHs.push_back({n, n.dot(a)});
    }
    int outside = 0;
    for (int i = 0; i < sphere.numVertices(); ++i)
        if (!isInsideAll(sphere.vertices.row(i).transpose(), simpHs, 1e-2)) ++outside;
    EXPECT_EQ(outside, 0) << outside << " input vertices outside the simplified expansion";
}

// volume() matches a known closed cube exactly.
TEST(ProgressiveHull, VolumeOfUnitCube) {
    Eigen::AlignedBox3d cube(Eigen::Vector3d::Constant(-1.0),
                             Eigen::Vector3d::Constant( 1.0));
    Mesh m = ClippingEngine::clip(cube, {});   // 2x2x2 cube
    EXPECT_NEAR(ProgressiveHull::volume(m), 8.0, 1e-6);
}
