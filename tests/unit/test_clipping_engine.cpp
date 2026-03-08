#include <gtest/gtest.h>
#include "expander/ClippingEngine.hpp"
#include "expander/MathUtils.hpp"

using namespace expander;
using namespace expander::math;

// Helper: unit cube box [-1,1]^3
static Eigen::AlignedBox3d unitBox() {
    return Eigen::AlignedBox3d(
        Eigen::Vector3d::Constant(-1.0),
        Eigen::Vector3d::Constant( 1.0));
}

// Helper: box with no additional cutting planes → should return a box mesh
TEST(ClippingEngine, BoxWithNoHalfSpaces) {
    Mesh m = ClippingEngine::clip(unitBox(), {});
    // A box has 8 vertices, 6 faces × 2 triangles = 12 triangles
    EXPECT_EQ(m.vertices.rows(), 8);
    EXPECT_EQ(static_cast<int>(m.faces.size()), 12);
}

// A single clipping plane that cuts x ≤ 0 → should produce exactly 5 faces
// (3 original box faces on the +x side are untouched; the cut produces 1 new face;
// plus the 3 faces of the -x half lose their vertices → net result: cube cut by plane)
TEST(ClippingEngine, SingleCutProducesValidMesh) {
    // Cut: x ≤ 0  (keeps only x ∈ [-1, 0] half of the unit cube)
    std::vector<HalfSpace> hs;
    hs.push_back({Eigen::Vector3d(1, 0, 0), 0.0}); // x ≤ 0

    Mesh m = ClippingEngine::clip(unitBox(), hs);
    EXPECT_GE(m.vertices.rows(), 4);
    EXPECT_FALSE(m.faces.empty());

    // All output vertices must satisfy x ≤ 0 + safety margin
    for (int i = 0; i < m.vertices.rows(); ++i)
        EXPECT_LE(m.vertices(i, 0), kSafetyMargin + kOnPlaneEps)
            << "Vertex " << i << " violates the clipping half-space";
}

// Clipping plane that does NOT intersect the box → output is unchanged box
TEST(ClippingEngine, NonIntersectingCutLeavesBoxIntact) {
    // x ≤ 5.0 doesn't cut the unit cube at all
    std::vector<HalfSpace> hs;
    hs.push_back({Eigen::Vector3d(1, 0, 0), 5.0});

    Mesh m = ClippingEngine::clip(unitBox(), hs);
    EXPECT_EQ(m.vertices.rows(), 8);
    EXPECT_EQ(static_cast<int>(m.faces.size()), 12);
}

// Clipping plane that entirely eliminates the box → degenerate, empty result
TEST(ClippingEngine, FullyEliminatingCutReturnsEmpty) {
    // x ≤ -5.0 eliminates all of the unit cube [-1,1]^3
    std::vector<HalfSpace> hs;
    hs.push_back({Eigen::Vector3d(1, 0, 0), -5.0});

    Mesh m = ClippingEngine::clip(unitBox(), hs);
    EXPECT_TRUE(m.empty());
}

// Safety margin: a plane at exactly D should be padded outward,
// so vertices near D are still inside the clipped region.
TEST(ClippingEngine, SafetyMarginPadsOutward) {
    // Clip at x = 0.5 exactly.  After adding kSafetyMargin, the boundary
    // is at 0.5 + kSafetyMargin, so a vertex originally at x = 0.5
    // must be accepted as inside (not cut away).
    std::vector<HalfSpace> hs;
    hs.push_back({Eigen::Vector3d(1, 0, 0), 0.5});

    Mesh m = ClippingEngine::clip(unitBox(), hs);
    EXPECT_FALSE(m.faces.empty());

    // The vertex at x = 0.5 should appear in the output (it's on the new face)
    bool found = false;
    for (int i = 0; i < m.vertices.rows(); ++i) {
        if (std::abs(m.vertices(i, 0) - 0.5) < kOnPlaneEps * 10)
            found = true;
    }
    EXPECT_TRUE(found) << "Vertex at x=0.5 not found in output — safety margin may have cut it";
}

// All output vertices of a clipped box must satisfy all half-spaces
TEST(ClippingEngine, AllOutputVerticesSatisfyHalfSpaces) {
    // Three diagonal cuts
    std::vector<HalfSpace> hs = {
        {Eigen::Vector3d( 1, 1, 0).normalized(), 0.8},
        {Eigen::Vector3d( 1, 0, 1).normalized(), 0.8},
        {Eigen::Vector3d( 0, 1, 1).normalized(), 0.8},
    };
    Mesh m = ClippingEngine::clip(unitBox(), hs);
    if (m.empty()) return; // allowed if fully clipped

    // For every output vertex, it must satisfy every user half-space
    // (with safety margin already included, so use distance + margin)
    for (int i = 0; i < m.vertices.rows(); ++i) {
        Eigen::Vector3d v = m.vertices.row(i).transpose();
        for (const auto& h : hs) {
            double dot = h.normal.dot(v);
            EXPECT_LE(dot, h.distance + kSafetyMargin + kOnPlaneEps)
                << "Vertex " << i << " violates half-space (n=" << h.normal.transpose()
                << ", D=" << h.distance << ")";
        }
    }
}

// Mesh must have no degenerate (zero-area) triangles
TEST(ClippingEngine, NoZeroAreaFaces) {
    std::vector<HalfSpace> hs = {
        {Eigen::Vector3d(1, 1, 0).normalized(), 0.5},
    };
    Mesh m = ClippingEngine::clip(unitBox(), hs);

    for (const auto& f : m.faces) {
        Eigen::Vector3d v0 = m.vertices.row(f[0]).transpose();
        Eigen::Vector3d v1 = m.vertices.row(f[1]).transpose();
        Eigen::Vector3d v2 = m.vertices.row(f[2]).transpose();
        double area = (v1 - v0).cross(v2 - v0).norm() * 0.5;
        EXPECT_GT(area, 1e-10) << "Degenerate triangle: face " << f.transpose();
    }
}
