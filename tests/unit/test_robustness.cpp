// Robustness tests: malformed / pathological input to BoxExpander and the
// ClippingEngine half-space guard. These should fail safe (empty result or a
// clear exception), never crash or hang.

#include <gtest/gtest.h>
#include "expander/BoxExpander.hpp"
#include "expander/ClippingEngine.hpp"
#include "expander/MathUtils.hpp"

#include <cmath>
#include <vector>

using namespace expander;

// ── BoxExpander malformed input ───────────────────────────────────────────────

TEST(Robustness, EmptyMeshReturnsEmpty) {
    Mesh empty;
    EXPECT_TRUE(BoxExpander().expand(empty, 1.0).empty());
}

TEST(Robustness, MeshWithVerticesButNoFacesReturnsEmpty) {
    Mesh m;
    m.vertices.resize(3, 3);
    m.vertices << 0, 0, 0, 1, 0, 0, 0, 1, 0;
    // no faces
    EXPECT_TRUE(BoxExpander().expand(m, 1.0).empty());
}

TEST(Robustness, AllDegenerateFacesFallBackToExpandedBox) {
    // A mesh whose only triangles are zero-area (collinear). faceNormals skips
    // them, so the carving has no half-spaces and must still return a valid,
    // conservative expanded box (not crash, not empty).
    Mesh m;
    m.vertices.resize(3, 3);
    m.vertices << 0, 0, 0,
                  1, 0, 0,
                  2, 0, 0;          // all collinear → zero-area triangle
    m.faces = {{0, 1, 2}};
    Mesh out = BoxExpander().expand(m, 1.0);
    ASSERT_FALSE(out.empty());
    ASSERT_GT(out.numFaces(), 0);
    // every input vertex must lie inside the conservative result
    for (int i = 0; i < m.numVertices(); ++i) {
        Eigen::Vector3d v = m.vertices.row(i).transpose();
        bool inside = true;
        for (const auto& f : out.faces) {
            Eigen::Vector3d a = out.vertices.row(f[0]), b = out.vertices.row(f[1]),
                            c = out.vertices.row(f[2]);
            Eigen::Vector3d n = (b - a).cross(c - a);
            double len = n.norm();
            if (len < 1e-300) continue;
            if ((v - a).dot(n) > 1e-6 * len) { inside = false; break; }
        }
        EXPECT_TRUE(inside) << "vertex " << i << " outside conservative result";
    }
}

TEST(Robustness, NonFiniteVerticesDoNotCrash) {
    // NaN/Inf input must not crash; a degenerate result (empty) is acceptable.
    Mesh m;
    m.vertices.resize(4, 3);
    m.vertices << 0, 0, 0,
                  1, 0, 0,
                  0, 1, 0,
                  0, 0, std::numeric_limits<double>::quiet_NaN();
    m.faces = {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
    EXPECT_NO_THROW({ Mesh out = BoxExpander().expand(m, 1.0); (void)out; });
}

// ── ClippingEngine half-space guard ──────────────────────────────────────────

TEST(Robustness, ClippingEngineRejectsTooManyHalfSpaces) {
    // Far more half-spaces than the guard allows → throws instead of an
    // O(n^3) blow-up. Use distinct directions on a sphere.
    Eigen::AlignedBox3d box(Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(1, 1, 1));
    std::vector<math::HalfSpace> hs;
    const int N = 400;  // > kMaxHalfSpaces (256)
    for (int i = 0; i < N; ++i) {
        double a = 2.0 * M_PI * i / N;
        double z = -1.0 + 2.0 * i / N;
        double r = std::sqrt(std::max(0.0, 1.0 - z * z));
        Eigen::Vector3d n(r * std::cos(a), r * std::sin(a), z);
        hs.push_back({n.normalized(), 1.0});
    }
    EXPECT_THROW(ClippingEngine::clip(box, hs), std::runtime_error);
}

TEST(Robustness, ClippingEngineAcceptsModerateHalfSpaces) {
    // Well within the guard — must succeed.
    Eigen::AlignedBox3d box(Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(1, 1, 1));
    std::vector<math::HalfSpace> hs;
    for (int i = 0; i < 20; ++i) {
        double a = 2.0 * M_PI * i / 20;
        hs.push_back({Eigen::Vector3d(std::cos(a), std::sin(a), 0.0), 0.9});
    }
    EXPECT_NO_THROW({ Mesh m = ClippingEngine::clip(box, hs); (void)m; });
}

// ── Anisotropic per-axis d ────────────────────────────────────────────────────

static Mesh makeAxisCube(double h) {
    Mesh m;
    m.vertices.resize(8, 3);
    m.vertices << -h,-h,-h,  h,-h,-h,  h,h,-h, -h,h,-h,
                  -h,-h, h,  h,-h, h,  h,h, h, -h,h, h;
    m.faces = {{0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,1,5},{0,5,4},
               {2,3,7},{2,7,6},{1,2,6},{1,6,5},{0,4,7},{0,7,3}};
    return m;
}

TEST(Anisotropic, IsotropicEquivalence) {
    Mesh cube = makeAxisCube(1.0);
    Mesh a = BoxExpander().expand(cube, 0.5);
    Mesh b = BoxExpander().expand(cube, Eigen::Vector3d(0.5, 0.5, 0.5));
    ASSERT_EQ(a.numVertices(), b.numVertices());
    Eigen::Vector3d amin = a.vertices.colwise().minCoeff().transpose();
    Eigen::Vector3d bmin = b.vertices.colwise().minCoeff().transpose();
    EXPECT_LT((amin - bmin).norm(), 1e-9);
}

TEST(Anisotropic, AxisAlignedExactness) {
    // Axis-aligned cube expanded by (2.0, 0.5, 0.25) → result AABB grows by
    // exactly those per-axis amounts on each side.
    Mesh cube = makeAxisCube(1.0);
    Eigen::Vector3d d(2.0, 0.5, 0.25);
    Mesh out = BoxExpander().expand(cube, d);
    Eigen::Vector3d mn = out.vertices.colwise().minCoeff().transpose();
    Eigen::Vector3d mx = out.vertices.colwise().maxCoeff().transpose();
    EXPECT_NEAR(mx.x(), 1.0 + 2.0,  1e-6);
    EXPECT_NEAR(mx.y(), 1.0 + 0.5,  1e-6);
    EXPECT_NEAR(mx.z(), 1.0 + 0.25, 1e-6);
    EXPECT_NEAR(mn.x(), -(1.0 + 2.0),  1e-6);
    EXPECT_NEAR(mn.y(), -(1.0 + 0.5),  1e-6);
    EXPECT_NEAR(mn.z(), -(1.0 + 0.25), 1e-6);
}

TEST(Anisotropic, ConservativePerAxis) {
    Mesh cube = makeAxisCube(2.0);
    Eigen::Vector3d d(0.3, 1.5, 0.8);
    Mesh out = BoxExpander().expand(cube, d);
    ASSERT_GT(out.numFaces(), 0);
    for (int i = 0; i < cube.numVertices(); ++i) {
        Eigen::Vector3d v = cube.vertices.row(i).transpose();
        bool inside = true;
        for (const auto& f : out.faces) {
            Eigen::Vector3d a = out.vertices.row(f[0]), b = out.vertices.row(f[1]),
                            c = out.vertices.row(f[2]);
            Eigen::Vector3d nn = (b - a).cross(c - a);
            double len = nn.norm();
            if (len < 1e-300) continue;
            if ((v - a).dot(nn) > 1e-6 * len) { inside = false; break; }
        }
        EXPECT_TRUE(inside) << "vertex " << i << " outside anisotropic result";
    }
}
