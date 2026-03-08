#include <gtest/gtest.h>
#include "expander/ConservativeExpander.hpp"
#include "expander/MathUtils.hpp"

using namespace expander;
using namespace expander::math;

// ---------------------------------------------------------------------------
// Mesh factories
// ---------------------------------------------------------------------------
static Mesh makeCube(double half = 1.0) {
    Mesh m;
    m.vertices.resize(8, 3);
    // clang-format off
    m.vertices << -half, -half, -half,
                   half, -half, -half,
                   half,  half, -half,
                  -half,  half, -half,
                  -half, -half,  half,
                   half, -half,  half,
                   half,  half,  half,
                  -half,  half,  half;
    // clang-format on
    return m;
}

static Mesh makeThinPlate(double hw = 1.0, double thickness = 1e-4) {
    Mesh m;
    m.vertices.resize(8, 3);
    // clang-format off
    m.vertices << -hw, -hw, -thickness,
                   hw, -hw, -thickness,
                   hw,  hw, -thickness,
                  -hw,  hw, -thickness,
                  -hw, -hw,  thickness,
                   hw, -hw,  thickness,
                   hw,  hw,  thickness,
                  -hw,  hw,  thickness;
    // clang-format on
    return m;
}

static Mesh makeTetrahedron() {
    // Vertices of a regular tetrahedron
    Mesh m;
    m.vertices.resize(4, 3);
    // clang-format off
    m.vertices <<  1.0,  1.0,  1.0,
                  -1.0, -1.0,  1.0,
                  -1.0,  1.0, -1.0,
                   1.0, -1.0, -1.0;
    // clang-format on
    return m;
}

// ---------------------------------------------------------------------------
// Helper: compute half-spaces in WORLD space from original vertices + d.
// Used by tests to verify conservativeness without access to private state.
// ---------------------------------------------------------------------------
static std::vector<HalfSpace> worldHalfSpaces(const Mesh& input, double d) {
    auto dirs   = generate26Directions();
    auto merged = mergeDirections(dirs);
    std::vector<HalfSpace> hs;
    hs.reserve(merged.size());
    for (const auto& n : merged) {
        double maxDot = (input.vertices * n).maxCoeff();
        hs.push_back({n, maxDot + d});
    }
    return hs;
}

// ---------------------------------------------------------------------------
// Test 1 (仕様 §4 Test1): 保守性の検証
//   全出力頂点が、元メッシュの頂点から導出した全半空間を満たすこと。
//   かつ、全入力頂点もその半空間に包含されていること（自明だが定義確認）。
// ---------------------------------------------------------------------------
TEST(ConservativeExpander, OutputSatisfiesAllHalfSpaces) {
    const double d = 0.15;
    Mesh cube = makeCube(1.0);
    ConservativeExpander expander;
    Mesh result = expander.expand(cube, d);

    ASSERT_GT(result.numVertices(), 0) << "Expansion returned empty mesh";

    auto hs = worldHalfSpaces(cube, d);

    for (int i = 0; i < result.numVertices(); ++i) {
        Eigen::Vector3d v = result.vertices.row(i).transpose();
        for (const auto& h : hs) {
            EXPECT_LE(h.normal.dot(v), h.distance + 1e-5)
                << "Output vertex " << i << " violates half-space with normal "
                << h.normal.transpose();
        }
    }
}

TEST(ConservativeExpander, AllInputVerticesInsideExpandedHalfSpaces) {
    const double d = 0.1;
    Mesh tet = makeTetrahedron();
    auto hs  = worldHalfSpaces(tet, d);

    // Every input vertex must satisfy every half-space (conservativeness axiom)
    for (int i = 0; i < tet.numVertices(); ++i) {
        Eigen::Vector3d v = tet.vertices.row(i).transpose();
        for (const auto& h : hs) {
            EXPECT_LE(h.normal.dot(v), h.distance + 1e-9)
                << "Input vertex " << i << " violates its own half-space "
                << "(construction invariant broken)";
        }
    }
}

// ---------------------------------------------------------------------------
// Test 2 (仕様 §4 Test2): 堅牢性の検証
//   極端に薄い板を入力しても NaN・Inf が生じないこと。
// ---------------------------------------------------------------------------
TEST(ConservativeExpander, ThinPlateNoNaN) {
    Mesh plate = makeThinPlate(1.0, 1e-6);
    ConservativeExpander expander;
    Mesh result = expander.expand(plate, 0.1);

    ASSERT_GT(result.numVertices(), 0);
    for (int i = 0; i < result.numVertices(); ++i) {
        EXPECT_TRUE(result.vertices.row(i).allFinite())
            << "NaN/Inf detected at output vertex " << i;
    }
}

TEST(ConservativeExpander, ThinPlateVerticesContainedInExpansion) {
    const double d = 0.05;
    Mesh plate    = makeThinPlate(1.0, 1e-5);
    ConservativeExpander expander;
    Mesh result   = expander.expand(plate, d);

    auto hs = worldHalfSpaces(plate, d);

    for (int i = 0; i < result.numVertices(); ++i) {
        Eigen::Vector3d v = result.vertices.row(i).transpose();
        for (const auto& h : hs) {
            EXPECT_LE(h.normal.dot(v), h.distance + 1e-5)
                << "Thin plate: output vertex " << i << " violates half-space";
        }
    }
}

// ---------------------------------------------------------------------------
// Test 3 (仕様 §4 Test3): 出力頂点数が入力サイズに依存せず軽量であること
//   小さな cube と大きな cube で出力頂点数が同一になること。
//   また頂点数の上界は C(|directions|, 3) 以下であること。
// ---------------------------------------------------------------------------
TEST(ConservativeExpander, VertexCountInputSizeIndependent) {
    ConservativeExpander expander;
    Mesh small  = makeCube(0.001);
    Mesh large  = makeCube(1000.0);

    Mesh r1 = expander.expand(small, 0.01);
    Mesh r2 = expander.expand(large, 10.0);

    EXPECT_EQ(r1.numVertices(), r2.numVertices())
        << "Output vertex count must not depend on input scale";
}

TEST(ConservativeExpander, VertexCountBoundedByDirections) {
    ConservativeExpander expander;
    Mesh cube = makeCube(1.0);
    Mesh result = expander.expand(cube, 0.1);

    // Maximum possible vertices = C(k, 3) where k = number of half-spaces
    auto dirs   = generate26Directions();
    auto merged = mergeDirections(dirs);
    const int k = static_cast<int>(merged.size());
    // C(k,3) = k*(k-1)*(k-2)/6
    const int maxVerts = k * (k - 1) * (k - 2) / 6;

    EXPECT_LE(result.numVertices(), maxVerts)
        << "Output vertex count " << result.numVertices()
        << " exceeds theoretical maximum C(" << k << ",3)=" << maxVerts;
}

// ---------------------------------------------------------------------------
// Additional: 膨張距離が AABB に反映されていること
// ---------------------------------------------------------------------------
TEST(ConservativeExpander, ExpansionReachesAtLeastD) {
    const double half = 1.0;
    const double d    = 0.5;
    Mesh cube = makeCube(half);
    ConservativeExpander expander;
    Mesh result = expander.expand(cube, d);

    Eigen::Vector3d resMin = result.vertices.colwise().minCoeff();
    Eigen::Vector3d resMax = result.vertices.colwise().maxCoeff();

    // Axis-aligned directions must produce faces at least half+d away
    for (int ax = 0; ax < 3; ++ax) {
        EXPECT_GE(resMax(ax), half + d - 1e-5)
            << "Positive axis " << ax << ": expanded extent too small";
        EXPECT_LE(resMin(ax), -(half + d) + 1e-5)
            << "Negative axis " << ax << ": expanded extent too small";
    }
}

// ---------------------------------------------------------------------------
// Edge case: empty input
// ---------------------------------------------------------------------------
TEST(ConservativeExpander, EmptyInputReturnsEmpty) {
    Mesh empty;
    empty.vertices.resize(0, 3);
    ConservativeExpander expander;
    Mesh result = expander.expand(empty, 0.1);
    EXPECT_TRUE(result.empty());
}
