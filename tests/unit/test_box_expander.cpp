#include <gtest/gtest.h>
#include "expander/BoxExpander.hpp"
#include "expander/MathUtils.hpp"

using namespace expander;
using namespace expander::math;

// ---------------------------------------------------------------------------
// Mesh factories (with face data — required for face-normal mode)
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
    m.faces = {
        {0,2,1},{0,3,2}, {4,5,6},{4,6,7},
        {0,1,5},{0,5,4}, {2,3,7},{2,7,6},
        {1,2,6},{1,6,5}, {0,4,7},{0,7,3},
    };
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
    m.faces = {
        {0,2,1},{0,3,2}, {4,5,6},{4,6,7},
        {0,1,5},{0,5,4}, {2,3,7},{2,7,6},
        {1,2,6},{1,6,5}, {0,4,7},{0,7,3},
    };
    return m;
}

static Mesh makeTetrahedron() {
    Mesh m;
    m.vertices.resize(4, 3);
    // clang-format off
    m.vertices <<  1.0,  1.0,  1.0,
                  -1.0, -1.0,  1.0,
                  -1.0,  1.0, -1.0,
                   1.0, -1.0, -1.0;
    // clang-format on
    m.faces = {{0,1,2},{0,2,3},{0,3,1},{1,3,2}};
    return m;
}

// ---------------------------------------------------------------------------
// Helper: check that all input vertices are inside the expanded output AABB.
// This is the fundamental conservativeness guarantee of BoxExpander.
// ---------------------------------------------------------------------------
static bool allVerticesCovered(const Mesh& input, const Mesh& expanded, double eps = 1e-5) {
    if (expanded.empty()) return false;
    const Eigen::Vector3d lo = expanded.vertices.colwise().minCoeff().transpose();
    const Eigen::Vector3d hi = expanded.vertices.colwise().maxCoeff().transpose();
    for (int i = 0; i < input.numVertices(); ++i) {
        Eigen::Vector3d v = input.vertices.row(i).transpose();
        if ((v.array() < lo.array() - eps).any() ||
            (v.array() > hi.array() + eps).any())
            return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Test 1: 保守性 — 全入力頂点が出力AABB内に収まること
// ---------------------------------------------------------------------------
TEST(BoxExpander, AllInputVerticesCovered_Cube) {
    const double d = 0.15;
    Mesh cube = makeCube(1.0);
    BoxExpander exp;
    Mesh result = exp.expand(cube, d);

    ASSERT_GT(result.numVertices(), 0) << "Expansion returned empty mesh";
    EXPECT_TRUE(allVerticesCovered(cube, result))
        << "Some cube vertices are outside the expanded AABB";
}

TEST(BoxExpander, AllInputVerticesCovered_Tetrahedron) {
    const double d = 0.1;
    Mesh tet = makeTetrahedron();
    BoxExpander exp;
    Mesh result = exp.expand(tet, d);

    ASSERT_GT(result.numVertices(), 0);
    EXPECT_TRUE(allVerticesCovered(tet, result))
        << "Some tetrahedron vertices are outside the expanded AABB";
}

// ---------------------------------------------------------------------------
// Test 2: 堅牢性 — 極端に薄い板でも NaN・Inf が生じないこと
// ---------------------------------------------------------------------------
TEST(BoxExpander, ThinPlateNoNaN) {
    Mesh plate = makeThinPlate(1.0, 1e-6);
    BoxExpander exp;
    Mesh result = exp.expand(plate, 0.1);

    ASSERT_GT(result.numVertices(), 0);
    for (int i = 0; i < result.numVertices(); ++i) {
        EXPECT_TRUE(result.vertices.row(i).allFinite())
            << "NaN/Inf detected at output vertex " << i;
    }
}

TEST(BoxExpander, ThinPlateVerticesCovered) {
    const double d = 0.05;
    Mesh plate = makeThinPlate(1.0, 1e-5);
    BoxExpander exp;
    Mesh result = exp.expand(plate, d);

    ASSERT_GT(result.numVertices(), 0);
    EXPECT_TRUE(allVerticesCovered(plate, result))
        << "Thin plate: some vertices outside expanded AABB";
}

// ---------------------------------------------------------------------------
// Test 3: スケール独立性 — 入力スケールに関わらず同数のトポロジー
// (cube の場合、±x/±y/±z の6方向のみ → 常に直方体になる)
// ---------------------------------------------------------------------------
TEST(BoxExpander, VertexCountInputSizeIndependent) {
    BoxExpander exp;
    Mesh small = makeCube(0.001);
    Mesh large = makeCube(1000.0);

    Mesh r1 = exp.expand(small, 0.01);
    Mesh r2 = exp.expand(large, 10.0);

    EXPECT_EQ(r1.numVertices(), r2.numVertices())
        << "Output vertex count must not depend on input scale";
}

// ---------------------------------------------------------------------------
// Test 4: 頂点数の上界 — C(k+6, 3) 以下であること
//   k = 面法線のマージ後の方向数, 6 = boxの面数 (ClippingEngineが追加)
// ---------------------------------------------------------------------------
TEST(BoxExpander, VertexCountBoundedByDirections) {
    Mesh tet = makeTetrahedron();
    BoxExpander exp;
    Mesh result = exp.expand(tet, 0.1);

    // Collect face normals for tetrahedron
    std::vector<Eigen::Vector3d> normals;
    for (const auto& f : tet.faces) {
        Eigen::Vector3d v0 = tet.vertices.row(f[0]).transpose();
        Eigen::Vector3d v1 = tet.vertices.row(f[1]).transpose();
        Eigen::Vector3d v2 = tet.vertices.row(f[2]).transpose();
        Eigen::Vector3d n  = (v1 - v0).cross(v2 - v0);
        double len = n.norm();
        if (len > math::kEpsilon) normals.push_back(n / len);
    }
    const auto merged = mergeDirections(normals, 20.0);
    const int k = static_cast<int>(merged.size()) + 6;  // +6 for box faces
    const long long maxVerts = (long long)k * (k - 1) * (k - 2) / 6;

    EXPECT_LE(result.numVertices(), maxVerts)
        << "Output vertex count " << result.numVertices()
        << " exceeds theoretical maximum C(" << k << ",3)=" << maxVerts;
}

// ---------------------------------------------------------------------------
// Test 5: 膨張距離が AABB に反映されていること
// ---------------------------------------------------------------------------
TEST(BoxExpander, ExpansionReachesAtLeastD) {
    const double half = 1.0;
    const double d    = 0.5;
    Mesh cube = makeCube(half);
    BoxExpander exp;
    Mesh result = exp.expand(cube, d);

    Eigen::Vector3d resMin = result.vertices.colwise().minCoeff();
    Eigen::Vector3d resMax = result.vertices.colwise().maxCoeff();

    for (int ax = 0; ax < 3; ++ax) {
        EXPECT_GE(resMax(ax), half + d - 1e-5)
            << "Positive axis " << ax << ": expanded extent too small";
        EXPECT_LE(resMin(ax), -(half + d) + 1e-5)
            << "Negative axis " << ax << ": expanded extent too small";
    }
}

// ---------------------------------------------------------------------------
// Test 6: 空入力 → 空出力
// ---------------------------------------------------------------------------
TEST(BoxExpander, EmptyInputReturnsEmpty) {
    Mesh empty;
    empty.vertices.resize(0, 3);
    BoxExpander exp;
    Mesh result = exp.expand(empty, 0.1);
    EXPECT_TRUE(result.empty());
}

// ---------------------------------------------------------------------------
// Test 7: 面なし入力 → 空出力（面なしは処理不能）
// ---------------------------------------------------------------------------
TEST(BoxExpander, NoFacesReturnsEmpty) {
    // Vertices only (no faces) — BoxExpander requires face data for normals
    Mesh m;
    m.vertices.resize(8, 3);
    m.vertices << -1, -1, -1,  1, -1, -1,  1,  1, -1, -1,  1, -1,
                  -1, -1,  1,  1, -1,  1,  1,  1,  1, -1,  1,  1;
    BoxExpander exp;
    Mesh result = exp.expand(m, 0.1);
    EXPECT_TRUE(result.empty()) << "No faces → should return empty mesh";
}
