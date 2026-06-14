#include <gtest/gtest.h>
#include "expander/AssemblyExpander.hpp"
#include "expander/BoxExpander.hpp"

using namespace expander;

// ---------------------------------------------------------------------------
// Mesh factories
// ---------------------------------------------------------------------------

static Mesh makeCubeMesh(double half = 1.0,
                          Eigen::Vector3d offset = Eigen::Vector3d::Zero())
{
    Mesh m;
    m.vertices.resize(8, 3);
    // clang-format off
    m.vertices << -half+offset.x(), -half+offset.y(), -half+offset.z(),
                   half+offset.x(), -half+offset.y(), -half+offset.z(),
                   half+offset.x(),  half+offset.y(), -half+offset.z(),
                  -half+offset.x(),  half+offset.y(), -half+offset.z(),
                  -half+offset.x(), -half+offset.y(),  half+offset.z(),
                   half+offset.x(), -half+offset.y(),  half+offset.z(),
                   half+offset.x(),  half+offset.y(),  half+offset.z(),
                  -half+offset.x(),  half+offset.y(),  half+offset.z();
    // clang-format on
    m.faces = {
        {0,2,1},{0,3,2},   // -z
        {4,5,6},{4,6,7},   // +z
        {0,1,5},{0,5,4},   // -y
        {2,3,7},{2,7,6},   // +y
        {1,2,6},{1,6,5},   // +x
        {0,4,7},{0,7,3},   // -x
    };
    return m;
}

// L-shaped concave mesh
static Mesh makeLShapeMesh()
{
    Mesh m;
    m.vertices.resize(12, 3);
    m.vertices.row(0)  = Eigen::RowVector3d(-1, -1,  1);
    m.vertices.row(1)  = Eigen::RowVector3d( 1, -1,  1);
    m.vertices.row(2)  = Eigen::RowVector3d( 1,  0,  1);
    m.vertices.row(3)  = Eigen::RowVector3d( 0,  0,  1);
    m.vertices.row(4)  = Eigen::RowVector3d( 0,  1,  1);
    m.vertices.row(5)  = Eigen::RowVector3d(-1,  1,  1);
    m.vertices.row(6)  = Eigen::RowVector3d(-1, -1, -1);
    m.vertices.row(7)  = Eigen::RowVector3d( 1, -1, -1);
    m.vertices.row(8)  = Eigen::RowVector3d( 1,  0, -1);
    m.vertices.row(9)  = Eigen::RowVector3d( 0,  0, -1);
    m.vertices.row(10) = Eigen::RowVector3d( 0,  1, -1);
    m.vertices.row(11) = Eigen::RowVector3d(-1,  1, -1);
    m.faces.push_back({0, 1, 2}); m.faces.push_back({0, 2, 3});
    m.faces.push_back({0, 3, 4}); m.faces.push_back({0, 4, 5});
    m.faces.push_back({6, 8, 7}); m.faces.push_back({6, 9, 8});
    m.faces.push_back({6,10, 9}); m.faces.push_back({6,11,10});
    m.faces.push_back({0, 6, 7}); m.faces.push_back({0, 7, 1});
    m.faces.push_back({1, 7, 8}); m.faces.push_back({1, 8, 2});
    m.faces.push_back({2, 8, 9}); m.faces.push_back({2, 9, 3});
    m.faces.push_back({3, 9,10}); m.faces.push_back({3,10, 4});
    m.faces.push_back({4,10,11}); m.faces.push_back({4,11, 5});
    m.faces.push_back({5,11, 6}); m.faces.push_back({5, 6, 0});
    return m;
}

// ---------------------------------------------------------------------------
// mergeContained
// ---------------------------------------------------------------------------
TEST(AssemblyExpander, MergeContained_SmallInsideLarge) {
    // Inner cube (half=0.3) fully inside outer cube (half=1.0)
    Mesh outer = makeCubeMesh(1.0);
    Mesh inner = makeCubeMesh(0.3);

    auto result = AssemblyExpander::mergeContained({outer, inner});

    // Should merge into 1 part
    ASSERT_EQ(result.size(), 1u);
    // The merged mesh should have vertices from both
    EXPECT_EQ(result[0].numVertices(), outer.numVertices() + inner.numVertices());
    EXPECT_EQ(result[0].numFaces(),    outer.numFaces()    + inner.numFaces());
}

TEST(AssemblyExpander, MergeContained_NonOverlapping) {
    // Two cubes far apart — neither contains the other
    Mesh a = makeCubeMesh(1.0, Eigen::Vector3d(-5, 0, 0));
    Mesh b = makeCubeMesh(1.0, Eigen::Vector3d( 5, 0, 0));

    auto result = AssemblyExpander::mergeContained({a, b});

    EXPECT_EQ(result.size(), 2u);
}

TEST(AssemblyExpander, MergeContained_SameSize) {
    // Two identical cubes — neither strictly contains the other
    Mesh a = makeCubeMesh(1.0);
    Mesh b = makeCubeMesh(1.0);

    auto result = AssemblyExpander::mergeContained({a, b});

    // Neither is contained in the other by volume; both remain
    EXPECT_EQ(result.size(), 2u);
}

TEST(AssemblyExpander, MergeContained_EmptyInput) {
    EXPECT_TRUE(AssemblyExpander::mergeContained({}).empty());
}

TEST(AssemblyExpander, MergeContained_SinglePart) {
    auto result = AssemblyExpander::mergeContained({makeCubeMesh()});
    ASSERT_EQ(result.size(), 1u);
}

// ---------------------------------------------------------------------------
// expand — single and multi part
// ---------------------------------------------------------------------------
TEST(AssemblyExpander, ExpandConvexPart_NonEmpty) {
    AssemblyExpander exp;
    auto results = exp.expand({makeCubeMesh()}, 0.1);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_GT(results[0].numFaces(), 0);
}

TEST(AssemblyExpander, ExpandConcavePart_NonEmpty) {
    // BoxExpander (削り出し法) applies to all parts, including concave
    AssemblyExpander exp;
    auto results = exp.expand({makeLShapeMesh()}, 0.1);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_GT(results[0].numFaces(), 0);
}

// 凹分解ブランチ end-to-end: Options で分解を有効化すると AssemblyExpander 経由で
// 複数ボックスが削り出され mergeMeshes で連結される（単一凸より面数が増える）。
TEST(AssemblyExpander, ExpandConcavePart_DecompositionBranch) {
    const Mesh L = makeLShapeMesh();

    // 既定（単一凸）の面数
    const int singleFaces = BoxExpander().expand(L, 0.1).numFaces();

    AssemblyExpander::Options o;
    o.maxConvexPieces = 8;
    AssemblyExpander exp(o);
    auto results = exp.expand({L}, 0.1);

    ASSERT_EQ(results.size(), 1u);
    EXPECT_TRUE(results[0].vertices.allFinite());
    EXPECT_GT(results[0].numFaces(), singleFaces)
        << "分解ブランチが走り複数ピースが連結されるはず";
}

// 後方互換: 既定 Options は単一凸（BoxExpander 直呼びと同じ面数）。
TEST(AssemblyExpander, DefaultOptionsMatchSingleConvex) {
    const Mesh L = makeLShapeMesh();
    const int singleFaces = BoxExpander().expand(L, 0.1).numFaces();
    AssemblyExpander exp;   // default options
    EXPECT_EQ(exp.expand({L}, 0.1)[0].numFaces(), singleFaces);
}

// concavityTol 単独指定（maxConvexPieces 未指定）でも分割が走る。
TEST(AssemblyExpander, ConcavityTolAloneTriggersSplit) {
    const Mesh L = makeLShapeMesh();
    const int singleFaces = BoxExpander().expand(L, 0.1).numFaces();
    AssemblyExpander::Options o;
    o.concavityTol = 0.1;   // maxConvexPieces は既定の 1 のまま
    AssemblyExpander exp(o);
    EXPECT_GT(exp.expand({L}, 0.1)[0].numFaces(), singleFaces);
}

TEST(AssemblyExpander, ExpandMultiPart_CountPreserved) {
    AssemblyExpander exp;

    Mesh cube   = makeCubeMesh(1.0, Eigen::Vector3d(-5, 0, 0));
    Mesh lshape = makeLShapeMesh();  // around origin

    auto results = exp.expand({cube, lshape}, 0.1);
    ASSERT_EQ(results.size(), 2u);
    EXPECT_GT(results[0].numFaces(), 0);
    EXPECT_GT(results[1].numFaces(), 0);
}

// ---------------------------------------------------------------------------
// expandMerged
// ---------------------------------------------------------------------------
TEST(AssemblyExpander, ExpandMerged_NonEmpty) {
    AssemblyExpander exp;

    Mesh a = makeCubeMesh(1.0, Eigen::Vector3d(-5, 0, 0));
    Mesh b = makeCubeMesh(1.0, Eigen::Vector3d( 5, 0, 0));
    Mesh merged = exp.expandMerged({a, b}, 0.1);

    EXPECT_GT(merged.numFaces(), 0);
    EXPECT_TRUE(merged.vertices.allFinite());
}

TEST(AssemblyExpander, ExpandMerged_EmptyInput) {
    AssemblyExpander exp;
    EXPECT_TRUE(exp.expandMerged({}, 0.1).empty());
}
