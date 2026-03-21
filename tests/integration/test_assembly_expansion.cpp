#include <gtest/gtest.h>
#include "expander/AssemblyExpander.hpp"

using namespace expander;

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

static Mesh makeCubeMesh(double half,
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
        {0,2,1},{0,3,2}, {4,5,6},{4,6,7},
        {0,1,5},{0,5,4}, {2,3,7},{2,7,6},
        {1,2,6},{1,6,5}, {0,4,7},{0,7,3},
    };
    return m;
}

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
    m.faces.push_back({0,1,2}); m.faces.push_back({0,2,3});
    m.faces.push_back({0,3,4}); m.faces.push_back({0,4,5});
    m.faces.push_back({6,8,7}); m.faces.push_back({6,9,8});
    m.faces.push_back({6,10,9}); m.faces.push_back({6,11,10});
    m.faces.push_back({0,6,7}); m.faces.push_back({0,7,1});
    m.faces.push_back({1,7,8}); m.faces.push_back({1,8,2});
    m.faces.push_back({2,8,9}); m.faces.push_back({2,9,3});
    m.faces.push_back({3,9,10}); m.faces.push_back({3,10,4});
    m.faces.push_back({4,10,11}); m.faces.push_back({4,11,5});
    m.faces.push_back({5,11,6}); m.faces.push_back({5,6,0});
    return m;
}

// Check that all input vertices are covered by the expanded output
static bool allVerticesCovered(const Mesh& input, const Mesh& expanded,
                                double eps = 1e-5)
{
    if (expanded.empty()) return false;
    const Eigen::Vector3d lo = expanded.vertices.colwise().minCoeff().transpose();
    const Eigen::Vector3d hi = expanded.vertices.colwise().maxCoeff().transpose();
    for (int i = 0; i < input.numVertices(); ++i) {
        const Eigen::Vector3d v = input.vertices.row(i).transpose();
        if ((v.array() < lo.array() - eps).any() ||
            (v.array() > hi.array() + eps).any())
            return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Two-cube assembly: conservativeness for each part
// ---------------------------------------------------------------------------
TEST(AssemblyExpansion, TwoCubes_EachVertexCovered)
{
    const double d = 0.5;
    Mesh a = makeCubeMesh(3.0, Eigen::Vector3d(-8, 0, 0));
    Mesh b = makeCubeMesh(2.0, Eigen::Vector3d( 8, 0, 0));

    AssemblyExpander exp;
    auto expanded = exp.expand({a, b}, d);
    ASSERT_EQ(expanded.size(), 2u);

    EXPECT_TRUE(allVerticesCovered(a, expanded[0]))
        << "Part A vertices not all covered after expansion";
    EXPECT_TRUE(allVerticesCovered(b, expanded[1]))
        << "Part B vertices not all covered after expansion";
}

// ---------------------------------------------------------------------------
// Mixed assembly: convex + concave
// ---------------------------------------------------------------------------
TEST(AssemblyExpansion, MixedAssembly_ConvexAndConcave_AllVerticesCovered)
{
    const double d = 0.1;
    Mesh cube   = makeCubeMesh(3.0, Eigen::Vector3d(-8, 0, 0));
    Mesh lshape = makeLShapeMesh();

    AssemblyExpander::Options opts;
    opts.resolution = 16;
    AssemblyExpander exp(opts);

    auto expanded = exp.expand({cube, lshape}, d);
    ASSERT_EQ(expanded.size(), 2u);

    EXPECT_TRUE(allVerticesCovered(cube,   expanded[0]))
        << "Cube vertices not covered";
    EXPECT_TRUE(allVerticesCovered(lshape, expanded[1]))
        << "L-shape vertices not covered";
}

// ---------------------------------------------------------------------------
// mergeContained + expand: nested parts treated as one
// ---------------------------------------------------------------------------
TEST(AssemblyExpansion, MergeContained_ThenExpand_Conservativeness)
{
    const double d = 0.1;
    Mesh outer = makeCubeMesh(5.0);
    Mesh inner = makeCubeMesh(1.0);  // fully inside outer

    auto merged = AssemblyExpander::mergeContained({outer, inner});
    ASSERT_EQ(merged.size(), 1u)
        << "Inner cube should be absorbed into outer";

    AssemblyExpander exp;
    auto expanded = exp.expand(merged, d);
    ASSERT_EQ(expanded.size(), 1u);

    // All vertices of the original outer cube must be covered
    EXPECT_TRUE(allVerticesCovered(outer, expanded[0]))
        << "Outer cube vertices not covered after merge+expand";
    // All vertices of the original inner cube must also be covered
    EXPECT_TRUE(allVerticesCovered(inner, expanded[0]))
        << "Inner cube vertices not covered after merge+expand";
}

// ---------------------------------------------------------------------------
// expandMerged: output is a valid single mesh
// ---------------------------------------------------------------------------
TEST(AssemblyExpansion, ExpandMerged_ValidMesh)
{
    Mesh a = makeCubeMesh(2.0, Eigen::Vector3d(-6, 0, 0));
    Mesh b = makeCubeMesh(2.0, Eigen::Vector3d( 6, 0, 0));

    AssemblyExpander exp;
    Mesh result = exp.expandMerged({a, b}, 0.2);

    EXPECT_GT(result.numFaces(), 0);
    EXPECT_TRUE(result.vertices.allFinite());
    // No NaN/Inf in faces
    for (const auto& f : result.faces) {
        EXPECT_GE(f[0], 0); EXPECT_LT(f[0], result.numVertices());
        EXPECT_GE(f[1], 0); EXPECT_LT(f[1], result.numVertices());
        EXPECT_GE(f[2], 0); EXPECT_LT(f[2], result.numVertices());
    }
}

// ---------------------------------------------------------------------------
// Expansion ratio: per-part expansion should be tighter than single-mesh expansion
// for a two-part assembly where parts are far apart.
// Expected: per-part total volume < single-mesh volume.
// ---------------------------------------------------------------------------
TEST(AssemblyExpansion, PerPartExpansion_TighterThanSingleMesh)
{
    const double d = 0.5;
    // Two cubes far apart
    Mesh a = makeCubeMesh(2.0, Eigen::Vector3d(-10, 0, 0));
    Mesh b = makeCubeMesh(2.0, Eigen::Vector3d( 10, 0, 0));

    // Merge into one mesh and expand as a single unit
    AssemblyExpander exp;
    std::vector<Mesh> merged = {exp.expandMerged({a, b}, d)};
    const Eigen::Vector3d merged_lo =
        merged[0].vertices.colwise().minCoeff().transpose();
    const Eigen::Vector3d merged_hi =
        merged[0].vertices.colwise().maxCoeff().transpose();
    const double singleVol = (merged_hi - merged_lo).prod();

    // Expand per part
    auto perPart = exp.expand({a, b}, d);
    double perPartVol = 0.0;
    for (const auto& m : perPart) {
        const Eigen::Vector3d lo = m.vertices.colwise().minCoeff().transpose();
        const Eigen::Vector3d hi = m.vertices.colwise().maxCoeff().transpose();
        perPartVol += (hi - lo).prod();
    }

    EXPECT_LT(perPartVol, singleVol)
        << "Per-part AABB volume (" << perPartVol
        << ") should be less than single-mesh AABB volume (" << singleVol
        << ") for widely separated parts";
}
