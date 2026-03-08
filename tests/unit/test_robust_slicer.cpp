#include <gtest/gtest.h>
#include "expander/RobustSlicer.hpp"

using namespace expander;

// ---------------------------------------------------------------------------
// Mesh factories
// ---------------------------------------------------------------------------

// Closed cube mesh with face connectivity
static Mesh makeCubeMesh(double half = 1.0) {
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
    // 12 triangles (6 faces × 2)
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

// Closed L-shaped mesh (concave, 12 vertices, 20 triangles)
// Cross-section polygon (CCW from +z):
//   (-1,-1), (1,-1), (1,0), (0,0), (0,1), (-1,1)
// Extruded in z: -1 to +1
static Mesh makeLShapeMesh() {
    Mesh m;
    m.vertices.resize(12, 3);
    // Front face (z=+1): vertices 0-5
    m.vertices.row(0)  = Eigen::RowVector3d(-1, -1,  1);
    m.vertices.row(1)  = Eigen::RowVector3d( 1, -1,  1);
    m.vertices.row(2)  = Eigen::RowVector3d( 1,  0,  1);
    m.vertices.row(3)  = Eigen::RowVector3d( 0,  0,  1);
    m.vertices.row(4)  = Eigen::RowVector3d( 0,  1,  1);
    m.vertices.row(5)  = Eigen::RowVector3d(-1,  1,  1);
    // Back face (z=-1): vertices 6-11
    m.vertices.row(6)  = Eigen::RowVector3d(-1, -1, -1);
    m.vertices.row(7)  = Eigen::RowVector3d( 1, -1, -1);
    m.vertices.row(8)  = Eigen::RowVector3d( 1,  0, -1);
    m.vertices.row(9)  = Eigen::RowVector3d( 0,  0, -1);
    m.vertices.row(10) = Eigen::RowVector3d( 0,  1, -1);
    m.vertices.row(11) = Eigen::RowVector3d(-1,  1, -1);

    // Front face (normal +z): fan from 0
    m.faces.push_back({0, 1, 2}); m.faces.push_back({0, 2, 3});
    m.faces.push_back({0, 3, 4}); m.faces.push_back({0, 4, 5});
    // Back face (normal -z): reverse winding
    m.faces.push_back({6, 8, 7}); m.faces.push_back({6, 9, 8});
    m.faces.push_back({6,10, 9}); m.faces.push_back({6,11,10});
    // Side faces (each quad → 2 triangles, outward winding verified)
    m.faces.push_back({0, 6, 7}); m.faces.push_back({0, 7, 1}); // bottom -y
    m.faces.push_back({1, 7, 8}); m.faces.push_back({1, 8, 2}); // right  +x
    m.faces.push_back({2, 8, 9}); m.faces.push_back({2, 9, 3}); // step   +y
    m.faces.push_back({3, 9,10}); m.faces.push_back({3,10, 4}); // inner  +x
    m.faces.push_back({4,10,11}); m.faces.push_back({4,11, 5}); // top    +y
    m.faces.push_back({5,11, 6}); m.faces.push_back({5, 6, 0}); // left   -x

    return m;
}

// ---------------------------------------------------------------------------
// Helper: check that every input vertex is inside the AABB of some output mesh
// (sufficient for conservativeness since each output polytope ⊆ expandedBox)
// ---------------------------------------------------------------------------
static bool allVerticesCovered(const Mesh&               input,
                                 const std::vector<Mesh>&  outputs,
                                 double                     eps = 1e-6)
{
    for (int i = 0; i < input.numVertices(); ++i) {
        const Eigen::Vector3d v = input.vertices.row(i).transpose();
        bool covered = false;
        for (const auto& m : outputs) {
            if (m.numVertices() == 0) continue;
            const Eigen::Vector3d lo = m.vertices.colwise().minCoeff().transpose();
            const Eigen::Vector3d hi = m.vertices.colwise().maxCoeff().transpose();
            if ((v.array() >= lo.array() - eps).all()
             && (v.array() <= hi.array() + eps).all()) {
                covered = true;
                break;
            }
        }
        if (!covered) return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Test: empty input → empty output
// ---------------------------------------------------------------------------
TEST(RobustSlicer, EmptyInputReturnsEmpty) {
    Mesh empty;
    empty.vertices.resize(0, 3);
    RobustSlicer slicer(4);
    EXPECT_TRUE(slicer.expandMulti(empty, 0.1).empty());
}

TEST(RobustSlicer, NoFacesReturnsEmpty) {
    // Vertices only, no face connectivity
    Mesh m;
    m.vertices.resize(4, 3);
    m.vertices << 0,0,0, 1,0,0, 0,1,0, 0,0,1;
    RobustSlicer slicer(4);
    EXPECT_TRUE(slicer.expandMulti(m, 0.1).empty());
}

// ---------------------------------------------------------------------------
// Test: cube produces at least one valid mesh, all vertices finite
// ---------------------------------------------------------------------------
TEST(RobustSlicer, CubeProducesValidOutput) {
    Mesh cube = makeCubeMesh(1.0);
    RobustSlicer slicer(4);
    auto meshes = slicer.expandMulti(cube, 0.1);

    ASSERT_GT(static_cast<int>(meshes.size()), 0) << "No output meshes";
    for (int mi = 0; mi < static_cast<int>(meshes.size()); ++mi) {
        const auto& m = meshes[mi];
        EXPECT_GT(m.numFaces(), 0) << "Mesh " << mi << " has no faces";
        for (int vi = 0; vi < m.numVertices(); ++vi)
            EXPECT_TRUE(m.vertices.row(vi).allFinite())
                << "NaN/Inf at mesh " << mi << " vertex " << vi;
    }
}

// ---------------------------------------------------------------------------
// Test: cube — all input vertices covered by output union
// ---------------------------------------------------------------------------
TEST(RobustSlicer, CubeAllVerticesCovered) {
    const double d = 0.1;
    Mesh cube = makeCubeMesh(1.0);
    RobustSlicer slicer(4);
    auto meshes = slicer.expandMulti(cube, d);

    ASSERT_GT(static_cast<int>(meshes.size()), 0);
    EXPECT_TRUE(allVerticesCovered(cube, meshes, 1e-5))
        << "Some cube vertices are not covered by any output mesh";
}

// ---------------------------------------------------------------------------
// Test: L-shape — all input vertices covered
// ---------------------------------------------------------------------------
TEST(RobustSlicer, LShapeAllVerticesCovered) {
    const double d = 0.1;
    Mesh lshape = makeLShapeMesh();
    RobustSlicer slicer(8);
    auto meshes = slicer.expandMulti(lshape, d);

    ASSERT_GT(static_cast<int>(meshes.size()), 0);
    EXPECT_TRUE(allVerticesCovered(lshape, meshes, 1e-5))
        << "Some L-shape vertices are not covered by any output mesh";
}

// ---------------------------------------------------------------------------
// Test: L-shape — no NaN in any output mesh
// ---------------------------------------------------------------------------
TEST(RobustSlicer, LShapeNoNaN) {
    Mesh lshape = makeLShapeMesh();
    RobustSlicer slicer(8);
    auto meshes = slicer.expandMulti(lshape, 0.1);

    for (int mi = 0; mi < static_cast<int>(meshes.size()); ++mi)
        for (int vi = 0; vi < meshes[mi].numVertices(); ++vi)
            EXPECT_TRUE(meshes[mi].vertices.row(vi).allFinite())
                << "NaN/Inf at mesh " << mi << " vertex " << vi;
}

// ---------------------------------------------------------------------------
// Test: IExpander::expand() returns non-empty for cube
// ---------------------------------------------------------------------------
TEST(RobustSlicer, ExpandCompatibilityReturnsSomething) {
    Mesh cube = makeCubeMesh(1.0);
    RobustSlicer slicer(4);
    Mesh result = slicer.expand(cube, 0.1);
    EXPECT_GT(result.numFaces(), 0);
}

// ---------------------------------------------------------------------------
// Test: resolution=1 → single-cell grid, still produces output
// ---------------------------------------------------------------------------
TEST(RobustSlicer, SingleCellResolutionDoesNotCrash) {
    Mesh cube = makeCubeMesh(1.0);
    RobustSlicer slicer(1);
    auto meshes = slicer.expandMulti(cube, 0.5);
    // Should not crash; may produce 0 or more meshes
    for (const auto& m : meshes)
        for (int vi = 0; vi < m.numVertices(); ++vi)
            EXPECT_TRUE(m.vertices.row(vi).allFinite());
}
