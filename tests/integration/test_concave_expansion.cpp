// Integration tests: concave shape expansion accuracy
//
// Tests verify:
//   1. Conservativeness: all input vertices are inside the output union
//   2. Accuracy: sum of output volumes vs ConservativeExpander's single volume
//      (RobustSlicer should produce a smaller total volume for concave shapes)
//   3. NaN/Inf safety

#include <gtest/gtest.h>
#include "expander/RobustSlicer.hpp"
#include "expander/ConservativeExpander.hpp"
#include "expander/StlWriter.hpp"

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const std::string kStlOutputDir = "stl_output";

using namespace expander;

// ---------------------------------------------------------------------------
// Volume of a triangulated closed mesh (divergence theorem)
// ---------------------------------------------------------------------------
static double meshVolume(const Mesh& m) {
    double vol = 0.0;
    for (const auto& f : m.faces) {
        const Eigen::Vector3d v0 = m.vertices.row(f[0]).transpose();
        const Eigen::Vector3d v1 = m.vertices.row(f[1]).transpose();
        const Eigen::Vector3d v2 = m.vertices.row(f[2]).transpose();
        vol += v0.dot(v1.cross(v2));
    }
    return std::abs(vol) / 6.0;
}

// ---------------------------------------------------------------------------
// Check if vertex v is inside any output mesh (AABB containment check)
// ---------------------------------------------------------------------------
static bool isInsideUnion(const Eigen::Vector3d&   v,
                            const std::vector<Mesh>& meshes,
                            double                   eps = 1e-5)
{
    for (const auto& m : meshes) {
        if (m.numVertices() == 0) continue;
        const Eigen::Vector3d lo = m.vertices.colwise().minCoeff().transpose();
        const Eigen::Vector3d hi = m.vertices.colwise().maxCoeff().transpose();
        if ((v.array() >= lo.array() - eps).all()
         && (v.array() <= hi.array() + eps).all())
            return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Mesh factories — with face connectivity
// ---------------------------------------------------------------------------

// L-shaped prism (concave cross-section extruded along Z)
// XY cross-section (CCW):  (-R,-R),(R,-R),(R,0),(0,0),(0,R),(-R,R)
// Z extent: [-H/2, H/2]
// Ideal volume = (2R * 2R - R * R) * H * 0.5 * 2 ... actually:
//   full square area = 4R^2, removed top-right square area = R^2
//   L-area = 4R^2 - R^2 = 3R^2
//   L-volume = 3R^2 * H
static Mesh makeLShapeMesh(double R = 10.0, double H = 20.0) {
    Mesh m;
    m.vertices.resize(12, 3);
    const double hz = H * 0.5;
    // Front (z=+hz): 0-5
    m.vertices.row(0)  = Eigen::RowVector3d(-R, -R,  hz);
    m.vertices.row(1)  = Eigen::RowVector3d( R, -R,  hz);
    m.vertices.row(2)  = Eigen::RowVector3d( R,  0,  hz);
    m.vertices.row(3)  = Eigen::RowVector3d( 0,  0,  hz);
    m.vertices.row(4)  = Eigen::RowVector3d( 0,  R,  hz);
    m.vertices.row(5)  = Eigen::RowVector3d(-R,  R,  hz);
    // Back (z=-hz): 6-11
    m.vertices.row(6)  = Eigen::RowVector3d(-R, -R, -hz);
    m.vertices.row(7)  = Eigen::RowVector3d( R, -R, -hz);
    m.vertices.row(8)  = Eigen::RowVector3d( R,  0, -hz);
    m.vertices.row(9)  = Eigen::RowVector3d( 0,  0, -hz);
    m.vertices.row(10) = Eigen::RowVector3d( 0,  R, -hz);
    m.vertices.row(11) = Eigen::RowVector3d(-R,  R, -hz);

    // Front face +z (fan from 0, CCW from +z)
    m.faces.push_back({0,1,2}); m.faces.push_back({0,2,3});
    m.faces.push_back({0,3,4}); m.faces.push_back({0,4,5});
    // Back face -z (reverse winding)
    m.faces.push_back({6,8,7}); m.faces.push_back({6,9,8});
    m.faces.push_back({6,10,9}); m.faces.push_back({6,11,10});
    // Side faces (quads → 2 triangles each)
    m.faces.push_back({0,6,7}); m.faces.push_back({0,7,1}); // bottom -y
    m.faces.push_back({1,7,8}); m.faces.push_back({1,8,2}); // right  +x
    m.faces.push_back({2,8,9}); m.faces.push_back({2,9,3}); // step   +y
    m.faces.push_back({3,9,10}); m.faces.push_back({3,10,4}); // inner +x
    m.faces.push_back({4,10,11}); m.faces.push_back({4,11,5}); // top   +y
    m.faces.push_back({5,11,6}); m.faces.push_back({5,6,0}); // left   -x

    return m;
}

// U-channel: base + two side walls (concave, open at top in Y)
// Cross-section: (-R,-R),(R,-R),(R,0),(-R,0) ... no, let's make it C-shaped
// C-shape cross-section (CCW): (-R,-R),(R,-R),(R,-t),(t,-t),(t,t),(R,t),(R,R),(-R,R)
// where t = 0.5*R (inner channel half-width = R-t)
static Mesh makeCShapeMesh(double R = 10.0, double H = 20.0) {
    const double t  = R * 0.5; // inner wall x-position
    const double hz = H * 0.5;
    Mesh m;
    m.vertices.resize(16, 3);

    // Front face (z=+hz): vertices 0-7 (cross-section CCW from +z)
    m.vertices.row(0) = Eigen::RowVector3d(-R, -R,  hz);
    m.vertices.row(1) = Eigen::RowVector3d( R, -R,  hz);
    m.vertices.row(2) = Eigen::RowVector3d( R, -t,  hz);
    m.vertices.row(3) = Eigen::RowVector3d( t, -t,  hz);
    m.vertices.row(4) = Eigen::RowVector3d( t,  t,  hz);
    m.vertices.row(5) = Eigen::RowVector3d( R,  t,  hz);
    m.vertices.row(6) = Eigen::RowVector3d( R,  R,  hz);
    m.vertices.row(7) = Eigen::RowVector3d(-R,  R,  hz);
    // Back face (z=-hz): vertices 8-15
    m.vertices.row(8)  = Eigen::RowVector3d(-R, -R, -hz);
    m.vertices.row(9)  = Eigen::RowVector3d( R, -R, -hz);
    m.vertices.row(10) = Eigen::RowVector3d( R, -t, -hz);
    m.vertices.row(11) = Eigen::RowVector3d( t, -t, -hz);
    m.vertices.row(12) = Eigen::RowVector3d( t,  t, -hz);
    m.vertices.row(13) = Eigen::RowVector3d( R,  t, -hz);
    m.vertices.row(14) = Eigen::RowVector3d( R,  R, -hz);
    m.vertices.row(15) = Eigen::RowVector3d(-R,  R, -hz);

    // Front face (fan from 0, CCW from +z)
    m.faces.push_back({0,1,2}); m.faces.push_back({0,2,7});
    m.faces.push_back({2,6,7}); m.faces.push_back({2,5,6});
    m.faces.push_back({2,3,5}); m.faces.push_back({3,4,5});
    // Back face (reversed)
    m.faces.push_back({8,10,9}); m.faces.push_back({8,15,10});
    m.faces.push_back({10,15,14}); m.faces.push_back({10,14,13});
    m.faces.push_back({10,13,11}); m.faces.push_back({11,13,12});
    // Side faces
    m.faces.push_back({0, 8, 9}); m.faces.push_back({0, 9, 1}); // bottom -y
    m.faces.push_back({1, 9,10}); m.faces.push_back({1,10, 2}); // right  +x bottom
    m.faces.push_back({2,10,11}); m.faces.push_back({2,11, 3}); // inner bottom +y
    m.faces.push_back({3,11,12}); m.faces.push_back({3,12, 4}); // inner right -x
    m.faces.push_back({4,12,13}); m.faces.push_back({4,13, 5}); // inner top -y
    m.faces.push_back({5,13,14}); m.faces.push_back({5,14, 6}); // right  +x top
    m.faces.push_back({6,14,15}); m.faces.push_back({6,15, 7}); // top    +y
    m.faces.push_back({7,15, 8}); m.faces.push_back({7, 8, 0}); // left   -x

    return m;
}

// ---------------------------------------------------------------------------
// Print helper
// ---------------------------------------------------------------------------
static void printComparison(const std::string& label,
                             double conservativeVol,
                             double robustTotal,
                             int    nMeshes,
                             double idealVol = -1.0)
{
    std::cout << std::fixed << std::setprecision(3)
              << "  " << std::left << std::setw(28) << label
              << "  conservative=" << std::setw(10) << conservativeVol
              << "  robust_sum=" << std::setw(10) << robustTotal
              << "  meshes=" << nMeshes;
    if (idealVol > 0.0) {
        std::cout << "  conservative/ideal=" << std::setw(6) << (conservativeVol / idealVol)
                  << "  robust/ideal=" << std::setw(6) << (robustTotal / idealVol);
    }
    std::cout << "\n";
}

// ---------------------------------------------------------------------------
// Test: L-shape conservativeness
// All input vertices must be inside the RobustSlicer output union.
// ---------------------------------------------------------------------------
TEST(ConcaveExpansion, LShapeConservativeness) {
    const double d = 1.0;
    Mesh lshape = makeLShapeMesh(10.0, 20.0);
    RobustSlicer slicer(8);
    auto meshes = slicer.expandMulti(lshape, d);

    ASSERT_GT(static_cast<int>(meshes.size()), 0)
        << "RobustSlicer returned no meshes for L-shape";

    for (int i = 0; i < lshape.numVertices(); ++i) {
        const Eigen::Vector3d v = lshape.vertices.row(i).transpose();
        EXPECT_TRUE(isInsideUnion(v, meshes, 1e-4))
            << "Input vertex " << i << " = " << v.transpose()
            << " not covered by output union";
    }
}

// ---------------------------------------------------------------------------
// Test: L-shape — RobustSlicer sum < ConservativeExpander volume
// The concave shape should benefit from per-box local normals.
// ---------------------------------------------------------------------------
TEST(ConcaveExpansion, LShapeRobustBetterThanConservative) {
    const double d     = 1.0;
    const double R     = 10.0, H = 20.0;
    const double idealVol = 3.0 * R * R * H;  // L cross-section = 3R^2

    Mesh lshape = makeLShapeMesh(R, H);

    ConservativeExpander cExp;
    Mesh cResult = cExp.expand(lshape, d);
    const double conservativeVol = (cResult.numFaces() > 0) ? meshVolume(cResult) : 0.0;

    RobustSlicer slicer(8);
    auto meshes = slicer.expandMulti(lshape, d);
    double robustTotal = 0.0;
    for (const auto& m : meshes)
        if (m.numFaces() > 0) robustTotal += meshVolume(m);

    std::cout << "\n[L-shape concave expansion: R=" << R << " H=" << H << " d=" << d << "]\n";
    printComparison("L-shape (R=10 H=20)", conservativeVol, robustTotal,
                    static_cast<int>(meshes.size()), idealVol);

    // Write STL for inspection
    try {
        std::filesystem::create_directories(kStlOutputDir);
        if (cResult.numFaces() > 0)
            StlWriter::write(kStlOutputDir + "/lshape_conservative.stl", cResult, "L-shape conservative");
        for (int i = 0; i < static_cast<int>(meshes.size()); ++i)
            if (meshes[i].numFaces() > 0)
                StlWriter::write(kStlOutputDir + "/lshape_robust_" + std::to_string(i) + ".stl",
                                 meshes[i], "L-shape robust " + std::to_string(i));
    } catch (...) {}

    EXPECT_GT(conservativeVol, 0.0);
    EXPECT_GT(robustTotal, 0.0);
    // RobustSlicer's volume sum should be smaller than ConservativeExpander's
    // (less over-expansion due to per-box local normals)
    EXPECT_LT(robustTotal, conservativeVol)
        << "RobustSlicer should produce smaller total volume than ConservativeExpander";
    // Both must be at least as large as ideal (conservativeness)
    EXPECT_GE(conservativeVol, idealVol);
    EXPECT_GE(robustTotal, idealVol);
}

// ---------------------------------------------------------------------------
// Test: C-shape conservativeness + NaN check
// ---------------------------------------------------------------------------
TEST(ConcaveExpansion, CShapeConservativenessAndNoNaN) {
    const double d = 1.0;
    Mesh cshape = makeCShapeMesh(10.0, 20.0);
    RobustSlicer slicer(8);
    auto meshes = slicer.expandMulti(cshape, d);

    ASSERT_GT(static_cast<int>(meshes.size()), 0)
        << "RobustSlicer returned no meshes for C-shape";

    // NaN check
    for (int mi = 0; mi < static_cast<int>(meshes.size()); ++mi)
        for (int vi = 0; vi < meshes[mi].numVertices(); ++vi)
            EXPECT_TRUE(meshes[mi].vertices.row(vi).allFinite())
                << "NaN/Inf at C-shape mesh " << mi << " vertex " << vi;

    // Conservativeness
    for (int i = 0; i < cshape.numVertices(); ++i) {
        const Eigen::Vector3d v = cshape.vertices.row(i).transpose();
        EXPECT_TRUE(isInsideUnion(v, meshes, 1e-4))
            << "C-shape input vertex " << i << " not covered";
    }
}

// ---------------------------------------------------------------------------
// Test: multiple resolution values — robustness sweep
// ---------------------------------------------------------------------------
TEST(ConcaveExpansion, ResolutionSweepNoNaN) {
    Mesh lshape = makeLShapeMesh(10.0, 20.0);
    for (int res : {2, 4, 8, 16}) {
        SCOPED_TRACE("resolution=" + std::to_string(res));
        RobustSlicer slicer(res);
        auto meshes = slicer.expandMulti(lshape, 1.0);
        EXPECT_GT(static_cast<int>(meshes.size()), 0);
        for (int mi = 0; mi < static_cast<int>(meshes.size()); ++mi)
            for (int vi = 0; vi < meshes[mi].numVertices(); ++vi)
                EXPECT_TRUE(meshes[mi].vertices.row(vi).allFinite())
                    << "NaN at resolution=" << res << " mesh=" << mi << " v=" << vi;
    }
}
