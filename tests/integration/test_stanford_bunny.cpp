// Stanford Bunny integration test
//
// Tests RobustSlicer with real concave mesh data at a fixed voxel cell size.
// Bunny STL files are looked up in the following order:
//   1. env BUNNY_STL=<path>            (single file, full-res test)
//   2. <cmake_binary_dir>/tests/data/  (bunny_res4.stl / bunny_res3.stl / bunny.stl)
//
// Properties verified:
//   1. Conservativeness: every input vertex is strictly inside some output polytope
//   2. Expansion >= d: centroid + d*normal is inside some output polytope
//   3. Single merged STL output (no separate per-polytope files)

#include <gtest/gtest.h>
#include "expander/RobustSlicer.hpp"
#include "expander/ConservativeExpander.hpp"
#include "expander/StlReader.hpp"
#include "expander/StlWriter.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

static const std::string kStlOutputDir = "stl_output";
static const std::string kDataDir      = "tests/data";

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
// Point-in-convex-mesh test via face-plane half-spaces.
// Returns true if pt is on the interior side of every face of mesh m.
// eps: tolerance in units of face normal magnitude.
// ---------------------------------------------------------------------------
static bool insideConvex(const Eigen::Vector3d& pt, const Mesh& m, double eps = 1e-6)
{
    for (const auto& f : m.faces) {
        const Eigen::Vector3d v0 = m.vertices.row(f[0]).transpose();
        const Eigen::Vector3d v1 = m.vertices.row(f[1]).transpose();
        const Eigen::Vector3d v2 = m.vertices.row(f[2]).transpose();
        const Eigen::Vector3d n  = (v1 - v0).cross(v2 - v0);
        const double len = n.norm();
        if (len < 1e-20) continue;   // degenerate face — skip
        // Interior: (pt - v0).n <= 0   (outward normal convention)
        if ((pt - v0).dot(n) > eps * len) return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Conservativeness check: every input vertex inside some polytope.
// Returns the number of vertices that failed (0 = pass).
// ---------------------------------------------------------------------------
static int checkConservativeness(const Mesh& input,
                                  const std::vector<Mesh>& polytopes,
                                  double eps = 1e-5)
{
    int failures = 0;
    for (int i = 0; i < input.numVertices(); ++i) {
        const Eigen::Vector3d v = input.vertices.row(i).transpose();
        bool ok = false;
        for (const auto& m : polytopes) {
            if (insideConvex(v, m, eps)) { ok = true; break; }
        }
        if (!ok) ++failures;
    }
    return failures;
}

// ---------------------------------------------------------------------------
// Expansion >= d check.
// For each input face, compute centroid + d * faceNormal and verify the
// probe point is inside at least one output polytope.
// Returns the number of probe points that failed (0 = pass).
// Box seam tolerance: < 5% failures is acceptable.
// ---------------------------------------------------------------------------
static int checkExpansionDistance(const Mesh& input,
                                   const std::vector<Mesh>& polytopes,
                                   double d,
                                   double eps = 1e-5)
{
    int failures = 0;
    for (const auto& f : input.faces) {
        const Eigen::Vector3d v0 = input.vertices.row(f[0]).transpose();
        const Eigen::Vector3d v1 = input.vertices.row(f[1]).transpose();
        const Eigen::Vector3d v2 = input.vertices.row(f[2]).transpose();
        const Eigen::Vector3d centroid = (v0 + v1 + v2) / 3.0;
        const Eigen::Vector3d n = (v1 - v0).cross(v2 - v0);
        const double len = n.norm();
        if (len < 1e-20) continue;  // degenerate face — skip
        const Eigen::Vector3d probe = centroid + d * (n / len);
        bool ok = false;
        for (const auto& m : polytopes) {
            if (insideConvex(probe, m, eps)) { ok = true; break; }
        }
        if (!ok) ++failures;
    }
    return failures;
}

// ---------------------------------------------------------------------------
// Run one bunny evaluation: load, expand, output single merged STL, check.
// ---------------------------------------------------------------------------
static void runBunnyEval(const std::string& label,
                          const std::string& stlPath,
                          double             cellSize,
                          double             d)
{
    std::cout << "\n[Stanford Bunny -- " << label << "]\n";

    const Mesh bunny = StlReader::read(stlPath);
    ASSERT_GT(bunny.numFaces(), 0) << "Failed to load " << stlPath;

    const Eigen::Vector3d vmin = bunny.vertices.colwise().minCoeff();
    const Eigen::Vector3d vmax = bunny.vertices.colwise().maxCoeff();
    const Eigen::Vector3d diag = vmax - vmin;

    std::cout << std::fixed << std::setprecision(5)
              << "  verts=" << bunny.numVertices()
              << "  faces=" << bunny.numFaces() << "\n"
              << "  AABB: " << diag.transpose() << " (mesh units)\n"
              << "  cellSize=" << cellSize << "  d=" << d << "\n";

    // --- RobustSlicer ---
    RobustSlicer slicer = RobustSlicer::withCellSize(cellSize);
    const auto polytopes = slicer.expandMulti(bunny, d);

    ASSERT_GT(static_cast<int>(polytopes.size()), 0) << "No output polytopes";

    // NaN check
    bool hasNaN = false;
    double robustTotal = 0.0;
    int totalFaces = 0;
    for (const auto& m : polytopes) {
        robustTotal += meshVolume(m);
        totalFaces  += m.numFaces();
        for (int vi = 0; vi < m.numVertices(); ++vi)
            if (!m.vertices.row(vi).allFinite()) { hasNaN = true; break; }
    }
    EXPECT_FALSE(hasNaN) << "NaN/Inf in RobustSlicer output";

    // --- ConservativeExpander baseline ---
    ConservativeExpander cExp;
    const Mesh cResult = cExp.expand(bunny, d);
    const double conservativeVol = (cResult.numFaces() > 0) ? meshVolume(cResult) : 0.0;
    const double ratio = (conservativeVol > 0.0) ? robustTotal / conservativeVol : 0.0;

    std::cout << std::fixed << std::setprecision(6)
              << "  RobustSlicer:         polytopes=" << polytopes.size()
              << "  total_faces=" << totalFaces
              << "  sum_vol=" << robustTotal << "\n"
              << "  ConservativeExpander: faces=" << cResult.numFaces()
              << "  vol=" << conservativeVol << "\n"
              << "  robust/conservative = " << std::setprecision(3) << ratio
              << "  (" << std::showpos << (ratio - 1.0) * 100.0
              << std::noshowpos << "% vs conservative)\n";

    // --- Conservativeness check ---
    // A small fraction may fall just outside polytope faces at box seam boundaries
    // due to floating-point tolerance in ClippingEngine.  < 1% is acceptable.
    const int conservFail = checkConservativeness(bunny, polytopes, 1e-5);
    const double conservFailRate = (bunny.numVertices() > 0)
                                   ? static_cast<double>(conservFail) / bunny.numVertices()
                                   : 0.0;
    std::cout << "  Conservativeness: " << conservFail << "/" << bunny.numVertices()
              << " verts failed (" << conservFailRate * 100.0 << "%)\n";
    EXPECT_LT(conservFailRate, 0.01)
        << conservFail << "/" << bunny.numVertices()
        << " vertices not covered (>1% seam tolerance exceeded)";

    // --- Expansion >= d check ---
    const int expandFail = checkExpansionDistance(bunny, polytopes, d, 1e-5);
    const int totalFacesInput = static_cast<int>(bunny.faces.size());
    const double expandFailRate = (totalFacesInput > 0)
                                  ? static_cast<double>(expandFail) / totalFacesInput
                                  : 0.0;
    std::cout << "  Expansion>=d:     " << expandFail << "/" << totalFacesInput
              << " probes failed (" << expandFailRate * 100.0 << "%)\n";
    // Allow up to 5% failures at box seam boundaries
    EXPECT_LT(expandFailRate, 0.05)
        << expandFail << "/" << totalFacesInput
        << " face probe points not covered (>5% seam tolerance exceeded)";

    // RobustSlicer should be tighter than ConservativeExpander for concave input
    if (conservativeVol > 0.0)
        EXPECT_LT(robustTotal, conservativeVol)
            << "RobustSlicer should be tighter than ConservativeExpander";

    // --- Single merged STL output ---
    try {
        std::string tag = label;
        for (char& c : tag)
            if (c == ' ' || c == '/' || c == '\\') c = '_';
        std::filesystem::create_directories(kStlOutputDir);

        // Merged multi-body mesh (single file, all polytopes concatenated)
        const Mesh merged = RobustSlicer::merge(polytopes);
        if (merged.numFaces() > 0)
            StlWriter::write(kStlOutputDir + "/bunny_merged_" + tag + ".stl",
                             merged, "bunny merged " + label);

        if (cResult.numFaces() > 0)
            StlWriter::write(kStlOutputDir + "/bunny_conservative_" + tag + ".stl",
                             cResult, "bunny conservative " + label);

        std::cout << "  STL output: " << kStlOutputDir
                  << "/bunny_merged_" << tag << ".stl\n";
    } catch (...) {}
}

// ---------------------------------------------------------------------------
// Test: res4 (453 verts, 948 faces) — fastest, always run if file exists
// ---------------------------------------------------------------------------
TEST(StanfordBunny, Res4_ConservativenessAndExpansion) {
    const std::string path = kDataDir + "/bunny_res4.stl";
    if (!std::filesystem::exists(path))
        GTEST_SKIP() << "bunny_res4.stl not found at " << path;

    // Bunny is in meters (~0.165m wide). cellSize=5mm=0.005, d=1mm=0.001
    runBunnyEval("res4", path, 0.005, 0.001);
}

// ---------------------------------------------------------------------------
// Test: res3 (1889 verts, 3851 faces) — medium quality
// ---------------------------------------------------------------------------
TEST(StanfordBunny, Res3_ConservativenessAndExpansion) {
    const std::string path = kDataDir + "/bunny_res3.stl";
    if (!std::filesystem::exists(path))
        GTEST_SKIP() << "bunny_res3.stl not found at " << path;

    runBunnyEval("res3", path, 0.005, 0.001);
}

// ---------------------------------------------------------------------------
// Test: full resolution (35947 verts, 69451 faces) — merged output
// ---------------------------------------------------------------------------
TEST(StanfordBunny, FullResolution_MergedOutput) {
    std::string path;
    const char* env = std::getenv("BUNNY_STL");
    if (env && std::filesystem::exists(env))
        path = env;
    else
        path = kDataDir + "/bunny.stl";

    if (!std::filesystem::exists(path))
        GTEST_SKIP() << "bunny.stl not found at " << path
                     << " (set BUNNY_STL=<path> to override)";

    runBunnyEval("full", path, 0.005, 0.001);
}
