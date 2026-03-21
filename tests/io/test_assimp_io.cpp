// ---------------------------------------------------------------------------
// test_assimp_io.cpp — integration tests for AssimpLoader / AssimpExporter
//
// Requires: MESHEXPANDER_BUILD_IO=ON
//
// Strategy: generate meshes in-process → write STL → load via Assimp →
//           expand → export via Assimp → verify file exists and is non-empty.
//           No external asset files required.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>
#include <filesystem>
#include <algorithm>

#include "expander/AssemblyExpander.hpp"
#include "expander/StlWriter.hpp"
#include "io/AssimpLoader.hpp"
#include "io/AssimpExporter.hpp"

namespace fs = std::filesystem;
using namespace expander;
using namespace expander::io;

// ── Geometry helpers ──────────────────────────────────────────────────────────

static Mesh makeCube(double half, Eigen::Vector3d offset = Eigen::Vector3d::Zero()) {
    Mesh m;
    m.vertices.resize(8, 3);
    // clang-format off
    m.vertices <<
        -half+offset.x(), -half+offset.y(), -half+offset.z(),
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

static std::string tmpPath(const std::string& name) {
    return (fs::temp_directory_path() / name).string();
}

// ── Tests ─────────────────────────────────────────────────────────────────────

// ---------------------------------------------------------------------------
// Load STL: AssimpLoader round-trip preserves geometry
// ---------------------------------------------------------------------------
TEST(AssimpIO, LoadSTL_GeometryPreserved) {
    const std::string path = tmpPath("me_io_cube.stl");
    Mesh cube = makeCube(2.0);
    StlWriter::write(path, cube);

    AssimpLoader loader;
    auto parts = loader.load(path);

    ASSERT_FALSE(parts.empty()) << "AssimpLoader returned no parts";
    EXPECT_GT(parts[0].numVertices(), 0);
    EXPECT_GT(parts[0].numFaces(),    0);

    // Bounding box should match source (within float precision)
    const Eigen::Vector3d srcMin = cube.vertices.colwise().minCoeff().transpose();
    const Eigen::Vector3d srcMax = cube.vertices.colwise().maxCoeff().transpose();
    const Eigen::Vector3d dstMin = parts[0].vertices.colwise().minCoeff().transpose();
    const Eigen::Vector3d dstMax = parts[0].vertices.colwise().maxCoeff().transpose();
    EXPECT_NEAR((srcMin - dstMin).norm(), 0.0, 1e-4) << "Bounding box min mismatch";
    EXPECT_NEAR((srcMax - dstMax).norm(), 0.0, 1e-4) << "Bounding box max mismatch";

    fs::remove(path);
}

// ---------------------------------------------------------------------------
// Export OBJ: non-empty, can be reloaded
// ---------------------------------------------------------------------------
TEST(AssimpIO, ExportOBJ_NonEmptyAndReloadable) {
    const std::string stlIn  = tmpPath("me_io_export_src.stl");
    const std::string objOut = tmpPath("me_io_export_result.obj");
    StlWriter::write(stlIn, makeCube(2.0));

    AssimpLoader loader;
    AssimpExporter exporter;

    auto parts = loader.load(stlIn);
    ASSERT_FALSE(parts.empty());

    AssemblyExpander exp;
    Mesh result = exp.expandMerged(parts, 0.1);
    ASSERT_NO_THROW(exporter.write(objOut, {result}));

    EXPECT_TRUE(fs::exists(objOut))             << "OBJ file not created";
    EXPECT_GT(fs::file_size(objOut), 0u)        << "OBJ file is empty";

    // Reload the OBJ and verify it has geometry
    auto reloaded = loader.load(objOut);
    ASSERT_FALSE(reloaded.empty());
    EXPECT_GT(reloaded[0].numVertices(), 0);
    EXPECT_GT(reloaded[0].numFaces(),    0);

    fs::remove(stlIn);
    fs::remove(objOut);
}

// ---------------------------------------------------------------------------
// Multi-part: load two separate STLs, expand per-part, export merged OBJ
// ---------------------------------------------------------------------------
TEST(AssimpIO, MultiPart_TwoParts_ExpandAndExport) {
    const std::string stlA = tmpPath("me_io_partA.stl");
    const std::string stlB = tmpPath("me_io_partB.stl");
    const std::string out  = tmpPath("me_io_merged.obj");

    StlWriter::write(stlA, makeCube(3.0, Eigen::Vector3d(-10, 0, 0)));
    StlWriter::write(stlB, makeCube(2.0, Eigen::Vector3d( 10, 0, 0)));

    AssimpLoader loader;
    auto pa = loader.load(stlA);
    auto pb = loader.load(stlB);
    ASSERT_FALSE(pa.empty());
    ASSERT_FALSE(pb.empty());

    std::vector<Mesh> parts;
    parts.insert(parts.end(), pa.begin(), pa.end());
    parts.insert(parts.end(), pb.begin(), pb.end());

    AssemblyExpander exp;
    Mesh result = exp.expandMerged(parts, 0.2);

    AssimpExporter exporter;
    ASSERT_NO_THROW(exporter.write(out, {result}));
    EXPECT_TRUE(fs::exists(out));
    EXPECT_GT(fs::file_size(out), 0u);

    fs::remove(stlA);
    fs::remove(stlB);
    fs::remove(out);
}

// ---------------------------------------------------------------------------
// mergeContained + export: nested cubes absorbed, single-part output
// ---------------------------------------------------------------------------
TEST(AssimpIO, MergeContained_ExportSinglePart) {
    const std::string stlOuter = tmpPath("me_io_outer.stl");
    const std::string stlInner = tmpPath("me_io_inner.stl");
    const std::string objOut   = tmpPath("me_io_nested.obj");

    StlWriter::write(stlOuter, makeCube(5.0));
    StlWriter::write(stlInner, makeCube(1.0));  // fully inside outer

    AssimpLoader loader;
    auto outer = loader.load(stlOuter);
    auto inner = loader.load(stlInner);
    ASSERT_FALSE(outer.empty());
    ASSERT_FALSE(inner.empty());

    std::vector<Mesh> parts;
    parts.insert(parts.end(), outer.begin(), outer.end());
    parts.insert(parts.end(), inner.begin(), inner.end());

    auto merged = AssemblyExpander::mergeContained(parts);
    EXPECT_EQ(merged.size(), 1u) << "Inner cube should be absorbed into outer";

    AssemblyExpander exp;
    Mesh result = exp.expandMerged(merged, 0.1);

    AssimpExporter exporter;
    ASSERT_NO_THROW(exporter.write(objOut, {result}));
    EXPECT_TRUE(fs::exists(objOut));

    fs::remove(stlOuter);
    fs::remove(stlInner);
    fs::remove(objOut);
}

// ---------------------------------------------------------------------------
// listFormats: OBJ and STL must be available
// ---------------------------------------------------------------------------
TEST(AssimpIO, ListFormats_ContainsOBJAndSTL) {
    auto formats = AssimpExporter::listFormats();
    EXPECT_FALSE(formats.empty());

    auto hasFormat = [&](const std::string& ext) {
        return std::find(formats.begin(), formats.end(), ext) != formats.end();
    };
    EXPECT_TRUE(hasFormat("obj")) << "OBJ not in supported export formats";
    EXPECT_TRUE(hasFormat("stl")) << "STL not in supported export formats";
}
