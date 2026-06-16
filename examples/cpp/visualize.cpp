// visualize.cpp — MeshExpander visualization sample (C++ / Open3D)
//
// Build:
//   cmake -S examples/cpp -B examples/cpp/build \
//         -DCMAKE_BUILD_TYPE=Release \
//         -DMeshExpander_SOURCE=<repo> -DMeshExpander_BUILD=<build_dir> \
//         -DOpen3D_DIR=<open3d_install>/lib/cmake/Open3D
//   cmake --build examples/cpp/build --config Release
//
// Run:
//   ./visualize model.stl --d 1.0
//   ./visualize model.stl --d 1.0 --max-pieces 8 --side-by-side

#include "expander/AssemblyExpander.hpp"
#include "expander/StlReader.hpp"
#include "expander/StlWriter.hpp"

#include <open3d/Open3D.h>

#include <cstring>
#include <iostream>
#include <string>

// ---------------------------------------------------------------------------
// expander::Mesh → open3d::geometry::TriangleMesh
// ---------------------------------------------------------------------------
static std::shared_ptr<open3d::geometry::TriangleMesh>
toO3D(const expander::Mesh& src)
{
    auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();

    mesh->vertices_.reserve(src.numVertices());
    for (int i = 0; i < src.numVertices(); ++i)
        mesh->vertices_.emplace_back(
            src.vertices(i, 0), src.vertices(i, 1), src.vertices(i, 2));

    mesh->triangles_.reserve(src.faces.size());
    for (const auto& f : src.faces)
        mesh->triangles_.emplace_back(f[0], f[1], f[2]);

    mesh->ComputeVertexNormals();
    return mesh;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: visualize <input.stl> [--d <dist>] "
                     "[--max-pieces <n>] [--side-by-side]\n";
        return 1;
    }

    std::string inputPath  = argv[1];
    double      d          = 1.0;
    int         maxPieces  = 1;     // >1 enables concave box decomposition
    bool        sideBySide = false;

    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--d") == 0 && i + 1 < argc)
            d = std::stod(argv[++i]);
        else if (std::strcmp(argv[i], "--max-pieces") == 0 && i + 1 < argc)
            maxPieces = std::stoi(argv[++i]);
        else if (std::strcmp(argv[i], "--side-by-side") == 0)
            sideBySide = true;
    }

    // --- Load & expand ---
    std::cout << "Loading: " << inputPath << "\n";
    expander::Mesh srcMesh = expander::StlReader::read(inputPath);
    if (srcMesh.empty()) {
        std::cerr << "Failed to read STL (binary STL only): " << inputPath << "\n";
        return 1;
    }

    std::cout << "Expanding: d=" << d << ", max_pieces=" << maxPieces << " ...\n";
    expander::AssemblyExpander::Options opts;
    opts.maxConvexPieces = maxPieces;
    expander::AssemblyExpander exp(opts);
    expander::Mesh expMesh = exp.expandMerged({srcMesh}, d);
    std::cout << "Done\n";

    // --- Convert to Open3D ---
    auto origO3D = toO3D(srcMesh);
    auto expO3D  = toO3D(expMesh);

    origO3D->PaintUniformColor({0.2, 0.4, 0.9});   // original: blue
    expO3D->PaintUniformColor({0.2, 0.8, 0.3});    // expanded: green

    if (sideBySide) {
        auto bbox     = origO3D->GetAxisAlignedBoundingBox();
        double offset = (bbox.GetExtent()[0] + d * 5.0) * 1.3;
        origO3D->Translate({-offset, 0.0, 0.0});
        std::cout << "Side-by-side: original offset to X=" << -offset << "\n";
    }

    auto wireframe = open3d::geometry::LineSet::CreateFromTriangleMesh(*expO3D);
    wireframe->PaintUniformColor({0.0, 0.5, 0.0});

    double axisSize = std::max(d * 10.0, 0.01);
    auto axes = open3d::geometry::TriangleMesh::CreateCoordinateFrame(axisSize);

    std::cout << "\nControls:\n";
    std::cout << "  left-drag: rotate / right-drag: pan / scroll: zoom\n";
    std::cout << "  Q / Esc: quit\n";

    open3d::visualization::DrawGeometries(
        {origO3D, expO3D, wireframe, axes},
        "MeshExpander — blue: original  green: expanded",
        1280, 720);

    return 0;
}
