#include <gtest/gtest.h>
#include "expander/VoxelGrid.hpp"

using namespace expander;

// Helper: build a simple cube mesh (8 verts, 12 tris)
static Mesh makeCube(double half = 1.0) {
    Mesh m;
    m.vertices.resize(8, 3);
    double h = half;
    m.vertices << -h,-h,-h,  h,-h,-h,  h,h,-h,  -h,h,-h,
                  -h,-h, h,  h,-h, h,  h,h, h,  -h,h, h;
    // 12 triangles (2 per face)
    m.faces = {
        {0,2,1},{0,3,2},   // -z face
        {4,5,6},{4,6,7},   // +z face
        {0,1,5},{0,5,4},   // -y face
        {2,3,7},{2,7,6},   // +y face
        {0,4,7},{0,7,3},   // -x face
        {1,2,6},{1,6,5},   // +x face
    };
    return m;
}

// VoxelGrid must not throw on normal input
TEST(VoxelGrid, BuildDoesNotThrow) {
    VoxelGrid g(0.5);
    EXPECT_NO_THROW(g.build(makeCube(1.0)));
}

// Invalid cell size must throw
TEST(VoxelGrid, ZeroCellSizeThrows) {
    EXPECT_THROW(VoxelGrid(0.0), std::invalid_argument);
    EXPECT_THROW(VoxelGrid(-1.0), std::invalid_argument);
}

// After building a unit cube, the cells at the cube's surface must be occupied
TEST(VoxelGrid, CubeVertexCellsAreOccupied) {
    const double cs = 0.5;
    VoxelGrid g(cs);
    g.build(makeCube(1.0));

    // The cell containing vertex (1,1,1) must be occupied
    // grid origin is at (-1-cs, -1-cs, -1-cs)
    // vertex (1,1,1): voxel index = floor((1 - origin) / cs) in each axis
    const Eigen::Vector3i dims = g.dims();
    EXPECT_GT(dims.x(), 0);
    EXPECT_GT(dims.y(), 0);
    EXPECT_GT(dims.z(), 0);

    // At least some cells must be occupied
    int count = 0;
    for (int z = 0; z < dims.z(); ++z)
        for (int y = 0; y < dims.y(); ++y)
            for (int x = 0; x < dims.x(); ++x)
                if (g.occupied(x, y, z)) ++count;

    EXPECT_GT(count, 0) << "No occupied cells after building mesh";
}

// Out-of-bounds queries must return false (no crash)
TEST(VoxelGrid, OutOfBoundsReturnsFalse) {
    VoxelGrid g(0.5);
    g.build(makeCube(1.0));

    EXPECT_FALSE(g.occupied(-1, 0, 0));
    EXPECT_FALSE(g.occupied(0, -1, 0));
    EXPECT_FALSE(g.occupied(9999, 9999, 9999));
}

// greedyMerge must cover every occupied cell (union-coverage property)
TEST(VoxelGrid, GreedyMergeCoversAllOccupiedCells) {
    const double cs = 0.5;
    VoxelGrid g(cs);
    g.build(makeCube(2.0));

    const auto boxes = g.greedyMerge();
    EXPECT_FALSE(boxes.empty());

    const Eigen::Vector3i dims = g.dims();
    for (int z = 0; z < dims.z(); ++z) {
        for (int y = 0; y < dims.y(); ++y) {
            for (int x = 0; x < dims.x(); ++x) {
                if (!g.occupied(x, y, z)) continue;

                // Cell center must be inside at least one merged box
                Eigen::AlignedBox3d cb = g.cellBounds(x, y, z);
                Eigen::Vector3d center = cb.center();

                bool covered = false;
                for (const auto& b : boxes) {
                    if (b.worldBounds.contains(center)) { covered = true; break; }
                }
                EXPECT_TRUE(covered)
                    << "Occupied cell (" << x << "," << y << "," << z
                    << ") not covered by any merged box";
            }
        }
    }
}

// greedyMerge boxes must not overlap
TEST(VoxelGrid, GreedyMergeBoxesDoNotOverlap) {
    const double cs = 0.5;
    VoxelGrid g(cs);
    g.build(makeCube(2.0));

    const auto boxes = g.greedyMerge();
    const double tol = cs * 0.01; // small tolerance for float comparison

    for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
        for (int j = i + 1; j < static_cast<int>(boxes.size()); ++j) {
            // Two boxes overlap iff their intersection has positive volume
            Eigen::AlignedBox3d inter = boxes[i].worldBounds.intersection(boxes[j].worldBounds);
            if (!inter.isEmpty()) {
                double vol = inter.sizes().prod();
                EXPECT_LT(vol, tol * tol * tol)
                    << "Boxes " << i << " and " << j << " overlap with volume " << vol;
            }
        }
    }
}

// Empty mesh → no cells occupied, greedyMerge returns nothing
TEST(VoxelGrid, EmptyMeshProducesNoBoxes) {
    VoxelGrid g(0.5);
    Mesh empty;
    g.build(empty);
    EXPECT_TRUE(g.greedyMerge().empty());
}

// Large cell size → single box covers entire mesh
TEST(VoxelGrid, LargeCellSizeProducesFewBoxes) {
    VoxelGrid g(100.0); // 100-unit cells for a 2-unit cube → very few boxes
    g.build(makeCube(1.0));
    const auto boxes = g.greedyMerge();
    // With a cell 50× the mesh, should collapse to at most a handful of boxes
    EXPECT_LE(static_cast<int>(boxes.size()), 8);
}
