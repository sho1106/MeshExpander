// Unit tests for StlReader — binary roundtrip + ASCII/corrupt rejection.
#include "expander/StlReader.hpp"
#include "expander/StlWriter.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

using namespace expander;

namespace {

// A small two-triangle quad mesh.
Mesh makeQuad() {
    Mesh m;
    m.vertices.resize(4, 3);
    m.vertices << 0, 0, 0,
                  1, 0, 0,
                  1, 1, 0,
                  0, 1, 0;
    m.faces = {{0, 1, 2}, {0, 2, 3}};
    return m;
}

std::string tmpPath(const char* name) {
    return std::string(::testing::TempDir()) + "/" + name;
}

} // namespace

TEST(StlReader, BinaryRoundtrip) {
    const std::string path = tmpPath("me_quad.stl");
    StlWriter::write(path, makeQuad());

    Mesh m = StlReader::read(path);
    // Binary STL stores triangles with independent (unshared) vertices.
    EXPECT_EQ(m.numFaces(), 2);
    EXPECT_EQ(m.numVertices(), 6);
    EXPECT_FALSE(m.empty());
    std::remove(path.c_str());
}

TEST(StlReader, RejectsAsciiStl) {
    const std::string path = tmpPath("me_ascii.stl");
    {
        std::ofstream ofs(path);
        ofs << "solid quad\n"
               "  facet normal 0 0 1\n"
               "    outer loop\n"
               "      vertex 0 0 0\n"
               "      vertex 1 0 0\n"
               "      vertex 1 1 0\n"
               "    endloop\n"
               "  endfacet\n"
               "endsolid quad\n";
    }
    Mesh m = StlReader::read(path);
    EXPECT_TRUE(m.empty()) << "ASCII STL must be rejected, not parsed as garbage";
    std::remove(path.c_str());
}

TEST(StlReader, RejectsMissingFile) {
    Mesh m = StlReader::read(tmpPath("does_not_exist_me.stl"));
    EXPECT_TRUE(m.empty());
}

TEST(StlReader, RejectsTruncatedBinary) {
    // Valid header + count claiming 2 triangles but no triangle data → size mismatch.
    const std::string path = tmpPath("me_trunc.stl");
    {
        std::ofstream ofs(path, std::ios::binary);
        char hdr[80] = {};
        ofs.write(hdr, 80);
        uint32_t n = 2;
        ofs.write(reinterpret_cast<const char*>(&n), 4);
        // intentionally omit the 2*50 bytes of triangle payload
    }
    Mesh m = StlReader::read(path);
    EXPECT_TRUE(m.empty()) << "size-mismatched binary STL must be rejected";
    std::remove(path.c_str());
}
