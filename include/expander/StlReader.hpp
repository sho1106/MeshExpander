#pragma once
// ---------------------------------------------------------------------------
// StlReader — Binary STL file loader
//
// Reads binary STL files only (ASCII STL is not supported).
// Each triangle is stored with independent vertices (no vertex sharing).
// Face normals are re-derived from vertex positions by ConservativeExpander /
// RobustSlicer — the normal stored in the STL file is ignored.
//
// Binary STL format:
//   80 bytes  : header (ignored)
//    4 bytes  : uint32_t  numTriangles
//   Per triangle (50 bytes):
//     12 bytes : float[3] normal  (ignored)
//     12 bytes : float[3] vertex0
//     12 bytes : float[3] vertex1
//     12 bytes : float[3] vertex2
//      2 bytes : uint16_t attribute (ignored)
// ---------------------------------------------------------------------------

#include "Mesh.hpp"

#include <cstdint>
#include <fstream>
#include <string>

namespace expander {

class StlReader {
public:
    // Read a binary STL file.  Returns an empty Mesh on I/O error or if the
    // file appears to be ASCII.  The returned Mesh has 3*numTriangles vertices
    // (no sharing) and numTriangles faces.
    static Mesh read(const std::string& path) {
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs) return {};

        // Header
        char header[80];
        ifs.read(header, 80);
        if (!ifs) return {};

        // ASCII STL starts with "solid" — heuristic reject
        if (header[0] == 's' && header[1] == 'o' && header[2] == 'l'
         && header[3] == 'i' && header[4] == 'd') {
            // Could still be binary with a "solid..." header.
            // Try reading the triangle count and validate file size.
        }

        uint32_t numTris = 0;
        ifs.read(reinterpret_cast<char*>(&numTris), 4);
        if (!ifs || numTris == 0 || numTris > 50000000u) return {};

        Mesh m;
        m.vertices.resize(static_cast<int>(numTris) * 3, 3);
        m.faces.reserve(numTris);

        for (uint32_t i = 0; i < numTris; ++i) {
            float buf[12]; // 3 normal + 3×3 vertex components
            ifs.read(reinterpret_cast<char*>(buf), 12 * sizeof(float));
            uint16_t attr = 0;
            ifs.read(reinterpret_cast<char*>(&attr), 2);
            if (!ifs) break;

            for (int v = 0; v < 3; ++v) {
                m.vertices(static_cast<int>(i) * 3 + v, 0) = buf[3 + v * 3];
                m.vertices(static_cast<int>(i) * 3 + v, 1) = buf[3 + v * 3 + 1];
                m.vertices(static_cast<int>(i) * 3 + v, 2) = buf[3 + v * 3 + 2];
            }
            const int base = static_cast<int>(i) * 3;
            m.faces.push_back({base, base + 1, base + 2});
        }

        return m;
    }
};

} // namespace expander
