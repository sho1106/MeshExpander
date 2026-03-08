#pragma once
// ---------------------------------------------------------------------------
// StlWriter — Binary STL 出力ユーティリティ (Assimp 不要)
//
// Binary STL フォーマット:
//   80 bytes  ヘッダ (任意の ASCII 文字列)
//    4 bytes  三角形数 (uint32_t, little-endian)
//   Per triangle (50 bytes):
//     12 bytes  法線ベクトル (3× float32)
//     36 bytes  3頂点 (3× 3× float32)
//      2 bytes  属性 (0)
// ---------------------------------------------------------------------------

#include "Mesh.hpp"

#include <Eigen/Dense>
#include <fstream>
#include <stdexcept>
#include <string>
#include <cstring>
#include <cstdint>

namespace expander {

class StlWriter {
public:
    // path へ Binary STL を書き出す。
    // 法線は各三角形の辺ベクトルから自動計算する (正規化済み)。
    static void write(const std::string& path, const Mesh& mesh,
                      const std::string& header = "MeshExpander output") {
        if (mesh.faces.empty())
            throw std::runtime_error("StlWriter: mesh has no faces");

        std::ofstream ofs(path, std::ios::binary);
        if (!ofs)
            throw std::runtime_error("StlWriter: cannot open file: " + path);

        // ---- 80 byte header ----
        char hdr[80] = {};
        std::size_t copyLen = std::min(header.size(), std::size_t(79));
        std::memcpy(hdr, header.data(), copyLen);
        ofs.write(hdr, 80);

        // ---- triangle count ----
        uint32_t nTri = static_cast<uint32_t>(mesh.faces.size());
        ofs.write(reinterpret_cast<const char*>(&nTri), 4);

        // ---- triangles ----
        const uint16_t attr = 0;
        for (const auto& f : mesh.faces) {
            Eigen::Vector3f v0 = mesh.vertices.row(f[0]).cast<float>();
            Eigen::Vector3f v1 = mesh.vertices.row(f[1]).cast<float>();
            Eigen::Vector3f v2 = mesh.vertices.row(f[2]).cast<float>();

            Eigen::Vector3f n = (v1 - v0).cross(v2 - v0);
            float len = n.norm();
            if (len > 1e-10f) n /= len;

            writeF3(ofs, n);
            writeF3(ofs, v0);
            writeF3(ofs, v1);
            writeF3(ofs, v2);
            ofs.write(reinterpret_cast<const char*>(&attr), 2);
        }
    }

private:
    static void writeF3(std::ofstream& ofs, const Eigen::Vector3f& v) {
        float buf[3] = {v.x(), v.y(), v.z()};
        ofs.write(reinterpret_cast<const char*>(buf), 12);
    }
};

} // namespace expander
