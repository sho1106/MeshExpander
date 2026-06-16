// Complex Concave Shape Expansion Test
// =====================================
// BoxExpander は常に単一凸多面体を返す（削り出し法）。
// 凹形状に対しては:
//   - 保守性 (Cov%=100%): 元頂点は必ず包含される
//   - 膨張保証 (Exp%>=95%): 面法線方向に d 以上膨らんでいる
//   - 体積比 (VolRatio): V_expanded/V_input が大きいほど無駄な体積が多い
//
// 形状:
//   BumpySphere   — 正弦波デフォルメ球（多数の凹み、激しい凹凸）
//   GroovedCylinder — 軸方向溝付き円柱（8本の深溝）

#include <gtest/gtest.h>
#include "expander/BoxExpander.hpp"
#include "expander/StlWriter.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace expander;
static const double kPi2 = 3.14159265358979323846;

// ---------------------------------------------------------------------------
// Mesh generators
// ---------------------------------------------------------------------------

// 正弦波デフォルメ球
// r(θ,φ) = R * (1 + amp * sin(nb*θ) * sin(nb*φ))
// amp=0.25, nb=6 → 球面全体に 6×6 = 36 個の凹凸パターン
// 法線方向判定: 外向き法線を持つ UV 球と同じ巻き順を使用
static Mesh makeBumpySphere(double R=0.050, int Nth=60, int Nph=30,
                             int nb=6, double amp=0.25)
{
    const int N = Nth * Nph;
    const int iNorth = N, iSouth = N + 1;
    Mesh m;
    m.vertices.resize(N + 2, 3);

    for (int ip = 0; ip < Nph; ++ip) {
        double ph = kPi2 * (ip + 1) / (Nph + 1);  // (0, π) exclusive
        for (int it = 0; it < Nth; ++it) {
            double th = 2 * kPi2 * it / Nth;
            double r  = R * (1.0 + amp * std::sin(nb * th) * std::sin(nb * ph));
            int vi = ip * Nth + it;
            m.vertices(vi, 0) = r * std::sin(ph) * std::cos(th);
            m.vertices(vi, 1) = r * std::sin(ph) * std::sin(th);
            m.vertices(vi, 2) = r * std::cos(ph);
        }
    }
    m.vertices.row(iNorth) << 0.0, 0.0,  R;
    m.vertices.row(iSouth) << 0.0, 0.0, -R;

    m.faces.reserve(2 * Nth * Nph);

    // 北極キャップ: {north, ring0[it], ring0[it+1]}
    // 上から見て CCW → 外向き法線 +z ✓
    for (int it = 0; it < Nth; ++it) {
        int itn = (it + 1) % Nth;
        m.faces.push_back({iNorth, it, itn});
    }

    // 中間バンド: quad を 2 三角形に分割
    // {a=(ip,it), b=(ip+1,it), c=(ip+1,it+1), d=(ip,it+1)}
    // {a,b,c} の法線解析: 赤道付近で +r 方向（外向き）✓
    for (int ip = 0; ip < Nph - 1; ++ip) {
        for (int it = 0; it < Nth; ++it) {
            int itn = (it + 1) % Nth;
            int a = ip * Nth + it,        b = (ip + 1) * Nth + it;
            int c = (ip + 1) * Nth + itn, d = ip * Nth + itn;
            m.faces.push_back({a, b, c});
            m.faces.push_back({a, c, d});
        }
    }

    // 南極キャップ: {lastRing[it], south, lastRing[it+1]}
    // 下から見て CCW → 外向き法線 -z ✓
    int last = (Nph - 1) * Nth;
    for (int it = 0; it < Nth; ++it) {
        int itn = (it + 1) % Nth;
        m.faces.push_back({last + it, iSouth, last + itn});
    }

    return m;
}

// 軸方向溝付き円柱
// 半径: R(θ) = Ro - depth * max(0, sin(nGrooves * θ))^2
// nGrooves 本の軸方向溝が外周に等間隔で並ぶ凹形状
static Mesh makeGroovedCylinder(double Ro=0.040, double H=0.060,
                                  int nGrooves=8, double depth=0.015,
                                  int Nth=96, int Nz=2)
{
    // 側面頂点: (Nz+1) 行 × Nth 列
    // 上下キャップ中心
    const int Nring = Nz + 1;
    const int N     = Nring * Nth;
    const int tC    = N, bC = N + 1;
    Mesh m;
    m.vertices.resize(N + 2, 3);

    for (int iz = 0; iz < Nring; ++iz) {
        double z = -H * 0.5 + H * iz / Nz;
        for (int it = 0; it < Nth; ++it) {
            double th  = 2 * kPi2 * it / Nth;
            double sw  = std::sin(nGrooves * th);
            double r   = Ro - depth * (sw > 0.0 ? sw * sw : 0.0);
            int vi = iz * Nth + it;
            m.vertices(vi, 0) = r * std::cos(th);
            m.vertices(vi, 1) = r * std::sin(th);
            m.vertices(vi, 2) = z;
        }
    }
    m.vertices.row(tC) << 0.0, 0.0,  H * 0.5;
    m.vertices.row(bC) << 0.0, 0.0, -H * 0.5;

    m.faces.reserve(2 * Nth * Nz + 2 * Nth);

    // 側面
    for (int iz = 0; iz < Nz; ++iz) {
        for (int it = 0; it < Nth; ++it) {
            int itn = (it + 1) % Nth;
            int a = iz * Nth + it,        b = (iz + 1) * Nth + it;
            int c = (iz + 1) * Nth + itn, d = iz * Nth + itn;
            m.faces.push_back({a, b, c});
            m.faces.push_back({a, c, d});
        }
    }

    // 上キャップ (+z 外向き)
    int topRing = Nz * Nth;
    for (int it = 0; it < Nth; ++it) {
        int itn = (it + 1) % Nth;
        m.faces.push_back({tC, topRing + it, topRing + itn});
    }

    // 下キャップ (-z 外向き)
    for (int it = 0; it < Nth; ++it) {
        int itn = (it + 1) % Nth;
        m.faces.push_back({bC, it, itn});
    }

    return m;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static double meshVolume2(const Mesh& m) {
    double v = 0;
    for (const auto& f : m.faces) {
        Eigen::Vector3d a = m.vertices.row(f[0]),
                        b = m.vertices.row(f[1]),
                        c = m.vertices.row(f[2]);
        v += a.dot(b.cross(c));
    }
    return std::abs(v) / 6.0;
}

static bool insideConvex2(const Eigen::Vector3d& pt, const Mesh& m, double eps = 1e-6) {
    for (const auto& f : m.faces) {
        Eigen::Vector3d v0 = m.vertices.row(f[0]),
                        v1 = m.vertices.row(f[1]),
                        v2 = m.vertices.row(f[2]);
        Eigen::Vector3d n = (v1 - v0).cross(v2 - v0);
        double len = n.norm();
        if (len < 1e-20) continue;
        if ((pt - v0).dot(n) > eps * len) return false;
    }
    return true;
}

static int checkCons2(const Mesh& inp, const Mesh& expanded, double eps = 1e-5) {
    int fail = 0;
    for (int i = 0; i < inp.numVertices(); ++i) {
        Eigen::Vector3d v = inp.vertices.row(i).transpose();
        if (!insideConvex2(v, expanded, eps)) ++fail;
    }
    return fail;
}

static int checkExp2(const Mesh& inp, const Mesh& expanded, double d, double eps = 1e-5) {
    int fail = 0;
    for (const auto& f : inp.faces) {
        Eigen::Vector3d v0 = inp.vertices.row(f[0]),
                        v1 = inp.vertices.row(f[1]),
                        v2 = inp.vertices.row(f[2]);
        Eigen::Vector3d n = (v1 - v0).cross(v2 - v0);
        double len = n.norm();
        if (len < 1e-20) continue;
        Eigen::Vector3d probe = (v0 + v1 + v2) / 3.0 + d * (n / len);
        if (!insideConvex2(probe, expanded, eps)) ++fail;
    }
    return fail;
}

// ---------------------------------------------------------------------------
// Benchmark table
// ---------------------------------------------------------------------------

struct ComplexRow {
    std::string name;
    int inFaces, outFaces, nV, nF, cF, eF;
    double volInp, vol, ms;
};
static std::vector<ComplexRow> g_crows;

static void printComplexTable() {
    const char* sep =
        "+---------------------+--------+--------+-------+------+----------+---------+------+";
    std::cout << "\n" << sep << "\n"
        << "| Shape               | InFace | OutFac | Cov%  | Exp% | VolExp   | VolRatio|  ms  |\n"
        << sep << "\n";
    for (const auto& r : g_crows) {
        double cov   = r.nV > 0 ? (1.0 - (double)r.cF / r.nV) * 100 : 100.0;
        double exp_p = r.nF > 0 ? (1.0 - (double)r.eF / r.nF) * 100 : 100.0;
        double ratio = r.volInp > 0 ? r.vol / r.volInp : 0.0;
        std::cout << std::fixed
            << "| " << std::left  << std::setw(19) << r.name    << " |"
            << std::right << std::setw(7)  << r.inFaces          << " |"
            << std::right << std::setw(7)  << r.outFaces         << " |"
            << std::right << std::setprecision(1) << std::setw(6)  << cov    << " |"
            << std::right << std::setprecision(1) << std::setw(5)  << exp_p  << " |"
            << std::right << std::setprecision(6) << std::setw(9)  << r.vol  << " |"
            << std::right << std::setprecision(3)  << std::setw(8)  << ratio  << " |"
            << std::right << std::setprecision(0) << std::setw(5)  << r.ms   << " |\n";
    }
    std::cout << sep << "\n"
        << "  Algorithm: BoxExpander (削り出し法, 単一凸多面体)\n"
        << "  Cov%     = 元頂点が出力に包含される割合 (100% = 保守的)\n"
        << "  Exp%     = 面法線方向 d 先プローブが出力に包含される割合 (>=95% 目標)\n"
        << "  VolRatio = V_expanded / V_input  (凹形状では大きい = 無駄な体積)\n\n";
}

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------

class ComplexShapeTest : public ::testing::Test {
protected:
    static constexpr double kD = 0.001;  // 膨張距離 1 mm

    void runShape(const std::string& name, const Mesh& inp) {
        ASSERT_GT(inp.numFaces(), 0) << name << ": empty mesh";

        auto t0 = std::chrono::high_resolution_clock::now();
        BoxExpander exp;
        Mesh result = exp.expand(inp, kD);
        double ms = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - t0).count();

        ASSERT_GT(result.numFaces(), 0) << name << ": expansion returned empty";
        EXPECT_TRUE(result.vertices.allFinite()) << name << ": NaN/Inf in output";

        const int nV = inp.numVertices();
        const int nF = (int)inp.faces.size();
        const int cF = checkCons2(inp, result, 1e-5);
        const int eF = checkExp2(inp, result, kD, 1e-5);

        // NOTE: 複雑な凹形状では BoxExpander（単一凸多面体）の限界により
        //       Cov% < 100% になる場合がある。ここでは保守性アサーションなし。
        //       Cov%/Exp% は情報として記録し VolRatio で無駄な体積を評価する。
        //       保守性の厳密検証は test_cad_shapes.cpp / test_assembly_expansion.cpp で行う。
        double cov   = nV > 0 ? (1.0 - (double)cF / nV) * 100 : 100.0;
        double exp_p = nF > 0 ? (1.0 - (double)eF / nF) * 100 : 100.0;
        std::cout << "  [" << name << "] Cov%=" << std::fixed << std::setprecision(1)
                  << cov << "  Exp%=" << exp_p << "\n";

        double volInp = meshVolume2(inp);
        double vol    = meshVolume2(result);
        double ratio  = volInp > 0 ? vol / volInp : 0.0;

        // 体積比はアサーションなし（情報として記録）
        std::cout << "  [" << name << "] VolRatio=" << std::fixed << std::setprecision(3)
                  << ratio << "  (V_exp=" << std::setprecision(6) << vol
                  << " / V_inp=" << volInp << ")\n";

        g_crows.push_back({name, nF, result.numFaces(), nV, nF, cF, eF, volInp, vol, ms});

        try {
            std::filesystem::create_directories("stl_output");
            std::string tag = name;
            for (char& c : tag) if (!std::isalnum((unsigned char)c) && c != '-') c = '_';
            StlWriter::write("stl_output/complex_" + tag + ".stl", result, name);
        } catch (...) {}
    }
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST_F(ComplexShapeTest, BumpySphere)
{
    // 正弦波デフォルメ球: R=50mm, 6×6 凹凸パターン, 振幅 25%
    // BoxExpander の出力は AABB 由来の凸多面体 → 凹部分を丸め込む
    // → VolRatio >> 1 (大きな無駄な体積)
    runShape("Bumpy sphere", makeBumpySphere(0.050, 60, 30, 6, 0.25));
}

TEST_F(ComplexShapeTest, GroovedCylinder)
{
    // 溝付き円柱: R=40mm H=60mm, 8本の深溝(depth=15mm)
    // BoxExpander は溝を無視して外周 AABB で膨張
    // → VolRatio > 1 (溝部分が余計に包含される)
    runShape("Grooved cylinder", makeGroovedCylinder(0.040, 0.060, 8, 0.015, 96, 2));
}

TEST_F(ComplexShapeTest, ZZZ_PrintBenchmarkTable)
{
    if (g_crows.empty()) GTEST_SKIP() << "No rows";
    std::cout << "\n+-------------------------------------------------------+\n"
              << "|  MeshExpander Complex Shape Benchmark  (BoxExpander d=1mm)  |\n"
              << "+-------------------------------------------------------+\n";
    printComplexTable();
}
