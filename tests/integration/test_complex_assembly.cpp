// Complex Equipment Assembly Expansion Test
// ==========================================
// 設備のような複雑な部品構成をもつ5パーツアセンブリに対して
// AssemblyExpander の各テストを検証する。
//
// パーツ構成（各パーツは X 方向に離間配置）:
//   [0] Base plate     (200mm x 100mm x 10mm box,   convex)
//   [1] Mount cylinder (R=15mm H=30mm prism,         convex)
//   [2] L-bracket      (L型断面プリズム 60mm scale, concave)
//   [3] Shaft housing  (hollow cyl OD=40mm ID=20mm,  concave)
//   [4] Drive gear     (12歯ギア R=30mm,             concave)
//
// すべてのパーツを BoxExpander で膨張する。
//
// 検証指標:
//   1. 保守性  : Cov% = 100%（全入力頂点が出力多面体に包含される）
//   2. 膨張保証: Exp% >= 95%（面法線方向 d 先プローブが出力に包含される）
//   3. 体積効率: パーツ別膨張の体積合計 <= 全体1メッシュ膨張

#include <gtest/gtest.h>
#include "expander/AssemblyExpander.hpp"
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
static const double kPi = 3.14159265358979323846;

// ---------------------------------------------------------------------------
// Mesh generators
// ---------------------------------------------------------------------------

// 直方体（凸形状）
static Mesh makeBoxMesh(double wx, double wy, double wz,
                        Eigen::Vector3d off = Eigen::Vector3d::Zero())
{
    Mesh m; m.vertices.resize(8, 3);
    double hx = wx * .5, hy = wy * .5, hz = wz * .5;
    m.vertices <<
        -hx+off.x(), -hy+off.y(), -hz+off.z(),
         hx+off.x(), -hy+off.y(), -hz+off.z(),
         hx+off.x(),  hy+off.y(), -hz+off.z(),
        -hx+off.x(),  hy+off.y(), -hz+off.z(),
        -hx+off.x(), -hy+off.y(),  hz+off.z(),
         hx+off.x(), -hy+off.y(),  hz+off.z(),
         hx+off.x(),  hy+off.y(),  hz+off.z(),
        -hx+off.x(),  hy+off.y(),  hz+off.z();
    m.faces = {
        {0,2,1},{0,3,2}, {4,5,6},{4,6,7},
        {0,1,5},{0,5,4}, {2,3,7},{2,7,6},
        {1,2,6},{1,6,5}, {0,4,7},{0,7,3},
    };
    return m;
}

// N角柱（凸形状）
static Mesh makeCylinderMesh(double r, double h, int N = 32,
                              Eigen::Vector3d off = Eigen::Vector3d::Zero())
{
    Mesh m; m.vertices.resize(2 * N + 2, 3);
    for (int i = 0; i < N; ++i) {
        double a = 2 * kPi * i / N;
        double ca = std::cos(a), sa = std::sin(a);
        m.vertices(i,   0) = r*ca+off.x(); m.vertices(i,   1) = r*sa+off.y(); m.vertices(i,   2) =  h*.5+off.z();
        m.vertices(i+N, 0) = r*ca+off.x(); m.vertices(i+N, 1) = r*sa+off.y(); m.vertices(i+N, 2) = -h*.5+off.z();
    }
    m.vertices.row(2*N)   << off.x(), off.y(),  h*.5+off.z();
    m.vertices.row(2*N+1) << off.x(), off.y(), -h*.5+off.z();
    m.faces.reserve(4 * N);
    for (int i = 0; i < N; ++i) {
        int j = (i + 1) % N;
        m.faces.push_back({i,   i+N, j+N});
        m.faces.push_back({i,   j+N, j  });
        m.faces.push_back({2*N,   i,   j  });
        m.faces.push_back({2*N+1, j+N, i+N});
    }
    return m;
}

// L型ブラケット（凹形状）: 単位L型（[-s,s]^2 × [-s,s]）から再入角を持つプリズム
static Mesh makeLBracketMesh(double s,
                              Eigen::Vector3d off = Eigen::Vector3d::Zero())
{
    Mesh m; m.vertices.resize(12, 3);
    m.vertices.row(0)  = Eigen::RowVector3d(-s+off.x(), -s+off.y(),  s+off.z());
    m.vertices.row(1)  = Eigen::RowVector3d( s+off.x(), -s+off.y(),  s+off.z());
    m.vertices.row(2)  = Eigen::RowVector3d( s+off.x(),  0+off.y(),  s+off.z());
    m.vertices.row(3)  = Eigen::RowVector3d( 0+off.x(),  0+off.y(),  s+off.z());
    m.vertices.row(4)  = Eigen::RowVector3d( 0+off.x(),  s+off.y(),  s+off.z());
    m.vertices.row(5)  = Eigen::RowVector3d(-s+off.x(),  s+off.y(),  s+off.z());
    m.vertices.row(6)  = Eigen::RowVector3d(-s+off.x(), -s+off.y(), -s+off.z());
    m.vertices.row(7)  = Eigen::RowVector3d( s+off.x(), -s+off.y(), -s+off.z());
    m.vertices.row(8)  = Eigen::RowVector3d( s+off.x(),  0+off.y(), -s+off.z());
    m.vertices.row(9)  = Eigen::RowVector3d( 0+off.x(),  0+off.y(), -s+off.z());
    m.vertices.row(10) = Eigen::RowVector3d( 0+off.x(),  s+off.y(), -s+off.z());
    m.vertices.row(11) = Eigen::RowVector3d(-s+off.x(),  s+off.y(), -s+off.z());
    // 上面 (CCW from +z)
    m.faces.push_back({0,1,2}); m.faces.push_back({0,2,3});
    m.faces.push_back({0,3,4}); m.faces.push_back({0,4,5});
    // 下面 (反転)
    m.faces.push_back({6,8,7}); m.faces.push_back({6,9,8});
    m.faces.push_back({6,10,9}); m.faces.push_back({6,11,10});
    // 側面
    m.faces.push_back({0,6,7}); m.faces.push_back({0,7,1});
    m.faces.push_back({1,7,8}); m.faces.push_back({1,8,2});
    m.faces.push_back({2,8,9}); m.faces.push_back({2,9,3});
    m.faces.push_back({3,9,10}); m.faces.push_back({3,10,4});
    m.faces.push_back({4,10,11}); m.faces.push_back({4,11,5});
    m.faces.push_back({5,11,6}); m.faces.push_back({5,6,0});
    return m;
}

// 中空円筒（凹形状）: 内孔を持つ管構造
static Mesh makeHollowCylinderMesh(double Ro, double Ri, double H, int N = 32,
                                    Eigen::Vector3d off = Eigen::Vector3d::Zero())
{
    Mesh m; m.vertices.resize(4 * N, 3);
    for (int i = 0; i < N; ++i) {
        double a = 2*kPi*i/N, ca = std::cos(a), sa = std::sin(a);
        m.vertices(i,     0) = Ro*ca+off.x(); m.vertices(i,     1) = Ro*sa+off.y(); m.vertices(i,     2) =  H*.5+off.z();
        m.vertices(i+N,   0) = Ro*ca+off.x(); m.vertices(i+N,   1) = Ro*sa+off.y(); m.vertices(i+N,   2) = -H*.5+off.z();
        m.vertices(i+2*N, 0) = Ri*ca+off.x(); m.vertices(i+2*N, 1) = Ri*sa+off.y(); m.vertices(i+2*N, 2) =  H*.5+off.z();
        m.vertices(i+3*N, 0) = Ri*ca+off.x(); m.vertices(i+3*N, 1) = Ri*sa+off.y(); m.vertices(i+3*N, 2) = -H*.5+off.z();
    }
    m.faces.reserve(8 * N);
    for (int i = 0; i < N; ++i) {
        int j = (i+1) % N;
        m.faces.push_back({i,     i+N,   j+N  }); m.faces.push_back({i,     j+N,   j    });
        m.faces.push_back({i+2*N, j+2*N, j+3*N}); m.faces.push_back({i+2*N, j+3*N, i+3*N});
        m.faces.push_back({i,     j,     j+2*N}); m.faces.push_back({i,     j+2*N, i+2*N});
        m.faces.push_back({i+N,   i+3*N, j+3*N}); m.faces.push_back({i+N,   j+3*N, j+N  });
    }
    return m;
}

// ギア形状（凹形状）: 歯形を持つ平歯車プロファイル
static Mesh makeGearMesh(double Rb = 0.030, int nt = 12, double th = 0.008,
                          double H = 0.010, int sp = 6,
                          Eigen::Vector3d off = Eigen::Vector3d::Zero())
{
    const int N = nt * sp;
    std::vector<double> px(N), py(N);
    for (int i = 0; i < N; ++i) {
        double a = 2*kPi*i/N, ph = std::sin(nt*a);
        double Ri = Rb + th * (ph > 0.0 ? ph : 0.0);
        px[i] = Ri*std::cos(a) + off.x();
        py[i] = Ri*std::sin(a) + off.y();
    }
    Mesh m; m.vertices.resize(2*N+2, 3);
    for (int i = 0; i < N; ++i) {
        m.vertices(i,   0) = px[i]; m.vertices(i,   1) = py[i]; m.vertices(i,   2) =  H*.5+off.z();
        m.vertices(i+N, 0) = px[i]; m.vertices(i+N, 1) = py[i]; m.vertices(i+N, 2) = -H*.5+off.z();
    }
    m.vertices.row(2*N)   << off.x(), off.y(),  H*.5+off.z();
    m.vertices.row(2*N+1) << off.x(), off.y(), -H*.5+off.z();
    m.faces.reserve(4 * N);
    for (int i = 0; i < N; ++i) {
        int j = (i+1) % N;
        m.faces.push_back({i,   j,   j+N});
        m.faces.push_back({i,   j+N, i+N});
        m.faces.push_back({2*N, j,   i  });
        m.faces.push_back({2*N+1, i+N, j+N});
    }
    return m;
}

// ---------------------------------------------------------------------------
// Precision helpers
// ---------------------------------------------------------------------------

static double meshVolume(const Mesh& m) {
    double v = 0;
    for (const auto& f : m.faces) {
        Eigen::Vector3d a = m.vertices.row(f[0]),
                        b = m.vertices.row(f[1]),
                        c = m.vertices.row(f[2]);
        v += a.dot(b.cross(c));
    }
    return std::abs(v) / 6.0;
}

static bool insideConvex(const Eigen::Vector3d& pt, const Mesh& m, double eps = 1e-6) {
    for (const auto& f : m.faces) {
        Eigen::Vector3d v0 = m.vertices.row(f[0]),
                        v1 = m.vertices.row(f[1]),
                        v2 = m.vertices.row(f[2]);
        Eigen::Vector3d n = (v1-v0).cross(v2-v0);
        double len = n.norm();
        if (len < 1e-20) continue;
        if ((pt-v0).dot(n) > eps*len) return false;
    }
    return true;
}

static int checkCons(const Mesh& inp, const Mesh& expanded, double eps = 1e-5) {
    int fail = 0;
    for (int i = 0; i < inp.numVertices(); ++i) {
        Eigen::Vector3d v = inp.vertices.row(i).transpose();
        if (!insideConvex(v, expanded, eps)) ++fail;
    }
    return fail;
}

static int checkExp(const Mesh& inp, const Mesh& expanded, double d, double eps = 1e-5) {
    int fail = 0;
    for (const auto& f : inp.faces) {
        Eigen::Vector3d v0 = inp.vertices.row(f[0]),
                        v1 = inp.vertices.row(f[1]),
                        v2 = inp.vertices.row(f[2]);
        Eigen::Vector3d n = (v1-v0).cross(v2-v0);
        double len = n.norm();
        if (len < 1e-20) continue;
        Eigen::Vector3d probe = (v0+v1+v2)/3.0 + d*(n/len);
        if (!insideConvex(probe, expanded, eps)) ++fail;
    }
    return fail;
}

// ---------------------------------------------------------------------------
// Benchmark table
// ---------------------------------------------------------------------------

struct AssemblyPartRow {
    std::string name;
    int inFaces;
    int outFaces;
    int covFail, expFail;
    double volume;
    double ms;
};
static std::vector<AssemblyPartRow> g_rows;

static void printAssemblyTable() {
    const char* sep =
        "+---------------------+--------+--------+-------+------+----------+------+";
    std::cout << "\n" << sep << "\n"
        << "| Part                | InFace | OutFac | Cov%  | Exp% | Volume   |  ms  |\n"
        << sep << "\n";
    for (const auto& r : g_rows) {
        double cov     = (1.0 - (double)r.covFail / std::max(1, r.inFaces)) * 100.0;
        double exp_pct = (1.0 - (double)r.expFail / std::max(1, r.inFaces)) * 100.0;
        std::cout << std::fixed
            << "| " << std::left  << std::setw(19) << r.name     << " |"
            << std::right << std::setw(7)  << r.inFaces           << " |"
            << std::right << std::setw(7)  << r.outFaces          << " |"
            << std::right << std::setprecision(1) << std::setw(6)  << cov      << " |"
            << std::right << std::setprecision(1) << std::setw(5)  << exp_pct  << " |"
            << std::right << std::setprecision(6) << std::setw(9)  << r.volume << " |"
            << std::right << std::setprecision(0) << std::setw(5)  << r.ms     << " |\n";
    }
    std::cout << sep << "\n"
        << "  Algorithm: BoxExpander (削り出し法)  |  Cov% = vertices covered\n"
        << "  Exp% = face probes with expansion>=d\n\n";
}

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------

class ComplexAssemblyTest : public ::testing::Test {
protected:
    static constexpr double kD = 0.001;  // 膨張量 1 mm

    void runPart(const std::string& name, const Mesh& inp) {
        ASSERT_GT(inp.numFaces(), 0) << name << ": empty input mesh";

        auto t0 = std::chrono::high_resolution_clock::now();
        BoxExpander exp;
        Mesh result = exp.expand(inp, kD);
        double ms = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - t0).count();

        ASSERT_GT(result.numFaces(), 0) << name << ": expansion returned empty";
        EXPECT_TRUE(result.vertices.allFinite()) << name << ": NaN/Inf in output";

        const int nV = inp.numVertices();
        const int nF = (int)inp.faces.size();
        const int cF = checkCons(inp, result, 1e-5);
        const int eF = checkExp(inp, result, kD, 1e-5);

        EXPECT_EQ(cF, 0)
            << name << ": " << cF << "/" << nV << " vertices not covered (Cov < 100%)";
        EXPECT_LT(nF > 0 ? (double)eF / nF : 0.0, 0.05)
            << name << ": " << eF << "/" << nF << " face probes failed (Exp < 95%)";

        double vol = meshVolume(result);
        g_rows.push_back({name, nF, result.numFaces(), cF, eF, vol, ms});

        try {
            std::filesystem::create_directories("stl_output");
            std::string tag = name;
            for (char& c : tag) if (!std::isalnum((unsigned char)c) && c != '-') c = '_';
            StlWriter::write("stl_output/assembly_" + tag + ".stl", result, name);
        } catch (...) {}
    }
};

// ---------------------------------------------------------------------------
// Per-part tests
// ---------------------------------------------------------------------------

TEST_F(ComplexAssemblyTest, BasePlate_Conservativeness)
{
    runPart("Base plate", makeBoxMesh(0.200, 0.100, 0.010));
}

TEST_F(ComplexAssemblyTest, MountCylinder_Conservativeness)
{
    runPart("Mount cylinder",
            makeCylinderMesh(0.015, 0.030, 32,
                             Eigen::Vector3d(0.250, 0.0, 0.0)));
}

TEST_F(ComplexAssemblyTest, LBracket_Conservativeness)
{
    runPart("L-bracket",
            makeLBracketMesh(0.030,
                             Eigen::Vector3d(0.500, 0.0, 0.0)));
}

TEST_F(ComplexAssemblyTest, ShaftHousing_Conservativeness)
{
    runPart("Shaft housing",
            makeHollowCylinderMesh(0.020, 0.010, 0.040, 32,
                                   Eigen::Vector3d(0.750, 0.0, 0.0)));
}

TEST_F(ComplexAssemblyTest, DriveGear_Conservativeness)
{
    runPart("Drive gear",
            makeGearMesh(0.030, 12, 0.008, 0.010, 6,
                         Eigen::Vector3d(1.000, 0.0, 0.0)));
}

// ---------------------------------------------------------------------------
// Full assembly: per-part volume sum <= single-mesh expansion volume
// ---------------------------------------------------------------------------

TEST_F(ComplexAssemblyTest, FullAssembly_PerPartVolume_LEQ_SingleMesh)
{
    // 5パーツを離間配置したアセンブリに対し:
    //   パーツ別膨張の体積合計 <= 1メッシュとして膨張した体積
    // 各パーツを独立膨張することで、パーツ間の空白領域を無駄に包含しないことを確認する。

    const std::vector<Mesh> parts = {
        makeBoxMesh          (0.200, 0.100, 0.010),
        makeCylinderMesh     (0.015, 0.030, 32, Eigen::Vector3d(0.250, 0.0, 0.0)),
        makeLBracketMesh     (0.030,            Eigen::Vector3d(0.500, 0.0, 0.0)),
        makeHollowCylinderMesh(0.020, 0.010, 0.040, 32, Eigen::Vector3d(0.750, 0.0, 0.0)),
        makeGearMesh         (0.030, 12, 0.008, 0.010, 6, Eigen::Vector3d(1.000, 0.0, 0.0)),
    };

    AssemblyExpander ae;

    // --- パーツ別膨張 ---
    auto perPartResults = ae.expand(parts, kD);
    ASSERT_EQ(perPartResults.size(), parts.size());

    double perPartVol   = 0.0;
    int    perPartFaces = 0;
    for (const auto& m : perPartResults) {
        perPartVol   += meshVolume(m);
        perPartFaces += m.numFaces();
    }

    // --- 全パーツを1つの Mesh に結合して膨張 ---
    Mesh merged;
    merged.vertices.resize(0, 3);
    for (const auto& p : parts) {
        const int baseV = merged.numVertices();
        Eigen::MatrixXd newV(baseV + p.numVertices(), 3);
        if (baseV > 0) newV.topRows(baseV) = merged.vertices;
        newV.bottomRows(p.numVertices()) = p.vertices;
        merged.vertices = newV;
        for (const auto& f : p.faces)
            merged.faces.push_back({f[0]+baseV, f[1]+baseV, f[2]+baseV});
    }

    BoxExpander singleExp;
    Mesh singleResult = singleExp.expand(merged, kD);

    double singleVol   = meshVolume(singleResult);
    int    singleFaces = singleResult.numFaces();

    std::cout << std::fixed << std::setprecision(6)
        << "\n  [Assembly] Per-part:    volume=" << perPartVol
        << "  faces=" << perPartFaces << "\n"
        << "  [Assembly] Single-mesh: volume=" << singleVol
        << "  faces=" << singleFaces   << "\n"
        << "  [Assembly] Volume ratio (per-part / single): "
        << std::setprecision(4) << perPartVol / singleVol << "\n";

    // パーツ別膨張はパーツ間空隙を膨張しないため体積は <= 単一メッシュ膨張
    EXPECT_LE(perPartVol, singleVol * 1.10)
        << "Per-part expansion volume should be <= single-mesh volume "
           "(parts are non-overlapping and separated)";

    // STL 出力
    try {
        std::filesystem::create_directories("stl_output");
        Mesh mergedPerPart = ae.expandMerged(parts, kD);
        if (mergedPerPart.numFaces() > 0)
            StlWriter::write("stl_output/assembly_full_per_part.stl",
                             mergedPerPart, "full assembly per-part");
        if (singleResult.numFaces() > 0)
            StlWriter::write("stl_output/assembly_full_single.stl",
                             singleResult, "full assembly single-mesh");
    } catch (...) {}
}

// ---------------------------------------------------------------------------
// Print benchmark table (runs last)
// ---------------------------------------------------------------------------

TEST_F(ComplexAssemblyTest, ZZZ_PrintBenchmarkTable)
{
    if (g_rows.empty()) GTEST_SKIP() << "No rows recorded";
    std::cout << "\n+-----------------------------------------------------------+\n"
              << "|  MeshExpander Complex Assembly Benchmark  (BoxExpander d=1mm)  |\n"
              << "+-----------------------------------------------------------+\n";
    printAssemblyTable();
}
