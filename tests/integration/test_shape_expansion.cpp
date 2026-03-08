// Integration tests: 球 / 円柱 / 尖った円錐 の 1mm 膨張精度評価
//
// 各テストは以下を計算・表示する:
//   ideal_volume  : Steiner公式による理論的なMinkowski和体積
//   actual_volume : 出力メッシュの体積 (発散定理)
//   ratio         : actual / ideal  (1.0 = 完璧, 大きいほど無駄が多い)
//   over%         : (ratio - 1) * 100
//
// アルゴリズム: 面法線ベース半空間交差 (ConservativeExpander, face-normal mode)
//   入力メッシュの面法線を収集し、20度閾値でマージ → 各法線方向の半空間を交差。
//   26固定方向と異なり形状に適応的。apex dominance なし。
//   球 ≈ +3%, 円柱 ≈ +1%, 円錐 ≈ +3%

#include <gtest/gtest.h>
#include "expander/ConservativeExpander.hpp"
#include "expander/MathUtils.hpp"
#include "expander/StlWriter.hpp"

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

static const std::string kStlOutputDir = "stl_output";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace expander;

// ---------------------------------------------------------------------------
// Volume of a triangulated closed mesh (divergence theorem)
// ---------------------------------------------------------------------------
static double meshVolume(const Mesh& m) {
    double vol = 0.0;
    for (const auto& f : m.faces) {
        Eigen::Vector3d v0 = m.vertices.row(f[0]).transpose();
        Eigen::Vector3d v1 = m.vertices.row(f[1]).transpose();
        Eigen::Vector3d v2 = m.vertices.row(f[2]).transpose();
        vol += v0.dot(v1.cross(v2));
    }
    return std::abs(vol) / 6.0;
}

// ---------------------------------------------------------------------------
// Mesh factories — 頂点 + 面データ (face-normal mode に必須)
// ---------------------------------------------------------------------------

// 球: latitude-longitude 三角形メッシュ
static Mesh makeSphere(double R, int N = 48) {
    Mesh m;
    std::vector<Eigen::RowVector3d> pts;

    pts.push_back({0.0, 0.0, R});
    for (int i = 1; i < N; ++i) {
        double theta = M_PI * i / N;
        for (int j = 0; j < 2 * N; ++j) {
            double phi = 2.0 * M_PI * j / (2 * N);
            pts.push_back({R * std::sin(theta) * std::cos(phi),
                           R * std::sin(theta) * std::sin(phi),
                           R * std::cos(theta)});
        }
    }
    pts.push_back({0.0, 0.0, -R});

    m.vertices.resize(static_cast<int>(pts.size()), 3);
    for (int i = 0; i < static_cast<int>(pts.size()); ++i)
        m.vertices.row(i) = pts[i];

    const int nMid = 2 * N;
    for (int j = 0; j < nMid; ++j)
        m.faces.push_back({0, 1 + j, 1 + (j + 1) % nMid});
    for (int i = 0; i < N - 2; ++i) {
        int base = 1 + i * nMid;
        for (int j = 0; j < nMid; ++j) {
            int a = base + j,         b = base + (j + 1) % nMid;
            int c = base + nMid + j,  d = base + nMid + (j + 1) % nMid;
            m.faces.push_back({a, c, b});
            m.faces.push_back({b, c, d});
        }
    }
    const int spIdx    = static_cast<int>(pts.size()) - 1;
    const int lastBase = 1 + (N - 2) * nMid;
    for (int j = 0; j < nMid; ++j)
        m.faces.push_back({lastBase + j, spIdx, lastBase + (j + 1) % nMid});

    return m;
}

// 円柱: 側面 + 上下蓋
static Mesh makeCylinder(double R, double H, int N = 128) {
    Mesh m;
    m.vertices.resize(2 * N + 2, 3);
    for (int i = 0; i < N; ++i) {
        double phi = 2.0 * M_PI * i / N;
        m.vertices.row(i)     = Eigen::RowVector3d(R * std::cos(phi), R * std::sin(phi),  H * 0.5);
        m.vertices.row(N + i) = Eigen::RowVector3d(R * std::cos(phi), R * std::sin(phi), -H * 0.5);
    }
    m.vertices.row(2 * N)     = Eigen::RowVector3d(0.0, 0.0,  H * 0.5);
    m.vertices.row(2 * N + 1) = Eigen::RowVector3d(0.0, 0.0, -H * 0.5);

    for (int i = 0; i < N; ++i) {
        int a = i, b = (i + 1) % N, c = N + i, d = N + (i + 1) % N;
        m.faces.push_back({a, c, b});
        m.faces.push_back({b, c, d});
    }
    for (int i = 0; i < N; ++i)
        m.faces.push_back({2 * N, i, (i + 1) % N});
    for (int i = 0; i < N; ++i)
        m.faces.push_back({2 * N + 1, N + (i + 1) % N, N + i});

    return m;
}

// 円錐: 側面 + 底蓋
static Mesh makeCone(double R, double H, int N = 128) {
    Mesh m;
    m.vertices.resize(N + 2, 3);
    m.vertices.row(0) = Eigen::RowVector3d(0.0, 0.0, H);
    for (int i = 0; i < N; ++i) {
        double phi = 2.0 * M_PI * i / N;
        m.vertices.row(1 + i) = Eigen::RowVector3d(R * std::cos(phi), R * std::sin(phi), 0.0);
    }
    m.vertices.row(N + 1) = Eigen::RowVector3d(0.0, 0.0, 0.0);

    for (int i = 0; i < N; ++i)
        m.faces.push_back({0, 1 + i, 1 + (i + 1) % N});
    for (int i = 0; i < N; ++i)
        m.faces.push_back({N + 1, 1 + (i + 1) % N, 1 + i});

    return m;
}

// ---------------------------------------------------------------------------
// 理想体積 (Steiner公式)
// ---------------------------------------------------------------------------
static double idealSphereVolume(double R, double d) {
    return (4.0 / 3.0) * M_PI * std::pow(R + d, 3.0);
}
static double idealCylinderVolume(double R, double H, double d) {
    return M_PI * R * R * H
         + d * (2.0 * M_PI * R * H + 2.0 * M_PI * R * R)
         + d * d * (M_PI * H + M_PI * M_PI * R)
         + (4.0 / 3.0) * M_PI * d * d * d;
}
static double idealConeVolume(double R, double H, double d) {
    const double l     = std::sqrt(R * R + H * H);
    const double alpha = std::atan2(R, H);
    return (1.0 / 3.0) * M_PI * R * R * H
         + d * M_PI * R * (l + R)
         + d * d * M_PI * R * (1.0 + M_PI / 2.0 + alpha)
         + (4.0 / 3.0) * M_PI * d * d * d;
}

// ---------------------------------------------------------------------------
// 評価ヘルパー
// ---------------------------------------------------------------------------
struct EvalResult {
    std::string label;
    double idealVol, actualVol, ratio, overPct;
    int nFaces, nVerts;
};

static EvalResult evalExpansion(const std::string& label,
                                 const Mesh&        input,
                                 double             d,
                                 double             ideal)
{
    ConservativeExpander expander;
    Mesh result = expander.expand(input, d);

    double actual = (result.numFaces() > 0) ? meshVolume(result) : 0.0;
    double ratio  = (ideal > 0.0) ? actual / ideal : 0.0;

    if (result.numFaces() > 0) {
        std::string fname = label;
        for (char& c : fname) {
            if (c == ' ') c = '_';
            else if (c == '/' || c == '\\' || c == ':') c = '-';
        }
        try {
            std::filesystem::create_directories(kStlOutputDir);
            StlWriter::write(kStlOutputDir + "/" + fname + ".stl", result, label);
        } catch (...) {}
    }

    return {label, ideal, actual, ratio, (ratio - 1.0) * 100.0,
            result.numFaces(), result.numVertices()};
}

static void printResult(const EvalResult& r) {
    std::cout << std::fixed << std::setprecision(3)
              << "  " << std::left << std::setw(34) << r.label
              << "  ideal=" << std::setw(11) << r.idealVol
              << "  actual=" << std::setw(11) << r.actualVol
              << "  ratio=" << std::setw(6) << r.ratio
              << "  over=" << std::setw(7) << r.overPct << "%"
              << "  faces=" << r.nFaces
              << "  verts=" << r.nVerts
              << "\n";
}

// ---------------------------------------------------------------------------
// 球 テスト — ratio ≈ 1.03 (面法線モード)
// ---------------------------------------------------------------------------
TEST(ShapeExpansion, Sphere) {
    const double d = 1.0;
    const double kMaxRatio = 1.10;

    struct Case { std::string name; double R; };
    std::vector<Case> cases = {
        {"Sphere  R= 10 d=1 (d/R=10%)", 10.0},
        {"Sphere  R= 50 d=1 (d/R= 2%)", 50.0},
        {"Sphere  R=100 d=1 (d/R= 1%)", 100.0},
    };

    std::cout << "\n[Sphere expansion accuracy — face-normal half-space]\n";
    for (const auto& c : cases) {
        SCOPED_TRACE(c.name);
        auto r = evalExpansion(c.name, makeSphere(c.R, 48), d, idealSphereVolume(c.R, d));
        printResult(r);
        EXPECT_GT(r.nFaces, 0);
        EXPECT_GE(r.ratio, 0.98);
        EXPECT_LE(r.ratio, kMaxRatio);
    }
}

// ---------------------------------------------------------------------------
// 円柱 テスト — ratio ≈ 1.01 (面法線モード)
// ---------------------------------------------------------------------------
TEST(ShapeExpansion, Cylinder) {
    const double d = 1.0;
    const double kMaxRatio = 1.05;

    struct Case { std::string name; double R; double H; };
    std::vector<Case> cases = {
        {"Cylinder R= 10 H= 20 d=1", 10.0,  20.0},
        {"Cylinder R= 30 H= 60 d=1", 30.0,  60.0},
        {"Cylinder R=100 H=200 d=1", 100.0, 200.0},
    };

    std::cout << "\n[Cylinder expansion accuracy — face-normal half-space]\n";
    for (const auto& c : cases) {
        SCOPED_TRACE(c.name);
        auto r = evalExpansion(c.name, makeCylinder(c.R, c.H, 128), d,
                               idealCylinderVolume(c.R, c.H, d));
        printResult(r);
        EXPECT_GT(r.nFaces, 0);
        EXPECT_GE(r.ratio, 0.98);
        EXPECT_LE(r.ratio, kMaxRatio);
    }
}

// ---------------------------------------------------------------------------
// 尖った円錐 テスト — ratio ≈ 1.03 (apex dominance 解消)
// ---------------------------------------------------------------------------
TEST(ShapeExpansion, SharpCone) {
    const double d = 1.0;
    const double kMaxRatio = 1.15;

    struct Case { std::string name; double R; double H; };
    std::vector<Case> cases = {
        {"Cone    R= 10 H= 30 d=1", 10.0,  30.0},
        {"Cone    R= 30 H= 90 d=1", 30.0,  90.0},
        {"Cone    R=100 H=300 d=1", 100.0, 300.0},
    };

    std::cout << "\n[Cone expansion accuracy — face-normal half-space]\n";
    for (const auto& c : cases) {
        SCOPED_TRACE(c.name);
        auto r = evalExpansion(c.name, makeCone(c.R, c.H, 128), d,
                               idealConeVolume(c.R, c.H, d));
        printResult(r);
        EXPECT_GT(r.nFaces, 0);
        EXPECT_GE(r.ratio, 0.98);
        EXPECT_LE(r.ratio, kMaxRatio);
    }
}

// ---------------------------------------------------------------------------
// NaN / Inf 堅牢性テスト
// ---------------------------------------------------------------------------
TEST(ShapeExpansion, ExtremeShapesNoNaN) {
    const double d = 1.0;
    ConservativeExpander expander;

    Mesh thinCyl = makeCylinder(1.0, 50.0, 64);
    Mesh r1 = expander.expand(thinCyl, d);
    for (int i = 0; i < r1.numVertices(); ++i)
        EXPECT_TRUE(r1.vertices.row(i).allFinite()) << "Thin cylinder: NaN at vertex " << i;

    Mesh sharpCone = makeCone(1.0, 30.0, 64);
    Mesh r2 = expander.expand(sharpCone, d);
    for (int i = 0; i < r2.numVertices(); ++i)
        EXPECT_TRUE(r2.vertices.row(i).allFinite()) << "Sharp cone: NaN at vertex " << i;

    Mesh flattened = makeSphere(50.0, 4);
    Mesh r3 = expander.expand(flattened, d);
    for (int i = 0; i < r3.numVertices(); ++i)
        EXPECT_TRUE(r3.vertices.row(i).allFinite()) << "Low-res sphere: NaN at vertex " << i;
}
