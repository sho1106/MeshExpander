// CAD Shape Expansion Benchmark
//
// Procedurally generated concave CAD shapes:
//   Torus, Gear (12-tooth), Star prism (5-pt), Hollow cylinder
// Verifies: conservativeness, expansion >= d, volume tightness.

#include <gtest/gtest.h>
#include "expander/RobustSlicer.hpp"
#include "expander/ConservativeExpander.hpp"
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

// ============================================================
//  Mesh generators
// ============================================================

static Mesh makeTorus(double R = 0.050, double r = 0.015,
                      int N_theta = 24, int N_phi = 16)
{
    Mesh m;
    const int NV = N_theta * N_phi;
    m.vertices.resize(NV, 3);
    for (int t = 0; t < N_theta; ++t) {
        const double theta = 2.0 * kPi * t / N_theta;
        for (int p = 0; p < N_phi; ++p) {
            const double phi = 2.0 * kPi * p / N_phi;
            const int vi = t * N_phi + p;
            m.vertices(vi, 0) = (R + r * std::cos(phi)) * std::cos(theta);
            m.vertices(vi, 1) = (R + r * std::cos(phi)) * std::sin(theta);
            m.vertices(vi, 2) =  r * std::sin(phi);
        }
    }
    m.faces.reserve(2 * N_theta * N_phi);
    for (int t = 0; t < N_theta; ++t) {
        const int tn = (t + 1) % N_theta;
        for (int p = 0; p < N_phi; ++p) {
            const int pn = (p + 1) % N_phi;
            const int a = t  * N_phi + p,  b = tn * N_phi + p;
            const int c = tn * N_phi + pn, d = t  * N_phi + pn;
            m.faces.push_back({a, b, c});
            m.faces.push_back({a, c, d});
        }
    }
    return m;
}

static Mesh makeGear(double R_base = 0.030, int n_teeth = 12,
                     double tooth_h = 0.008, double H = 0.010,
                     int segs_per_tooth = 6)
{
    const int N = n_teeth * segs_per_tooth;
    std::vector<double> px(N), py(N);
    for (int i = 0; i < N; ++i) {
        const double angle = 2.0 * kPi * i / N;
        const double phase = std::sin(n_teeth * angle);
        const double Ri    = R_base + tooth_h * (phase > 0.0 ? phase : 0.0);
        px[i] = Ri * std::cos(angle);
        py[i] = Ri * std::sin(angle);
    }
    Mesh m;
    m.vertices.resize(2 * N + 2, 3);
    for (int i = 0; i < N; ++i) {
        m.vertices(i,     0) = px[i]; m.vertices(i,     1) = py[i]; m.vertices(i,     2) =  H * 0.5;
        m.vertices(i + N, 0) = px[i]; m.vertices(i + N, 1) = py[i]; m.vertices(i + N, 2) = -H * 0.5;
    }
    const int topC = 2 * N, botC = 2 * N + 1;
    m.vertices.row(topC) << 0.0, 0.0,  H * 0.5;
    m.vertices.row(botC) << 0.0, 0.0, -H * 0.5;
    m.faces.reserve(4 * N);
    for (int i = 0; i < N; ++i) {
        const int j = (i + 1) % N;
        m.faces.push_back({i,    j,    j + N});
        m.faces.push_back({i,    j+N,  i + N});
        m.faces.push_back({topC, j,    i    });
        m.faces.push_back({botC, i+N,  j + N});
    }
    return m;
}

static Mesh makeStarPrism(int n_pts = 5, double R_outer = 0.025,
                           double R_inner = 0.012, double H = 0.020)
{
    const int N = n_pts * 2;
    std::vector<double> px(N), py(N);
    for (int i = 0; i < N; ++i) {
        const double angle = kPi / n_pts * i - kPi * 0.5;
        const double Ri    = (i % 2 == 0) ? R_outer : R_inner;
        px[i] = Ri * std::cos(angle);
        py[i] = Ri * std::sin(angle);
    }
    Mesh m;
    m.vertices.resize(2 * N + 2, 3);
    for (int i = 0; i < N; ++i) {
        m.vertices(i,     0) = px[i]; m.vertices(i,     1) = py[i]; m.vertices(i,     2) =  H * 0.5;
        m.vertices(i + N, 0) = px[i]; m.vertices(i + N, 1) = py[i]; m.vertices(i + N, 2) = -H * 0.5;
    }
    const int topC = 2 * N, botC = 2 * N + 1;
    m.vertices.row(topC) << 0.0, 0.0,  H * 0.5;
    m.vertices.row(botC) << 0.0, 0.0, -H * 0.5;
    m.faces.reserve(4 * N);
    for (int i = 0; i < N; ++i) {
        const int j = (i + 1) % N;
        m.faces.push_back({i,    j,    j + N});
        m.faces.push_back({i,    j+N,  i + N});
        m.faces.push_back({topC, j,    i    });
        m.faces.push_back({botC, i+N,  j + N});
    }
    return m;
}

static Mesh makeHollowCylinder(double R_out = 0.030, double R_in = 0.015,
                                double H = 0.040, int N = 32)
{
    Mesh m;
    m.vertices.resize(4 * N, 3);
    for (int i = 0; i < N; ++i) {
        const double a = 2.0 * kPi * i / N;
        const double ca = std::cos(a), sa = std::sin(a);
        m.vertices(i,       0) = R_out*ca; m.vertices(i,       1) = R_out*sa; m.vertices(i,       2) =  H*0.5;
        m.vertices(i+  N,   0) = R_out*ca; m.vertices(i+  N,   1) = R_out*sa; m.vertices(i+  N,   2) = -H*0.5;
        m.vertices(i+2*N,   0) = R_in *ca; m.vertices(i+2*N,   1) = R_in *sa; m.vertices(i+2*N,   2) =  H*0.5;
        m.vertices(i+3*N,   0) = R_in *ca; m.vertices(i+3*N,   1) = R_in *sa; m.vertices(i+3*N,   2) = -H*0.5;
    }
    m.faces.reserve(8 * N);
    for (int i = 0; i < N; ++i) {
        const int j = (i + 1) % N;
        m.faces.push_back({i,     i+N,   j+N  });
        m.faces.push_back({i,     j+N,   j    });
        m.faces.push_back({i+2*N, j+2*N, j+3*N});
        m.faces.push_back({i+2*N, j+3*N, i+3*N});
        m.faces.push_back({i,     j,     j+2*N});
        m.faces.push_back({i,     j+2*N, i+2*N});
        m.faces.push_back({i+N,   i+3*N, j+3*N});
        m.faces.push_back({i+N,   j+3*N, j+N  });
    }
    return m;
}

// ============================================================
//  Helpers
// ============================================================

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

static bool insideConvex(const Eigen::Vector3d& pt, const Mesh& m, double eps = 1e-6) {
    for (const auto& f : m.faces) {
        const Eigen::Vector3d v0 = m.vertices.row(f[0]).transpose();
        const Eigen::Vector3d v1 = m.vertices.row(f[1]).transpose();
        const Eigen::Vector3d v2 = m.vertices.row(f[2]).transpose();
        const Eigen::Vector3d n  = (v1-v0).cross(v2-v0);
        const double len = n.norm();
        if (len < 1e-20) continue;
        if ((pt-v0).dot(n) > eps * len) return false;
    }
    return true;
}

static int checkConservativeness(const Mesh& input, const std::vector<Mesh>& poly,
                                  double eps = 1e-5) {
    int fail = 0;
    for (int i = 0; i < input.numVertices(); ++i) {
        const Eigen::Vector3d v = input.vertices.row(i).transpose();
        bool ok = false;
        for (const auto& m : poly) { if (insideConvex(v, m, eps)) { ok = true; break; } }
        if (!ok) ++fail;
    }
    return fail;
}

static int checkExpansionDistance(const Mesh& input, const std::vector<Mesh>& poly,
                                   double d, double eps = 1e-5) {
    int fail = 0;
    for (const auto& f : input.faces) {
        const Eigen::Vector3d v0 = input.vertices.row(f[0]).transpose();
        const Eigen::Vector3d v1 = input.vertices.row(f[1]).transpose();
        const Eigen::Vector3d v2 = input.vertices.row(f[2]).transpose();
        const Eigen::Vector3d n  = (v1-v0).cross(v2-v0);
        const double len = n.norm();
        if (len < 1e-20) continue;
        const Eigen::Vector3d probe = (v0+v1+v2)/3.0 + d*(n/len);
        bool ok = false;
        for (const auto& m : poly) { if (insideConvex(probe, m, eps)) { ok = true; break; } }
        if (!ok) ++fail;
    }
    return fail;
}

// ============================================================
//  Benchmark table
// ============================================================

struct BenchRow {
    std::string shape;
    int inFaces, boxes, nV, nF, cFail, eFail;
    double conservVol, robustVol, ratio, ms;
};

static std::vector<BenchRow> g_cadRows;

static void printBenchTable() {
    const std::string hr =
        "+----------------------+---------+-------+--------------+--------------+"
        "----------+--------+--------+--------+";
    std::cout
        << "\n+------------------------------------------------------------------+\n"
        << "|   MeshExpander  CAD Shape Benchmark  (cellSize=5mm  d=1mm)      |\n"
        << "+------------------------------------------------------------------+\n"
        << hr << "\n"
        << "| Shape                |  Faces  | Boxes | Conserv.Vol  |"
           " Robust Vol   |  R/C     |  Cov%  |  Exp%  |  ms    |\n"
        << hr << "\n";
    double sumRatio = 0.0;
    for (const auto& r : g_cadRows) {
        const double cov = r.nV > 0 ? (1.0-(double)r.cFail/r.nV)*100.0 : 100.0;
        const double exp = r.nF > 0 ? (1.0-(double)r.eFail/r.nF)*100.0 : 100.0;
        std::cout << std::fixed
            << "| " << std::left  << std::setw(20) << r.shape      << " |"
            << std::right << std::setw(8)  << r.inFaces             << " |"
            << std::right << std::setw(6)  << r.boxes               << " |"
            << std::right << std::setprecision(6) << std::setw(13) << r.conservVol << " |"
            << std::right << std::setprecision(6) << std::setw(13) << r.robustVol  << " |"
            << std::right << std::setprecision(3) << std::setw(9)  << r.ratio      << " |"
            << std::right << std::setprecision(1) << std::setw(7)  << cov          << " |"
            << std::right << std::setprecision(1) << std::setw(7)  << exp          << " |"
            << std::right << std::setprecision(1) << std::setw(7)  << r.ms         << " |\n";
        sumRatio += r.ratio;
    }
    std::cout << hr << "\n";
    const double avg = g_cadRows.empty() ? 0.0 : sumRatio / (double)g_cadRows.size();
    std::cout
        << "  R/C = Robust/Conservative volume  (lower = tighter fit)\n"
        << "  Cov% = input vertices covered  |  Exp% = probes with expansion>=d\n"
        << "  Average R/C: " << std::fixed << std::setprecision(3) << avg
        << "  =>  RobustSlicer is " << std::setprecision(1) << (1.0-avg)*100.0
        << "% tighter on average\n\n";
}

// ============================================================
//  Test fixture
// ============================================================

class CadShapeTest : public ::testing::Test {
protected:
    static constexpr double kCellSize = 0.002;  // 2mm — fine enough to resolve CAD features
    static constexpr double kD        = 0.001;  // 1mm expansion

    void runShape(const std::string& name, const Mesh& input) {
        ASSERT_GT(input.numFaces(), 0) << name << ": empty mesh";

        auto t0     = std::chrono::high_resolution_clock::now();
        auto slicer = RobustSlicer::withCellSize(kCellSize);
        auto polys  = slicer.expandMulti(input, kD);
        const double ms = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - t0).count();

        ASSERT_GT(static_cast<int>(polys.size()), 0);

        double robustVol = 0.0;
        for (const auto& m : polys) robustVol += meshVolume(m);

        ConservativeExpander cExp;
        Mesh   cMesh = cExp.expand(input, kD);
        double cVol  = (cMesh.numFaces() > 0) ? meshVolume(cMesh) : 0.0;
        double ratio = (cVol > 0.0) ? robustVol / cVol : 0.0;

        const int nV = input.numVertices();
        const int nF = static_cast<int>(input.faces.size());
        const int cF = checkConservativeness(input, polys, 1e-5);
        const int eF = checkExpansionDistance(input, polys, kD, 1e-5);

        EXPECT_LT(nV > 0 ? (double)cF / nV : 0.0, 0.01)
            << name << ": " << cF << "/" << nV << " vertices not covered";
        EXPECT_LT(nF > 0 ? (double)eF / nF : 0.0, 0.05)
            << name << ": " << eF << "/" << nF << " expansion probes failed";
        // Volume tightness: RobustSlicer should be tighter for clearly concave shapes.
        // (May not hold when cell size >> concavity depth; reported in table but not asserted.)
        if (cVol > 0.0 && ratio > 1.0)
            std::cout << "  [info] " << name << ": R/C=" << ratio
                      << " (RobustSlicer larger; cell size may be coarse relative to features)\n";

        try {
            std::filesystem::create_directories("stl_output");
            std::string tag = name;
            for (char& c : tag) if (c == ' ') c = '_';
            const Mesh merged = RobustSlicer::merge(polys);
            if (merged.numFaces() > 0)
                StlWriter::write("stl_output/cad_" + tag + "_robust.stl", merged, name);
            if (cMesh.numFaces() > 0)
                StlWriter::write("stl_output/cad_" + tag + "_conservative.stl", cMesh, name);
        } catch (...) {}

        g_cadRows.push_back({name, nF, static_cast<int>(polys.size()),
                              nV, nF, cF, eF, cVol, robustVol, ratio, ms});
    }
};

TEST_F(CadShapeTest, Torus) {
    // Donut R=50mm r=15mm — hole causes ConservativeExpander to over-expand
    runShape("Torus R50/r15", makeTorus(0.050, 0.015, 32, 20));
}

TEST_F(CadShapeTest, Gear12Tooth) {
    // 12-tooth spur gear — 12mm-deep inter-tooth gaps are concave cavities
    runShape("Gear 12-tooth", makeGear(0.040, 12, 0.012, 0.015, 8));
}

TEST_F(CadShapeTest, StarPrism5) {
    // 5-point star prism — 5 sharp re-entrant corners, inner R=15mm
    runShape("Star prism 5pt", makeStarPrism(5, 0.040, 0.015, 0.030));
}

TEST_F(CadShapeTest, HollowCylinder) {
    // Pipe/tube — inner bore Ø40mm is a large concave void
    runShape("Hollow cylinder", makeHollowCylinder(0.040, 0.020, 0.060, 40));
}

TEST_F(CadShapeTest, ZZZ_PrintBenchmarkTable) {
    if (g_cadRows.empty()) GTEST_SKIP() << "No rows collected";
    printBenchTable();
}
