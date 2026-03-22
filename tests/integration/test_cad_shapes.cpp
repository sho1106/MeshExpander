// CAD Shape Expansion Benchmark
// Shapes: Torus, Gear (12-tooth), Star prism (5-pt), Hollow cylinder
// Algorithm: BoxExpander (削り出し法)
// Verifies: conservativeness (Cov%=100%), expansion>=d (Exp%>=95%)

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
static const double kPi = 3.14159265358979323846;

// ===== Mesh generators =====

static Mesh makeTorus(double R=0.050, double r=0.015, int Nt=24, int Np=16) {
    Mesh m; m.vertices.resize(Nt*Np, 3);
    for (int t=0; t<Nt; ++t) {
        double th = 2*kPi*t/Nt;
        for (int p=0; p<Np; ++p) {
            double ph = 2*kPi*p/Np; int vi = t*Np+p;
            m.vertices(vi,0) = (R+r*std::cos(ph))*std::cos(th);
            m.vertices(vi,1) = (R+r*std::cos(ph))*std::sin(th);
            m.vertices(vi,2) = r*std::sin(ph); } }
    m.faces.reserve(2*Nt*Np);
    for (int t=0; t<Nt; ++t) { int tn=(t+1)%Nt;
        for (int p=0; p<Np; ++p) { int pn=(p+1)%Np;
            int a=t*Np+p, b=tn*Np+p, c=tn*Np+pn, d=t*Np+pn;
            m.faces.push_back({a,b,c}); m.faces.push_back({a,c,d}); } }
    return m;
}

static Mesh makeGear(double Rb=0.030, int nt=12, double th=0.008,
                     double H=0.010, int sp=6) {
    const int N = nt*sp;
    std::vector<double> px(N), py(N);
    for (int i=0; i<N; ++i) {
        double a=2*kPi*i/N, ph=std::sin(nt*a);
        double Ri = Rb + th*(ph>0.0 ? ph : 0.0);
        px[i]=Ri*std::cos(a); py[i]=Ri*std::sin(a); }
    Mesh m; m.vertices.resize(2*N+2, 3);
    for (int i=0; i<N; ++i) {
        m.vertices(i,  0)=px[i]; m.vertices(i,  1)=py[i]; m.vertices(i,  2)= H*0.5;
        m.vertices(i+N,0)=px[i]; m.vertices(i+N,1)=py[i]; m.vertices(i+N,2)=-H*0.5; }
    int tC=2*N, bC=2*N+1;
    m.vertices.row(tC) << 0.0, 0.0,  H*0.5;
    m.vertices.row(bC) << 0.0, 0.0, -H*0.5;
    m.faces.reserve(4*N);
    for (int i=0; i<N; ++i) { int j=(i+1)%N;
        m.faces.push_back({i,j,j+N}); m.faces.push_back({i,j+N,i+N});
        m.faces.push_back({tC,j,i}); m.faces.push_back({bC,i+N,j+N}); }
    return m;
}

static Mesh makeStarPrism(int np=5, double Ro=0.025, double Ri=0.012, double H=0.020) {
    const int N = np*2;
    std::vector<double> px(N), py(N);
    for (int i=0; i<N; ++i) {
        double a = kPi/np*i - kPi*0.5;
        double R = (i%2==0) ? Ro : Ri;
        px[i]=R*std::cos(a); py[i]=R*std::sin(a); }
    Mesh m; m.vertices.resize(2*N+2, 3);
    for (int i=0; i<N; ++i) {
        m.vertices(i,  0)=px[i]; m.vertices(i,  1)=py[i]; m.vertices(i,  2)= H*0.5;
        m.vertices(i+N,0)=px[i]; m.vertices(i+N,1)=py[i]; m.vertices(i+N,2)=-H*0.5; }
    int tC=2*N, bC=2*N+1;
    m.vertices.row(tC) << 0.0, 0.0,  H*0.5;
    m.vertices.row(bC) << 0.0, 0.0, -H*0.5;
    m.faces.reserve(4*N);
    for (int i=0; i<N; ++i) { int j=(i+1)%N;
        m.faces.push_back({i,j,j+N}); m.faces.push_back({i,j+N,i+N});
        m.faces.push_back({tC,j,i}); m.faces.push_back({bC,i+N,j+N}); }
    return m;
}

static Mesh makeHollowCylinder(double Ro=0.030, double Ri=0.015, double H=0.040, int N=32) {
    Mesh m; m.vertices.resize(4*N, 3);
    for (int i=0; i<N; ++i) {
        double a=2*kPi*i/N, ca=std::cos(a), sa=std::sin(a);
        m.vertices(i,    0)=Ro*ca; m.vertices(i,    1)=Ro*sa; m.vertices(i,    2)= H*0.5;
        m.vertices(i+N,  0)=Ro*ca; m.vertices(i+N,  1)=Ro*sa; m.vertices(i+N,  2)=-H*0.5;
        m.vertices(i+2*N,0)=Ri*ca; m.vertices(i+2*N,1)=Ri*sa; m.vertices(i+2*N,2)= H*0.5;
        m.vertices(i+3*N,0)=Ri*ca; m.vertices(i+3*N,1)=Ri*sa; m.vertices(i+3*N,2)=-H*0.5; }
    m.faces.reserve(8*N);
    for (int i=0; i<N; ++i) { int j=(i+1)%N;
        m.faces.push_back({i,    i+N,  j+N  }); m.faces.push_back({i,    j+N,  j    });
        m.faces.push_back({i+2*N,j+2*N,j+3*N}); m.faces.push_back({i+2*N,j+3*N,i+3*N});
        m.faces.push_back({i,    j,    j+2*N}); m.faces.push_back({i,    j+2*N,i+2*N});
        m.faces.push_back({i+N,  i+3*N,j+3*N}); m.faces.push_back({i+N,  j+3*N,j+N  }); }
    return m;
}

// ===== Helpers =====

static bool insideConvex(const Eigen::Vector3d& pt, const Mesh& m, double eps=1e-6) {
    for (const auto& f : m.faces) {
        Eigen::Vector3d v0=m.vertices.row(f[0]),v1=m.vertices.row(f[1]),v2=m.vertices.row(f[2]);
        Eigen::Vector3d n=(v1-v0).cross(v2-v0); double len=n.norm();
        if (len<1e-20) continue;
        if ((pt-v0).dot(n) > eps*len) return false; }
    return true;
}

static int checkCons(const Mesh& inp, const Mesh& expanded, double eps=1e-5) {
    int fail=0;
    for (int i=0; i<inp.numVertices(); ++i) {
        Eigen::Vector3d v=inp.vertices.row(i).transpose();
        if (!insideConvex(v, expanded, eps)) ++fail;
    }
    return fail;
}

static int checkExp(const Mesh& inp, const Mesh& expanded, double d, double eps=1e-5) {
    int fail=0;
    for (const auto& f : inp.faces) {
        Eigen::Vector3d v0=inp.vertices.row(f[0]),v1=inp.vertices.row(f[1]),v2=inp.vertices.row(f[2]);
        Eigen::Vector3d n=(v1-v0).cross(v2-v0); double len=n.norm();
        if (len<1e-20) continue;
        Eigen::Vector3d probe=(v0+v1+v2)/3.0 + d*(n/len);
        if (!insideConvex(probe, expanded, eps)) ++fail;
    }
    return fail;
}

// ===== Result table =====

struct BenchRow { std::string name; int inFaces,outFaces,nV,nF,cF,eF; double volInp,vol,ms; };
static std::vector<BenchRow> g_rows;

static double meshVolume(const Mesh& m) {
    double v = 0;
    for (const auto& f : m.faces) {
        Eigen::Vector3d a=m.vertices.row(f[0]),b=m.vertices.row(f[1]),c=m.vertices.row(f[2]);
        v += a.dot(b.cross(c));
    }
    return std::abs(v)/6.0;
}

static void printTable() {
    const char* sep = "+----------------------+--------+--------+-------+------+----------+---------+------+";
    std::cout << "\n" << sep << "\n"
        << "| Shape                | InFace | OutFac | Cov%  | Exp% | VolExp   | VolRatio|  ms  |\n"
        << sep << "\n";
    for (const auto& r : g_rows) {
        double cov = r.nV>0 ? (1.0-(double)r.cF/r.nV)*100 : 100.0;
        double exp = r.nF>0 ? (1.0-(double)r.eF/r.nF)*100 : 100.0;
        double ratio = r.volInp > 0 ? r.vol / r.volInp : 0.0;
        std::cout << std::fixed
            << "| " << std::left  << std::setw(20) << r.name    << " |"
            << std::right << std::setw(7)  << r.inFaces          << " |"
            << std::right << std::setw(7)  << r.outFaces         << " |"
            << std::right << std::setprecision(1) << std::setw(6)  << cov    << " |"
            << std::right << std::setprecision(1) << std::setw(5)  << exp    << " |"
            << std::right << std::setprecision(6) << std::setw(9)  << r.vol  << " |"
            << std::right << std::setprecision(3)  << std::setw(8)  << ratio  << " |"
            << std::right << std::setprecision(0) << std::setw(5)  << r.ms   << " |\n";
    }
    std::cout << sep << "\n"
        << "  Algorithm: BoxExpander (削り出し法)  |  Cov% = vertices covered"
        << "  |  Exp% = face probes with expansion>=d\n"
        << "  VolRatio = V_expanded / V_input  (1.0 = no wasted volume; larger = more wasted)\n\n";
}

// ===== Test fixture =====

class CadShapeTest : public ::testing::Test {
protected:
    static constexpr double kD = 0.001;

    void runShape(const std::string& name, const Mesh& inp) {
        ASSERT_GT(inp.numFaces(), 0) << name << ": empty mesh";
        auto t0 = std::chrono::high_resolution_clock::now();
        BoxExpander exp;
        Mesh result = exp.expand(inp, kD);
        double ms = std::chrono::duration<double,std::milli>(
            std::chrono::high_resolution_clock::now()-t0).count();
        ASSERT_GT(result.numFaces(), 0) << name << ": expansion returned empty";

        int nV = inp.numVertices(), nF = (int)inp.faces.size();
        int cF = checkCons(inp, result, 1e-5);
        int eF = checkExp(inp, result, kD, 1e-5);

        EXPECT_EQ(cF, 0) << name << ": " << cF << "/" << nV << " vertices uncovered (Cov < 100%)";
        EXPECT_LT(nF>0?(double)eF/nF:0.0, 0.05)
            << name << ": " << eF << "/" << nF << " face probes failed (Exp < 95%)";

        try {
            std::filesystem::create_directories("stl_output");
            std::string tag = name;
            for (char& c : tag) if (!std::isalnum((unsigned char)c) && c != '-') c = '_';
            StlWriter::write("stl_output/cad_"+tag+".stl", result, name);
        } catch (...) {}

        double volInp = meshVolume(inp);
        double vol    = meshVolume(result);
        g_rows.push_back({name, nF, result.numFaces(), nV, nF, cF, eF, volInp, vol, ms});
    }
};

TEST_F(CadShapeTest, Torus) {
    runShape("Torus R60/r20", makeTorus(0.060, 0.020, 24, 16));
}
TEST_F(CadShapeTest, Gear12Tooth) {
    runShape("Gear 12-tooth", makeGear(0.060, 12, 0.020, 0.040, 6));
}
TEST_F(CadShapeTest, StarPrism5) {
    runShape("Star prism 5pt", makeStarPrism(5, 0.060, 0.025, 0.050));
}
TEST_F(CadShapeTest, HollowCylinder) {
    runShape("Hollow cylinder", makeHollowCylinder(0.060, 0.040, 0.100, 32));
}
TEST_F(CadShapeTest, ZZZ_PrintBenchmarkTable) {
    if (g_rows.empty()) GTEST_SKIP() << "No rows";
    std::cout << "\n+----------------------------------------------------------+\n"
              << "|   MeshExpander CAD Benchmark  (BoxExpander  d=1mm)       |\n"
              << "+----------------------------------------------------------+\n";
    printTable();
}
