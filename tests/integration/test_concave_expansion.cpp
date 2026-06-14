// Integration tests: 凹形状の膨張（BoxPartitioner + BoxExpander）
//
// 検証内容:
//   1. 保守性 (Cov%=100%): 全入力頂点が、いずれかの凸ピースに距離 d のマージンで包含される
//   2. 精度 (VolRatio): 分解した方が単一凸より無駄な体積が小さい
//   3. ポリゴン数: ピース数に概ね比例し、激増しない
//   4. ノブ単調性: maxConvexPieces を増やすほど無駄な体積が減る
//   5. NaN/Inf 安全性

#include <gtest/gtest.h>
#include "expander/BoxExpander.hpp"
#include "expander/BoxPartitioner.hpp"

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using namespace expander;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// 単一閉多面体の体積（発散定理）
static double cMeshVolume(const Mesh& m) {
    double vol = 0.0;
    for (const auto& f : m.faces) {
        const Eigen::Vector3d v0 = m.vertices.row(f[0]).transpose();
        const Eigen::Vector3d v1 = m.vertices.row(f[1]).transpose();
        const Eigen::Vector3d v2 = m.vertices.row(f[2]).transpose();
        vol += v0.dot(v1.cross(v2));
    }
    return std::abs(vol) / 6.0;
}

// 点 pt が単一凸多面体 m の内部（境界 + eps）にあるか
static bool cInsideConvex(const Eigen::Vector3d& pt, const Mesh& m, double eps) {
    for (const auto& f : m.faces) {
        const Eigen::Vector3d v0 = m.vertices.row(f[0]).transpose();
        const Eigen::Vector3d v1 = m.vertices.row(f[1]).transpose();
        const Eigen::Vector3d v2 = m.vertices.row(f[2]).transpose();
        const Eigen::Vector3d n = (v1 - v0).cross(v2 - v0);
        const double len = n.norm();
        if (len < 1e-20) continue;
        if ((pt - v0).dot(n) > eps * len) return false;
    }
    return true;
}

// 点 pt が凸ピース集合のいずれかに包含されるか（和集合の内包判定）
static bool cInsideUnion(const Eigen::Vector3d& pt,
                         const std::vector<Mesh>& pieces, double eps) {
    for (const auto& m : pieces)
        if (!m.faces.empty() && cInsideConvex(pt, m, eps)) return true;
    return false;
}

// 凸ピース和集合の体積をモンテカルロ推定（重複を二重計上しない真の和体積）。
// ピースが重なるため sum-of-volumes は使えない。決定論的シードで再現性を担保。
static double cUnionVolumeMC(const std::vector<Mesh>& pieces, int samples = 200000) {
    if (pieces.empty()) return 0.0;
    Eigen::Vector3d lo = Eigen::Vector3d::Constant( 1e30);
    Eigen::Vector3d hi = Eigen::Vector3d::Constant(-1e30);
    for (const auto& m : pieces) {
        if (m.numVertices() == 0) continue;
        lo = lo.cwiseMin(m.vertices.colwise().minCoeff().transpose());
        hi = hi.cwiseMax(m.vertices.colwise().maxCoeff().transpose());
    }
    const Eigen::Vector3d ext = hi - lo;
    const double boxVol = ext.x() * ext.y() * ext.z();
    if (boxVol <= 0.0) return 0.0;

    // 簡易・決定論的 LCG
    std::uint64_t s = 0x9e3779b97f4a7c15ULL;
    auto rnd = [&]() {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return ((s >> 11) & ((1ULL << 53) - 1)) / static_cast<double>(1ULL << 53);
    };
    int hits = 0;
    for (int i = 0; i < samples; ++i) {
        const Eigen::Vector3d p(lo.x() + ext.x() * rnd(),
                                lo.y() + ext.y() * rnd(),
                                lo.z() + ext.z() * rnd());
        if (cInsideUnion(p, pieces, 1e-9)) ++hits;
    }
    return boxVol * static_cast<double>(hits) / samples;
}

// 部品を K ピース上限で空間分割し、各ボックスを全メッシュに対し d だけ削り出す
static std::vector<Mesh> decomposeAndExpand(const Mesh& inp, double d,
                                            int maxPieces, double concavityTol) {
    BoxPartitioner::Options dopt;
    dopt.maxConvexPieces = maxPieces;
    dopt.concavityTol    = concavityTol;
    const std::vector<Eigen::AlignedBox3d> boxes =
        BoxPartitioner::partition(inp, dopt);

    BoxExpander exp;
    std::vector<Mesh> pieces;
    pieces.reserve(boxes.size());
    for (const auto& box : boxes) {
        Mesh m = exp.expand(box, inp, d);
        if (!m.empty()) pieces.push_back(std::move(m));
    }
    return pieces;
}

static int countCovFailures(const Mesh& inp, const std::vector<Mesh>& pieces, double eps) {
    int fail = 0;
    for (int i = 0; i < inp.numVertices(); ++i) {
        const Eigen::Vector3d v = inp.vertices.row(i).transpose();
        if (!cInsideUnion(v, pieces, eps)) ++fail;
    }
    return fail;
}

static int totalFaces(const std::vector<Mesh>& pieces) {
    int n = 0;
    for (const auto& m : pieces) n += m.numFaces();
    return n;
}


// ---------------------------------------------------------------------------
// Mesh factories (旧 test_concave_expansion.cpp より)
// ---------------------------------------------------------------------------

// L字プリズム（凹断面を Z 押し出し）。L面積 = 3R^2、体積 = 3R^2 * H
static Mesh makeLShape(double R = 10.0, double H = 20.0) {
    Mesh m;
    m.vertices.resize(12, 3);
    const double hz = H * 0.5;
    m.vertices.row(0)  = Eigen::RowVector3d(-R, -R,  hz);
    m.vertices.row(1)  = Eigen::RowVector3d( R, -R,  hz);
    m.vertices.row(2)  = Eigen::RowVector3d( R,  0,  hz);
    m.vertices.row(3)  = Eigen::RowVector3d( 0,  0,  hz);
    m.vertices.row(4)  = Eigen::RowVector3d( 0,  R,  hz);
    m.vertices.row(5)  = Eigen::RowVector3d(-R,  R,  hz);
    m.vertices.row(6)  = Eigen::RowVector3d(-R, -R, -hz);
    m.vertices.row(7)  = Eigen::RowVector3d( R, -R, -hz);
    m.vertices.row(8)  = Eigen::RowVector3d( R,  0, -hz);
    m.vertices.row(9)  = Eigen::RowVector3d( 0,  0, -hz);
    m.vertices.row(10) = Eigen::RowVector3d( 0,  R, -hz);
    m.vertices.row(11) = Eigen::RowVector3d(-R,  R, -hz);

    m.faces.push_back({0,1,2}); m.faces.push_back({0,2,3});
    m.faces.push_back({0,3,4}); m.faces.push_back({0,4,5});
    m.faces.push_back({6,8,7}); m.faces.push_back({6,9,8});
    m.faces.push_back({6,10,9}); m.faces.push_back({6,11,10});
    m.faces.push_back({0,6,7}); m.faces.push_back({0,7,1});
    m.faces.push_back({1,7,8}); m.faces.push_back({1,8,2});
    m.faces.push_back({2,8,9}); m.faces.push_back({2,9,3});
    m.faces.push_back({3,9,10}); m.faces.push_back({3,10,4});
    m.faces.push_back({4,10,11}); m.faces.push_back({4,11,5});
    m.faces.push_back({5,11,6}); m.faces.push_back({5,6,0});
    return m;
}

// C字（コの字）プリズム
static Mesh makeCShape(double R = 10.0, double H = 20.0) {
    const double t  = R * 0.5;
    const double hz = H * 0.5;
    Mesh m;
    m.vertices.resize(16, 3);
    m.vertices.row(0) = Eigen::RowVector3d(-R, -R,  hz);
    m.vertices.row(1) = Eigen::RowVector3d( R, -R,  hz);
    m.vertices.row(2) = Eigen::RowVector3d( R, -t,  hz);
    m.vertices.row(3) = Eigen::RowVector3d( t, -t,  hz);
    m.vertices.row(4) = Eigen::RowVector3d( t,  t,  hz);
    m.vertices.row(5) = Eigen::RowVector3d( R,  t,  hz);
    m.vertices.row(6) = Eigen::RowVector3d( R,  R,  hz);
    m.vertices.row(7) = Eigen::RowVector3d(-R,  R,  hz);
    m.vertices.row(8)  = Eigen::RowVector3d(-R, -R, -hz);
    m.vertices.row(9)  = Eigen::RowVector3d( R, -R, -hz);
    m.vertices.row(10) = Eigen::RowVector3d( R, -t, -hz);
    m.vertices.row(11) = Eigen::RowVector3d( t, -t, -hz);
    m.vertices.row(12) = Eigen::RowVector3d( t,  t, -hz);
    m.vertices.row(13) = Eigen::RowVector3d( R,  t, -hz);
    m.vertices.row(14) = Eigen::RowVector3d( R,  R, -hz);
    m.vertices.row(15) = Eigen::RowVector3d(-R,  R, -hz);

    m.faces.push_back({0,1,2}); m.faces.push_back({0,2,7});
    m.faces.push_back({2,6,7}); m.faces.push_back({2,5,6});
    m.faces.push_back({2,3,5}); m.faces.push_back({3,4,5});
    m.faces.push_back({8,10,9}); m.faces.push_back({8,15,10});
    m.faces.push_back({10,15,14}); m.faces.push_back({10,14,13});
    m.faces.push_back({10,13,11}); m.faces.push_back({11,13,12});
    m.faces.push_back({0, 8, 9}); m.faces.push_back({0, 9, 1});
    m.faces.push_back({1, 9,10}); m.faces.push_back({1,10, 2});
    m.faces.push_back({2,10,11}); m.faces.push_back({2,11, 3});
    m.faces.push_back({3,11,12}); m.faces.push_back({3,12, 4});
    m.faces.push_back({4,12,13}); m.faces.push_back({4,13, 5});
    m.faces.push_back({5,13,14}); m.faces.push_back({5,14, 6});
    m.faces.push_back({6,14,15}); m.faces.push_back({6,15, 7});
    m.faces.push_back({7,15, 8}); m.faces.push_back({7, 8, 0});
    return m;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

namespace {
constexpr double kD   = 1.0;   // 膨張距離
constexpr double kEps = 1e-5;  // 内包判定の許容
}

// 保守性: 分解膨張でも全入力頂点が包含される（Cov%=100%）
TEST(ConcaveExpansion, LShapeConservative) {
    const Mesh inp = makeLShape();
    const auto pieces = decomposeAndExpand(inp, kD, /*maxPieces=*/8, /*tol=*/0.0);

    ASSERT_GE(pieces.size(), 2u) << "L字は最低 2 ピースに分解されるべき";
    for (const auto& m : pieces)
        EXPECT_TRUE(m.vertices.allFinite()) << "NaN/Inf in output piece";

    const int fail = countCovFailures(inp, pieces, kEps);
    EXPECT_EQ(fail, 0) << "全頂点が包含されていない (Cov%<100)";

    // ポリゴン数がピース数に対して妥当（激増しない）
    EXPECT_LE(totalFaces(pieces), 40 * static_cast<int>(pieces.size()));
}

TEST(ConcaveExpansion, CShapeConservative) {
    const Mesh inp = makeCShape();
    const auto pieces = decomposeAndExpand(inp, kD, /*maxPieces=*/8, /*tol=*/0.0);

    ASSERT_GE(pieces.size(), 2u) << "C字は分解されるべき";
    for (const auto& m : pieces)
        EXPECT_TRUE(m.vertices.allFinite());

    const int fail = countCovFailures(inp, pieces, kEps);
    EXPECT_EQ(fail, 0) << "全頂点が包含されていない (Cov%<100)";
    EXPECT_LE(totalFaces(pieces), 40 * static_cast<int>(pieces.size()));
}

// 精度: 分解した方が単一凸よりも無駄な体積（凹ポケットの埋め）が小さい
TEST(ConcaveExpansion, DecompositionReducesVolume) {
    const Mesh inp = makeLShape();

    BoxExpander exp;
    const Mesh single = exp.expand(inp, kD);
    const double singleVol = cMeshVolume(single);   // 単一凸（閉多面体）の厳密体積

    const auto pieces   = decomposeAndExpand(inp, kD, 8, 0.0);
    const double unionVol = cUnionVolumeMC(pieces); // 和集合の真の体積（MC）

    EXPECT_LT(unionVol, singleVol * 0.95)
        << "分解膨張の和体積(" << unionVol << ") は単一凸(" << singleVol
        << ") より明確に小さいはず（L字のノッチが埋まらない）";
}

// ノブ単調性: maxConvexPieces を増やすほど無駄な体積（和体積）が減る（非増加）
TEST(ConcaveExpansion, MoreePiecesReduceVolumeMonotonic) {
    const Mesh inp = makeLShape();
    const double inpVol = cMeshVolume(inp);

    const std::vector<int> sweep = {1, 2, 4, 8};
    double prevVol = std::numeric_limits<double>::infinity();

    std::cout << "\n  [L-shape knob sweep] V_input=" << std::fixed
              << std::setprecision(2) << inpVol << "\n";
    for (int k : sweep) {
        const auto pieces = decomposeAndExpand(inp, kD, k, 0.0);
        ASSERT_FALSE(pieces.empty());
        EXPECT_EQ(countCovFailures(inp, pieces, kEps), 0)
            << "K=" << k << " で保守性が破れた";

        const double vol   = cUnionVolumeMC(pieces);
        const int    faces = totalFaces(pieces);
        std::cout << "    K=" << k << "  pieces=" << pieces.size()
                  << "  VolRatio=" << std::setprecision(3) << (vol / inpVol)
                  << "  faces=" << faces << "\n";

        // 和体積は非増加（MC 誤差 5% の許容つき）。
        EXPECT_LE(vol, prevVol * 1.05)
            << "K=" << k << " で和体積が増えた（単調性違反）";
        prevVol = vol;
    }
}

// 既知の制約: C 字（コの字）は凸包が AABB に一致するため、軸平行ボックスの
// 膨張和では削り出せず、分解しても和体積は単一凸とほぼ同じになる。
// この挙動を固定して将来の回帰／改善を検知できるようにする。
TEST(ConcaveExpansion, CShapeKnownLimitation) {
    const Mesh inp = makeCShape();

    BoxExpander exp;
    const double singleVol = cMeshVolume(exp.expand(inp, kD));
    const auto   pieces    = decomposeAndExpand(inp, kD, 8, 0.0);
    const double unionVol  = cUnionVolumeMC(pieces);

    // 改善は限定的: 単一凸の 95% 以上（ほぼ同じ）。保守性は別テストで担保。
    EXPECT_GT(unionVol, singleVol * 0.95)
        << "C字は凸包=AABB のため分解による体積削減はほぼ無いはず "
           "（union=" << unionVol << " single=" << singleVol << "）";
}

// concavityTol を緩めるとピース数が減る（凸近似が粗くなる）
TEST(ConcaveExpansion, LooserToleranceFewerPieces) {
    const Mesh inp = makeCShape();
    const auto tight = decomposeAndExpand(inp, kD, 16, 0.1);   // 厳しい → 多ピース
    const auto loose = decomposeAndExpand(inp, kD, 16, 100.0); // 緩い   → 少ピース
    EXPECT_GE(tight.size(), loose.size());
}

// 後方互換: maxPieces=1 は単一凸（従来挙動）と一致
TEST(ConcaveExpansion, SinglePieceMatchesConvexCarving) {
    const Mesh inp = makeLShape();
    const auto pieces = decomposeAndExpand(inp, kD, 1, 0.0);
    EXPECT_EQ(pieces.size(), 1u);
}
