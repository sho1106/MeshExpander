// Unit tests: ConvexDecomposer（concavity 駆動の空間ボックス分割）
//
// 検証:
//   - 凸入力は 1 ボックスに収束（concavity ≈ 0）
//   - 凹入力は複数ボックスに分割される
//   - 出力ボックスは面を含む（空ボックスは除外）
//   - regionConcavity: 凸≈0、凹>0
//   - ボックスは root AABB 内に収まる

#include <gtest/gtest.h>
#include "expander/ConvexDecomposer.hpp"
#include "expander/BoxExpander.hpp"

#include <vector>

using namespace expander;

// 軸並行ボックス（凸）
static Mesh makeBox(double s = 10.0) {
    Mesh m;
    m.vertices.resize(8, 3);
    m.vertices.row(0) = Eigen::RowVector3d(-s, -s, -s);
    m.vertices.row(1) = Eigen::RowVector3d( s, -s, -s);
    m.vertices.row(2) = Eigen::RowVector3d( s,  s, -s);
    m.vertices.row(3) = Eigen::RowVector3d(-s,  s, -s);
    m.vertices.row(4) = Eigen::RowVector3d(-s, -s,  s);
    m.vertices.row(5) = Eigen::RowVector3d( s, -s,  s);
    m.vertices.row(6) = Eigen::RowVector3d( s,  s,  s);
    m.vertices.row(7) = Eigen::RowVector3d(-s,  s,  s);
    m.faces = {
        {0,2,1},{0,3,2},   // -z
        {4,5,6},{4,6,7},   // +z
        {0,1,5},{0,5,4},   // -y
        {2,3,7},{2,7,6},   // +y
        {1,2,6},{1,6,5},   // +x
        {0,4,7},{0,7,3},   // -x
    };
    return m;
}

// L字プリズム（凹）
static Mesh makeLShapeU(double R = 10.0, double H = 20.0) {
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

static Eigen::AlignedBox3d meshAABB(const Mesh& m) {
    return Eigen::AlignedBox3d(
        m.vertices.colwise().minCoeff().transpose(),
        m.vertices.colwise().maxCoeff().transpose());
}

// 凸入力（ボックス）の concavity はほぼ 0
TEST(ConvexDecomposer, ConvexConcavityNearZero) {
    const Mesh box = makeBox();
    const double c = ConvexDecomposer::regionConcavity(box, meshAABB(box), 20.0);
    EXPECT_LT(c, 1e-6) << "凸形状の concavity は 0 のはず: " << c;
}

// 凹入力（L字）の concavity は正
TEST(ConvexDecomposer, ConcaveConcavityPositive) {
    const Mesh L = makeLShapeU();
    const double c = ConvexDecomposer::regionConcavity(L, meshAABB(L), 20.0);
    EXPECT_GT(c, 1.0) << "L字の concavity は正で十分大きいはず: " << c;
}

// 凸入力は分割されない（1 ボックス）
TEST(ConvexDecomposer, ConvexStaysSinglePiece) {
    const Mesh box = makeBox();
    ConvexDecomposer::Options opt;
    opt.maxConvexPieces = 8;
    opt.concavityTol    = 0.0;
    const auto boxes = ConvexDecomposer::partition(box, opt);
    EXPECT_EQ(boxes.size(), 1u);
}

// 凹入力は複数ボックスに分割される
TEST(ConvexDecomposer, ConcaveSplits) {
    const Mesh L = makeLShapeU();
    ConvexDecomposer::Options opt;
    opt.maxConvexPieces = 4;
    opt.concavityTol    = 0.0;
    const auto boxes = ConvexDecomposer::partition(L, opt);
    EXPECT_GE(boxes.size(), 2u);
    EXPECT_LE(boxes.size(), 4u);
}

// 出力ボックスはすべて面を含む（空ボックスは除外されている）
TEST(ConvexDecomposer, OutputBoxesContainFaces) {
    const Mesh L = makeLShapeU();
    ConvexDecomposer::Options opt;
    opt.maxConvexPieces = 8;
    opt.concavityTol    = 0.0;
    const auto boxes = ConvexDecomposer::partition(L, opt);
    ASSERT_FALSE(boxes.empty());
    for (const auto& b : boxes)
        EXPECT_FALSE(BoxExpander::collectFaces(L, b).empty())
            << "面を含まないボックスが出力された";
}

// 出力ボックスは root AABB 内に収まる
TEST(ConvexDecomposer, BoxesWithinRoot) {
    const Mesh L = makeLShapeU();
    const Eigen::AlignedBox3d root = meshAABB(L);
    ConvexDecomposer::Options opt;
    opt.maxConvexPieces = 4;
    const auto boxes = ConvexDecomposer::partition(L, opt);
    const double eps = 1e-9;
    for (const auto& b : boxes) {
        EXPECT_TRUE((b.min().array() >= root.min().array() - eps).all());
        EXPECT_TRUE((b.max().array() <= root.max().array() + eps).all());
    }
}

// maxConvexPieces=1 は分割しない
TEST(ConvexDecomposer, MaxOnePieceNoSplit) {
    const Mesh L = makeLShapeU();
    ConvexDecomposer::Options opt;
    opt.maxConvexPieces = 1;
    const auto boxes = ConvexDecomposer::partition(L, opt);
    EXPECT_EQ(boxes.size(), 1u);
}

// 空メッシュはボックスを返さない
TEST(ConvexDecomposer, EmptyMeshNoBoxes) {
    ConvexDecomposer::Options opt;
    opt.maxConvexPieces = 8;
    EXPECT_TRUE(ConvexDecomposer::partition(Mesh{}, opt).empty());
}

// concavityTol が負でも 0 にクランプされ、結果は tol=0 と一致
TEST(ConvexDecomposer, NegativeTolClampedToZero) {
    const Mesh L = makeLShapeU();
    ConvexDecomposer::Options a; a.maxConvexPieces = 4; a.concavityTol =  0.0;
    ConvexDecomposer::Options b; b.maxConvexPieces = 4; b.concavityTol = -5.0;
    EXPECT_EQ(ConvexDecomposer::partition(L, a).size(),
              ConvexDecomposer::partition(L, b).size());
}

// 巨大な maxConvexPieces でも終了し、面数（=分割可能箇所）で頭打ちになる
TEST(ConvexDecomposer, HugeMaxPiecesTerminates) {
    const Mesh L = makeLShapeU();
    ConvexDecomposer::Options opt;
    opt.maxConvexPieces = 100000;
    opt.concavityTol    = 0.0;
    const auto boxes = ConvexDecomposer::partition(L, opt);
    EXPECT_GE(boxes.size(), 2u);
    EXPECT_LE(boxes.size(), static_cast<std::size_t>(opt.maxConvexPieces));
    for (const auto& b : boxes)
        EXPECT_FALSE(BoxExpander::collectFaces(L, b).empty());
}
