#pragma once
// ---------------------------------------------------------------------------
// BoxPartitioner — concavity 駆動の適応的空間ボックス分割
//
// 凹メッシュの AABB を、凹さ(concavity)が大きい領域だけを選んで再帰的に
// 軸平行ボックスへ二分割する。**返すのは凸包ではなく AABB の集合**で、
// 各ボックスを後段の BoxExpander::expand(box, mesh, d) が削り出して凸ピースにする。
// ボックス和をとることで、凹形状でもポケットを埋めずに膨張モデルを生成できる。
//
// ボクセルグリッドは使わない。分割は concavity が許容値を超える領域に集中するため、
// 少数ボックス（maxConvexPieces で上限）で済み、resolution パラメータも不要。
//
// 核心アイデア（concavity の測り方）:
//   ボックスに重なる面 f（法線 n_f）に対し、削り出しの半空間境界は
//   D = max(ボックス局所の面頂点·n_f)。面ローカル支持値 s_f = max(face頂点·n_f)。
//   gap_f = D - s_f は「面 f の前方が削り出しで埋まる量」＝凹による過膨張。
//   領域の concavity = max_f gap_f。これが許容値以下になるまで分割する。
//
// 保守性 (Cov%=100%):
//   ボックス群は AABB を隙間なくタイル分割する。入力頂点 v を含むリーフボックスには
//   v を持つ面が必ず重なる（面の三角形 AABB が v を含む）ため、その面が v を局所頂点
//   として半空間に取り込み、expand(box, mesh, d) が v を距離 d のマージンで包含する。
//   面を 1 つも含まないボックス（凹ノッチの外側）は出力から除外する（頂点を含む
//   ボックスは必ず面を持つため、除外しても保守性は壊れない）。
// ---------------------------------------------------------------------------

#include "expander/Mesh.hpp"

#include <Eigen/Geometry>
#include <vector>

namespace expander {

class BoxPartitioner {
public:
    struct Options {
        // 分割を止める concavity 許容値（ワールド単位）。
        // 全領域の concavity がこれ以下になれば分割終了。
        double concavityTol      = 0.0;
        // 出力ボックス数の上限。1 なら分割せず単一ボックス（=入力 AABB）。
        int    maxConvexPieces   = 1;
        // 近似平行な面法線をマージする閾値（度）。
        double faceNormalMergeDeg = 20.0;
    };

    // メッシュの AABB を concavity 駆動で少数の軸平行ボックスへ分割する。
    // 面を含むボックスのみ返す（凹ノッチの外側の空ボックスは除外）。
    // 返ったボックスを BoxExpander::expand(box, mesh, d) で削り出して和をとる。
    static std::vector<Eigen::AlignedBox3d> partition(const Mesh& mesh,
                                                      const Options& opt);

    // 指定ボックス領域の concavity（核心アイデアのギャップ式の最大値）。
    // 凸領域では 0、凹が深いほど大きくなる。テスト・診断用に公開。
    static double regionConcavity(const Mesh&                mesh,
                                  const Eigen::AlignedBox3d& box,
                                  double                     faceNormalMergeDeg);
};

} // namespace expander
