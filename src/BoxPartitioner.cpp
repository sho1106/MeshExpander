#include "expander/BoxPartitioner.hpp"
#include "expander/BoxExpander.hpp"
#include "expander/MathUtils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace expander {

namespace {

// 領域の評価結果: 最大ギャップ値と、それを生んだ分割軸・分割位置。
struct RegionEval {
    double concavity = 0.0;
    int    splitAxis = 0;
    double splitPos  = 0.0;
    bool   splittable = false;
};

// ボックス領域の concavity と、分割軸・分割位置を計算する。
//   box に重なる面を集め、各面 f の法線を最近傍マージ方向 m* に割り当て、
//   gap_f = max(局所頂点·m*) - max(face頂点·m*) の最大値を concavity とする。
//   分割は最悪面（ポケットを定義する面）の位置で、最悪方向に最も沿う軸で行う。
RegionEval evalRegion(const Mesh&                mesh,
                      const Eigen::AlignedBox3d& box,
                      double                     faceNormalMergeDeg)
{
    RegionEval res;

    const std::vector<int> faceIdx = BoxExpander::collectFaces(mesh, box);
    if (faceIdx.empty()) return res;

    // 1. 各面の単位法線（縮退面は除外）。validFaces は normals と並行。
    std::vector<int> validFaces;
    const std::vector<Eigen::Vector3d> normals =
        BoxExpander::faceNormals(mesh, faceIdx, &validFaces);
    if (normals.empty()) return res;

    // 2. 近似平行法線をマージした方向集合
    const auto dirs = math::mergeDirections(normals, faceNormalMergeDeg);

    // 3. 各マージ方向の support D_m = max(局所面頂点·m)
    std::vector<double> D(dirs.size());
    for (std::size_t di = 0; di < dirs.size(); ++di)
        D[di] = BoxExpander::maxSupport(mesh, faceIdx, dirs[di]);

    // 4. 各面のギャップ = 最近傍マージ方向の D - 面 support。最大値が concavity。
    //    最悪の面（ポケットを定義する面）も記録する。
    Eigen::Vector3d worstDir  = Eigen::Vector3d::UnitX();
    int             worstFace = -1;
    for (std::size_t i = 0; i < validFaces.size(); ++i) {
        const Eigen::Vector3d& nf = normals[i];

        int    best    = 0;
        double bestDot = -2.0;
        for (std::size_t di = 0; di < dirs.size(); ++di) {
            const double dt = nf.dot(dirs[di]);
            if (dt > bestDot) { bestDot = dt; best = static_cast<int>(di); }
        }

        const Eigen::Vector3d& m = dirs[best];
        const auto& f = mesh.faces[validFaces[i]];
        double sf = -std::numeric_limits<double>::infinity();
        for (int k = 0; k < 3; ++k)
            sf = std::max(sf, mesh.vertices.row(f[k]).dot(m));

        const double gap = D[best] - sf;
        if (gap > res.concavity) {
            res.concavity = gap;
            worstDir      = m;
            worstFace     = validFaces[i];
        }
    }
    if (worstFace < 0) return res;

    // 5. 分割軸 = 最悪方向に最も沿う軸。
    //    分割位置 = 最悪面（ポケット面）の重心位置。ここで割ると、ポケットの
    //    奥にある空領域が面なしボックスとして分離され出力から除外される。
    worstDir.cwiseAbs().maxCoeff(&res.splitAxis);

    const auto& wf = mesh.faces[worstFace];
    double pos = (mesh.vertices(wf[0], res.splitAxis) +
                  mesh.vertices(wf[1], res.splitAxis) +
                  mesh.vertices(wf[2], res.splitAxis)) / 3.0;

    // 分割位置がボックス境界に寄りすぎる場合は中心にフォールバック。
    const double lo = box.min()[res.splitAxis];
    const double hi = box.max()[res.splitAxis];
    const double w  = hi - lo;
    if (w <= math::kEpsilon) { res.splittable = false; return res; }
    if (pos <= lo + 1e-6 * w || pos >= hi - 1e-6 * w)
        pos = 0.5 * (lo + hi);

    res.splitPos   = pos;
    res.splittable = (pos > lo + 1e-9 * w && pos < hi - 1e-9 * w);
    return res;
}

} // namespace

// ---------------------------------------------------------------------------
// regionConcavity()
// ---------------------------------------------------------------------------
double BoxPartitioner::regionConcavity(const Mesh&                mesh,
                                       const Eigen::AlignedBox3d& box,
                                       double                     faceNormalMergeDeg)
{
    return evalRegion(mesh, box, faceNormalMergeDeg).concavity;
}

// ---------------------------------------------------------------------------
// partition()
// ---------------------------------------------------------------------------
std::vector<Eigen::AlignedBox3d> BoxPartitioner::partition(const Mesh&    mesh,
                                                           const Options& opt)
{
    std::vector<Eigen::AlignedBox3d> result;
    if (mesh.empty() || mesh.faces.empty()) return result;

    const int    maxPieces = std::max(1, opt.maxConvexPieces);
    const double tol        = std::max(0.0, opt.concavityTol);

    const Eigen::AlignedBox3d root(
        mesh.vertices.colwise().minCoeff().transpose(),
        mesh.vertices.colwise().maxCoeff().transpose());

    struct Region {
        Eigen::AlignedBox3d box;
        RegionEval          ev;
    };
    auto make = [&](const Eigen::AlignedBox3d& b) {
        return Region{ b, evalRegion(mesh, b, opt.faceNormalMergeDeg) };
    };

    std::vector<Region> active;
    active.push_back(make(root));

    // 最も concavity の大きい領域を許容値を超える限り二分割する。
    while (static_cast<int>(active.size()) < maxPieces) {
        int    best  = -1;
        double bestC = tol;
        for (int i = 0; i < static_cast<int>(active.size()); ++i)
            if (active[i].ev.splittable && active[i].ev.concavity > bestC) {
                bestC = active[i].ev.concavity;
                best  = i;
            }
        if (best < 0) break;   // 全領域が許容値以内、または分割不能

        const Eigen::AlignedBox3d b = active[best].box;
        const int    axis = active[best].ev.splitAxis;
        const double pos  = active[best].ev.splitPos;

        Eigen::Vector3d loMax = b.max();  loMax[axis] = pos;
        Eigen::Vector3d hiMin = b.min();  hiMin[axis] = pos;
        const Eigen::AlignedBox3d lo(b.min(), loMax);
        const Eigen::AlignedBox3d hi(hiMin, b.max());

        active[best] = make(lo);
        active.push_back(make(hi));
    }

    // 面を含むボックスのみ出力（凹ノッチの外側の空ボックスは除外）。
    result.reserve(active.size());
    for (const auto& r : active)
        if (!BoxExpander::collectFaces(mesh, r.box).empty())
            result.push_back(r.box);
    return result;
}

} // namespace expander
