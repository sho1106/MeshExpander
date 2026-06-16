#include "expander/AssemblyExpander.hpp"
#include "expander/BoxExpander.hpp"
#include "expander/ClippingEngine.hpp"
#include "expander/BoxPartitioner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace expander {

AssemblyExpander::AssemblyExpander()
    : opts_(Options{})
{}

AssemblyExpander::AssemblyExpander(Options opts)
    : opts_(opts)
{}

namespace {

// Point-in-closed-mesh test by ray casting along +X (Möller–Trumbore).
// An odd number of forward crossings means the point is inside the solid.
// Used by mergeContained to avoid absorbing a part that merely sits inside a
// parent's bounding box but is actually in the parent's hollow/cavity.
bool pointInsideMesh(const Mesh& mesh, const Eigen::Vector3d& p) {
    // Irrational, non-axis-aligned direction: avoids the ray passing exactly
    // through a shared edge/vertex of an axis-aligned mesh (which would
    // double-count crossings and misclassify the point).
    const Eigen::Vector3d dir =
        Eigen::Vector3d(1.0, 0.4142135624, 0.2718281828).normalized();
    int crossings = 0;
    for (const auto& f : mesh.faces) {
        const Eigen::Vector3d a = mesh.vertices.row(f[0]).transpose();
        const Eigen::Vector3d b = mesh.vertices.row(f[1]).transpose();
        const Eigen::Vector3d c = mesh.vertices.row(f[2]).transpose();
        const Eigen::Vector3d e1 = b - a, e2 = c - a;
        const Eigen::Vector3d pv = dir.cross(e2);
        const double det = e1.dot(pv);
        // Scale-relative parallel cull: |det| ~ parallelogram area, so compare
        // against the edge magnitudes rather than an absolute epsilon (keeps the
        // test correct for micron- and kilometre-scale meshes alike).
        if (std::abs(det) < 1e-12 * (e1.norm() * e2.norm() + 1e-300)) continue;
        const double inv = 1.0 / det;
        const Eigen::Vector3d tv = p - a;
        const double u = tv.dot(pv) * inv;
        if (u < 0.0 || u > 1.0) continue;
        const Eigen::Vector3d qv = tv.cross(e1);
        const double v = dir.dot(qv) * inv;
        if (v < 0.0 || u + v > 1.0) continue;
        const double t = e2.dot(qv) * inv;
        if (t > 1e-12) ++crossings;                   // forward hit only
    }
    return (crossings & 1) == 1;
}

} // namespace

// ---------------------------------------------------------------------------
// mergeContained()
// ---------------------------------------------------------------------------
std::vector<Mesh> AssemblyExpander::mergeContained(const std::vector<Mesh>& parts,
                                                    double tolerance)
{
    const int n = static_cast<int>(parts.size());
    if (n <= 1) return parts;

    struct BB { Eigen::Vector3d lo, hi; double vol; };
    std::vector<BB> bbs(n);
    for (int i = 0; i < n; ++i) {
        if (parts[i].empty()) {
            bbs[i] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0};
            continue;
        }
        bbs[i].lo = parts[i].vertices.colwise().minCoeff().transpose();
        bbs[i].hi = parts[i].vertices.colwise().maxCoeff().transpose();
        const Eigen::Vector3d dim = bbs[i].hi - bbs[i].lo;
        bbs[i].vol = dim.x() * dim.y() * dim.z();
    }

    std::vector<int> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
              [&](int a, int b) { return bbs[a].vol < bbs[b].vol; });

    std::vector<Mesh> result = parts;
    std::vector<bool> absorbed(n, false);

    for (int oi = 0; oi < n; ++oi) {
        const int j = order[oi];
        if (absorbed[j] || result[j].empty()) continue;

        // Find the smallest-volume part whose AABB fully contains part j's AABB.
        int bestContainer = -1;
        double bestVol    = std::numeric_limits<double>::infinity();

        for (int i = 0; i < n; ++i) {
            if (i == j || absorbed[i] || result[i].empty()) continue;
            if (bbs[i].vol <= bbs[j].vol) continue;

            const bool aabbContained =
                (bbs[i].lo.array() <= bbs[j].lo.array() + tolerance).all() &&
                (bbs[i].hi.array() >= bbs[j].hi.array() - tolerance).all();
            if (!aabbContained || bbs[i].vol >= bestVol) continue;

            // AABB containment is necessary but not sufficient: a part sitting in
            // the hollow/cavity of a parent (annulus, C-shape) has its AABB inside
            // the parent's but is NOT solidly contained. Require the child's
            // centroid to actually lie inside the parent mesh before absorbing.
            const Eigen::Vector3d centroid =
                result[j].vertices.colwise().mean().transpose();
            if (!pointInsideMesh(result[i], centroid)) continue;

            bestContainer = i;
            bestVol = bbs[i].vol;
        }

        if (bestContainer < 0) continue;

        appendMesh(result[bestContainer], result[j]);
        absorbed[j] = true;
    }

    std::vector<Mesh> out;
    out.reserve(n);
    for (int i = 0; i < n; ++i)
        if (!absorbed[i]) out.push_back(std::move(result[i]));
    return out;
}

// ---------------------------------------------------------------------------
// expandPart()
// ---------------------------------------------------------------------------
Mesh AssemblyExpander::expandPart(const Mesh& part, double d) const
{
    if (part.empty() || part.faces.empty()) return {};

    BoxExpander exp(opts_.faceNormalMergeDeg);

    // 従来挙動: 単一凸ピース（分解なし）。
    if (opts_.maxConvexPieces <= 1 && opts_.concavityTol <= 0.0)
        return exp.expand(part, d);

    // 凹対応: concavity 駆動の空間ボックス分割 → ボックス毎に削り出し → 連結。
    // concavityTol 単独指定（maxConvexPieces 未指定）でも分割できるよう、
    // ピース上限が未設定なら既定上限を与える。
    constexpr int kDefaultMaxPiecesForTol = 64;
    BoxPartitioner::Options dopt;
    dopt.concavityTol       = opts_.concavityTol;
    dopt.maxConvexPieces    = (opts_.maxConvexPieces > 1)
                                  ? opts_.maxConvexPieces
                                  : kDefaultMaxPiecesForTol;
    dopt.faceNormalMergeDeg = opts_.faceNormalMergeDeg;

    const std::vector<Eigen::AlignedBox3d> boxes =
        BoxPartitioner::partition(part, dopt);
    if (boxes.size() <= 1)
        return exp.expand(part, d);   // 分割されなかった（凸）→ 単一ピース

    std::vector<Mesh> pieces;
    pieces.reserve(boxes.size());
    for (const auto& box : boxes) {
        Mesh m = exp.expand(box, part, d);   // 全メッシュに対してボックスを局所削り出し
        if (m.empty()) {
            // 削り出しが退化した場合でも保守性を保つため、未カットの膨張ボックスを出力。
            const Eigen::AlignedBox3d expandedBox(
                box.min() - Eigen::Vector3d::Constant(d),
                box.max() + Eigen::Vector3d::Constant(d));
            m = ClippingEngine::clip(expandedBox, {});
        }
        if (!m.empty()) pieces.push_back(std::move(m));
    }
    return mergeMeshes(pieces);
}

// ---------------------------------------------------------------------------
// appendMesh() — concatenate src into dst in place, offsetting face indices
// ---------------------------------------------------------------------------
void AssemblyExpander::appendMesh(Mesh& dst, const Mesh& src)
{
    if (src.empty()) return;
    const int vOffset = dst.numVertices();

    Eigen::MatrixXd newV(vOffset + src.numVertices(), 3);
    newV.topRows(vOffset)              = dst.vertices;
    newV.bottomRows(src.numVertices()) = src.vertices;
    dst.vertices = std::move(newV);

    dst.faces.reserve(dst.numFaces() + src.numFaces());
    for (const auto& f : src.faces)
        dst.faces.push_back({f[0] + vOffset, f[1] + vOffset, f[2] + vOffset});
}

// ---------------------------------------------------------------------------
// mergeMeshes()
// ---------------------------------------------------------------------------
Mesh AssemblyExpander::mergeMeshes(const std::vector<Mesh>& meshes)
{
    int totalV = 0, totalF = 0;
    for (const auto& m : meshes) { totalV += m.numVertices(); totalF += m.numFaces(); }
    if (totalV == 0) return {};

    Mesh out;
    out.vertices.resize(totalV, 3);
    out.faces.reserve(totalF);

    int vOff = 0;
    for (const auto& m : meshes) {
        out.vertices.middleRows(vOff, m.numVertices()) = m.vertices;
        for (const auto& f : m.faces)
            out.faces.push_back({f[0] + vOff, f[1] + vOff, f[2] + vOff});
        vOff += m.numVertices();
    }
    return out;
}

// ---------------------------------------------------------------------------
// expand() / expandMerged()
// ---------------------------------------------------------------------------
std::vector<Mesh> AssemblyExpander::expand(const std::vector<Mesh>& parts,
                                           double d) const
{
    std::vector<Mesh> result;
    result.reserve(parts.size());
    for (const auto& p : parts)
        result.push_back(expandPart(p, d));
    return result;
}

Mesh AssemblyExpander::expandMerged(const std::vector<Mesh>& parts, double d) const
{
    return mergeMeshes(expand(parts, d));
}

} // namespace expander
