#include "expander/AssemblyExpander.hpp"
#include "expander/BoxExpander.hpp"

#include <algorithm>
#include <limits>
#include <numeric>

namespace expander {

AssemblyExpander::AssemblyExpander(Options opts)
    : opts_(opts)
{}

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

        int bestContainer = -1;
        double bestVol    = std::numeric_limits<double>::infinity();

        for (int i = 0; i < n; ++i) {
            if (i == j || absorbed[i] || result[i].empty()) continue;
            if (bbs[i].vol <= bbs[j].vol) continue;

            bool contained =
                (bbs[i].lo.array() <= bbs[j].lo.array() + tolerance).all() &&
                (bbs[i].hi.array() >= bbs[j].hi.array() - tolerance).all();

            if (contained && bbs[i].vol < bestVol) {
                bestContainer = i;
                bestVol = bbs[i].vol;
            }
        }

        if (bestContainer < 0) continue;

        Mesh& dst = result[bestContainer];
        const Mesh& src = result[j];
        const int vOffset = dst.numVertices();

        Eigen::MatrixXd newV(vOffset + src.numVertices(), 3);
        newV.topRows(vOffset)              = dst.vertices;
        newV.bottomRows(src.numVertices()) = src.vertices;
        dst.vertices = std::move(newV);

        dst.faces.reserve(dst.numFaces() + src.numFaces());
        for (const auto& f : src.faces)
            dst.faces.push_back({f[0] + vOffset, f[1] + vOffset, f[2] + vOffset});

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
    return exp.expand(part, d);
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
