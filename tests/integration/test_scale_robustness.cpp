// Scale-robustness: the carving path uses absolute tolerances (kOnPlaneEps,
// kSafetyMargin), so verify conservativeness (Cov% = 100%) holds across a wide
// range of model scales — from micrometers to kilometers — with d scaled
// proportionally. Cov% is checked with a scale-relative epsilon so the test is
// fair at every scale.

#include <gtest/gtest.h>
#include "expander/BoxExpander.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <vector>

using namespace expander;

namespace {

// A cube of half-extent `h`, rotated off-axis so faces are not axis-aligned
// (this actually exercises the face-normal carving rather than the box faces).
Mesh makeRotatedCube(double h) {
    Mesh m;
    m.vertices.resize(8, 3);
    int idx = 0;
    Eigen::AngleAxisd rot(0.6, Eigen::Vector3d(1, 2, 3).normalized());
    for (int sx : {-1, 1})
        for (int sy : {-1, 1})
            for (int sz : {-1, 1}) {
                Eigen::Vector3d p(sx * h, sy * h, sz * h);
                m.vertices.row(idx++) = (rot * p).transpose();
            }
    // 12 triangles over the 8 corners (cube connectivity in this ordering:
    // bit pattern x,y,z → index = 4*sx_bit + 2*sy_bit + sz_bit)
    auto V = [](int x, int y, int z) { return 4 * x + 2 * y + z; };
    int q[6][4] = {
        {V(0,0,0),V(0,1,0),V(1,1,0),V(1,0,0)},  // z-
        {V(0,0,1),V(1,0,1),V(1,1,1),V(0,1,1)},  // z+
        {V(0,0,0),V(1,0,0),V(1,0,1),V(0,0,1)},  // y-
        {V(0,1,0),V(0,1,1),V(1,1,1),V(1,1,0)},  // y+
        {V(0,0,0),V(0,0,1),V(0,1,1),V(0,1,0)},  // x-
        {V(1,0,0),V(1,1,0),V(1,1,1),V(1,0,1)},  // x+
    };
    for (auto& f : q) {
        m.faces.push_back({f[0], f[1], f[2]});
        m.faces.push_back({f[0], f[2], f[3]});
    }
    return m;
}

bool insideConvex(const Eigen::Vector3d& pt, const Mesh& m, double eps) {
    for (const auto& f : m.faces) {
        Eigen::Vector3d v0 = m.vertices.row(f[0]), v1 = m.vertices.row(f[1]),
                        v2 = m.vertices.row(f[2]);
        Eigen::Vector3d n = (v1 - v0).cross(v2 - v0);
        double len = n.norm();
        if (len < 1e-300) continue;
        if ((pt - v0).dot(n) > eps * len) return false;
    }
    return true;
}

} // namespace

TEST(ScaleRobustness, ConservativeAcrossScales) {
    // half-extent in model units: 1µm, 1mm, 1m, 1km (treating base unit as m)
    const std::vector<double> scales = {1e-6, 1e-3, 1.0, 1e3};
    for (double h : scales) {
        Mesh inp = makeRotatedCube(h);
        double d = 0.1 * h;                 // expansion proportional to size
        Mesh out = BoxExpander().expand(inp, d);
        ASSERT_GT(out.numFaces(), 0) << "scale h=" << h << " produced empty output";

        // scale-relative containment tolerance
        const double eps = 1e-7;            // relative to face-normal length * scale
        int fail = 0;
        for (int i = 0; i < inp.numVertices(); ++i) {
            Eigen::Vector3d v = inp.vertices.row(i).transpose();
            if (!insideConvex(v, out, eps * h)) ++fail;
        }
        double cov = 100.0 * (1.0 - double(fail) / inp.numVertices());
        std::cout << "  scale h=" << h << "  d=" << d
                  << "  Cov%=" << cov << "  outFaces=" << out.numFaces() << "\n";
        EXPECT_EQ(fail, 0) << "non-conservative at scale h=" << h
                           << " (" << fail << " vertices outside)";
    }
}
