#include <gtest/gtest.h>
#include "expander/MathUtils.hpp"

using namespace expander::math;

// ---------------------------------------------------------------------------
// Normalization
// ---------------------------------------------------------------------------
TEST(MathUtils, NormalizationCentersVertices) {
    Eigen::MatrixXd V(4, 3);
    V << 0, 0, 0,
         2, 0, 0,
         2, 2, 0,
         0, 2, 0;

    auto info = computeNormalization(V);
    EXPECT_NEAR(info.center.x(), 1.0, 1e-12);
    EXPECT_NEAR(info.center.y(), 1.0, 1e-12);
    EXPECT_NEAR(info.center.z(), 0.0, 1e-12);
}

TEST(MathUtils, NormalizationRangeIsMinusOneToOne) {
    Eigen::MatrixXd V(2, 3);
    V << 0.0, 0.0, 0.0,
         4.0, 0.0, 0.0;   // longest edge = 4

    auto info = computeNormalization(V);
    auto Vn   = applyNormalization(V, info);

    EXPECT_NEAR(Vn.col(0).minCoeff(), -1.0, 1e-12);
    EXPECT_NEAR(Vn.col(0).maxCoeff(),  1.0, 1e-12);
}

TEST(MathUtils, NormalizationIsUniform) {
    // Non-square AABB: uniform scale must keep all axes in [-1,1]
    Eigen::MatrixXd V(2, 3);
    V << 0.0, 0.0, 0.0,
         8.0, 2.0, 1.0;   // longest edge = 8

    auto info = computeNormalization(V);
    auto Vn   = applyNormalization(V, info);

    // All values must be within [-1, 1]
    EXPECT_LE(Vn.maxCoeff(),  1.0 + 1e-12);
    EXPECT_GE(Vn.minCoeff(), -1.0 - 1e-12);
}

TEST(MathUtils, NormalizationRoundTrip) {
    Eigen::MatrixXd V(3, 3);
    V << 1.0, 2.0, 3.0,
         4.0, 5.0, 6.0,
         7.0, 8.0, 9.0;

    auto info  = computeNormalization(V);
    auto Vn    = applyNormalization(V, info);
    auto Vback = applyDenormalization(Vn, info);

    EXPECT_NEAR((V - Vback).norm(), 0.0, 1e-10);
}

TEST(MathUtils, ScaledDistanceIsConsistent) {
    Eigen::MatrixXd V(2, 3);
    V << 0.0, 0.0, 0.0,
         4.0, 0.0, 0.0;

    double d    = 1.0;
    auto info   = computeNormalization(V);  // scale = 0.5 (longest = 4 -> 2)
    double dSc  = d * info.scale;

    // In world space: max along +X is 4.0, support = 4 + 1 = 5
    // In norm space:  max along +X is 1.0, support_n = 1 + d*scale
    Eigen::Vector3d nx(1, 0, 0);
    auto Vn = applyNormalization(V, info);
    double supportNorm  = (Vn * nx).maxCoeff() + dSc;
    double supportWorld = (V  * nx).maxCoeff() + d;

    // Both should describe the same physical plane
    EXPECT_NEAR(supportNorm * info.scaleInv + info.center.x(), supportWorld, 1e-10);
}

// ---------------------------------------------------------------------------
// Direction generation & merging
// ---------------------------------------------------------------------------
TEST(MathUtils, Generate26DirectionsCount) {
    auto dirs = generate26Directions();
    EXPECT_EQ(static_cast<int>(dirs.size()), 26);
}

TEST(MathUtils, AllDirectionsAreNormalized) {
    auto dirs = generate26Directions();
    for (const auto& d : dirs)
        EXPECT_NEAR(d.norm(), 1.0, 1e-12);
}

TEST(MathUtils, MergeDirectionsRemovesNearDuplicates) {
    // Manually construct two nearly-identical directions
    Eigen::Vector3d d1(1.0, 0.0, 0.0);
    Eigen::Vector3d d2 = Eigen::Vector3d(1.0, 0.01, 0.0).normalized();  // ~0.57 deg away
    Eigen::Vector3d d3(0.0, 1.0, 0.0);

    auto merged = mergeDirections({d1, d2, d3});
    // d1 and d2 are < 5 deg apart, so one should be removed
    EXPECT_EQ(static_cast<int>(merged.size()), 2);
}

TEST(MathUtils, MergeDirectionsKeepsAntiParallel) {
    // Anti-parallel directions constrain opposite sides — must NOT be merged
    Eigen::Vector3d d1(1.0, 0.0, 0.0);
    Eigen::Vector3d d2(-1.0, 0.0, 0.0);

    auto merged = mergeDirections({d1, d2});
    EXPECT_EQ(static_cast<int>(merged.size()), 2);
}

// ---------------------------------------------------------------------------
// isInsideAll
// ---------------------------------------------------------------------------
TEST(MathUtils, IsInsideAllUnit) {
    // Two half-spaces: x <= 1 and -x <= 1  =>  -1 <= x <= 1
    std::vector<HalfSpace> hs = {
        {Eigen::Vector3d( 1, 0, 0),  1.0},
        {Eigen::Vector3d(-1, 0, 0),  1.0},
    };

    EXPECT_TRUE(isInsideAll(Eigen::Vector3d(0.0, 0, 0), hs));
    EXPECT_TRUE(isInsideAll(Eigen::Vector3d(1.0, 0, 0), hs));   // on boundary
    EXPECT_FALSE(isInsideAll(Eigen::Vector3d(1.1, 0, 0), hs));  // outside
    EXPECT_FALSE(isInsideAll(Eigen::Vector3d(-1.1, 0, 0), hs));
}
