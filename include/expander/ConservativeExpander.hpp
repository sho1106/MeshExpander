#pragma once

#include "IExpander.hpp"
#include "MathUtils.hpp"

namespace expander {

// ---------------------------------------------------------------------------
// ConservativeExpander
//
// Algorithm (Half-space Intersection / "削り出し"):
//
//  1. Uniform normalize all vertices into [-1,1]^3.
//  2. Build half-spaces from support directions:
//       - If input mesh has face data: use face normals (merged to ~20 deg),
//         giving a tight shape-adaptive polytope.
//       - Otherwise: use the constructor-supplied fixed directions
//         (default: 26-neighbourhood of the unit grid).
//     Each half-space: H_i = { x | n_i . x <= max(V.n_i) + d_scaled }
//  3. Enumerate all C(k,3) triples of planes; solve for intersection points
//     via ColPivHouseholderQR.  Skip near-singular (rank < 3) systems.
//  4. Retain only intersection points that satisfy ALL half-spaces.
//  5. Per face (half-space), collect its boundary vertices, sort by angle,
//     fan-triangulate with outward-consistent winding.
//  6. Denormalize back to world space.
//
// Accuracy (face-normal mode):
//   For convex meshes, face normals are the exact Minkowski-sum directions.
//   No apex-dominance artefact (unlike 26 fixed diagonal directions).
//   Sphere ≈ +3%, cylinder ≈ +1%, cone ≈ +3%.
// ---------------------------------------------------------------------------
class ConservativeExpander : public IExpander {
public:
    // dirs: support directions used when the input mesh has no face data.
    //   Default = 26-neighbourhood of unit grid (backward compatible).
    // faceNormalMergeDeg: merge threshold when extracting face normals from mesh.
    explicit ConservativeExpander(
        std::vector<Eigen::Vector3d> dirs            = math::generate26Directions(),
        double                       faceNormalMergeDeg = 20.0);

    Mesh expand(const Mesh& input, double expansionDist) override;

private:
    std::vector<Eigen::Vector3d> dirs_;
    double faceNormalMergeDeg_;

    // Find all vertices of the polytope defined by hs.
    std::vector<Eigen::Vector3d> intersectHalfSpaces(
        const std::vector<math::HalfSpace>& hs) const;

    // Build a triangulated Mesh from polytope vertices + defining half-spaces.
    Mesh buildMesh(const std::vector<Eigen::Vector3d>& verts,
                   const std::vector<math::HalfSpace>& hs) const;
};

} // namespace expander
