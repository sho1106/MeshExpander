#pragma once

#include <Eigen/Dense>
#include <vector>

namespace expander {

struct Mesh {
    Eigen::MatrixXd          vertices;  // Nx3 row-major: each row is one vertex
    std::vector<Eigen::Vector3i> faces; // M triangles, each holds 3 vertex indices

    int numVertices() const { return static_cast<int>(vertices.rows()); }
    int numFaces()    const { return static_cast<int>(faces.size()); }
    bool empty()      const { return vertices.rows() == 0; }
};

} // namespace expander
