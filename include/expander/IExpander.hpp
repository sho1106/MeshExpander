#pragma once

#include "Mesh.hpp"

namespace expander {

class IExpander {
public:
    virtual ~IExpander() = default;

    // Expand input mesh by expansionDist in world units.
    // The result is a conservative (all original vertices lie inside or on
    // the boundary) and lightweight (vertex count bounded by direction count)
    // convex polyhedron.
    virtual Mesh expand(const Mesh& input, double expansionDist) = 0;
};

} // namespace expander
