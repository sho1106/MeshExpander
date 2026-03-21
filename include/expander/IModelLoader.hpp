#pragma once
// ---------------------------------------------------------------------------
// IModelLoader — abstract interface for multi-part model file loading
//
// Implementations (e.g. AssimpLoader) are in the optional mesh_expander_io
// library and can be swapped without touching the core or AssemblyExpander.
// ---------------------------------------------------------------------------

#include "expander/Mesh.hpp"
#include <string>
#include <vector>

namespace expander {

class IModelLoader {
public:
    virtual ~IModelLoader() = default;

    // Load all mesh parts from a file.
    // Each mesh in the returned vector corresponds to one part (scene mesh node).
    // Vertex coordinates are in world space (node transforms applied).
    // Throws std::runtime_error on failure.
    virtual std::vector<Mesh> load(const std::string& path) = 0;
};

} // namespace expander
