#pragma once
// ---------------------------------------------------------------------------
// IModelExporter — abstract interface for multi-part model file export
//
// Implementations (e.g. AssimpExporter) are in the optional mesh_expander_io
// library. Output format is determined by the file extension at runtime.
// ---------------------------------------------------------------------------

#include "expander/Mesh.hpp"
#include <string>
#include <vector>

namespace expander {

class IModelExporter {
public:
    virtual ~IModelExporter() = default;

    // Write meshes to file.
    // Format is determined by the file extension (e.g. ".obj", ".stl", ".fbx").
    // Each Mesh in the vector is written as a separate part/node.
    // Throws std::runtime_error on failure or unsupported format.
    virtual void write(const std::string& path,
                       const std::vector<Mesh>& meshes) = 0;
};

} // namespace expander
