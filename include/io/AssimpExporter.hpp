#pragma once
// ---------------------------------------------------------------------------
// AssimpExporter — IModelExporter implementation backed by Assimp
//
// Output format is determined by the file extension at runtime.
// Supported formats depend on the Assimp build (query via listFormats()).
// Common formats: .obj .stl .fbx .gltf .glb .ply .dae
//
// Each Mesh in the input vector is written as a separate mesh node.
//
// Available only when MESHEXPANDER_BUILD_IO is ON (links mesh_expander_io).
// ---------------------------------------------------------------------------

#include "expander/IModelExporter.hpp"
#include <string>
#include <vector>

namespace expander {
namespace io {

class AssimpExporter : public IModelExporter {
public:
    // Write meshes to file. Format inferred from path extension.
    // Throws std::runtime_error on failure or unknown extension.
    void write(const std::string& path,
               const std::vector<Mesh>& meshes) override;

    // Return a list of supported export format extensions (e.g. {"obj","stl",...}).
    static std::vector<std::string> listFormats();

private:
    // Map file extension (lower-case, no dot) to Assimp format ID string.
    // Queries Assimp at runtime — no hard-coded map needed.
    static std::string extensionToFormatId(const std::string& path);
};

} // namespace io
} // namespace expander
