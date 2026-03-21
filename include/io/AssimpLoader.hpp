#pragma once
// ---------------------------------------------------------------------------
// AssimpLoader — IModelLoader implementation backed by Assimp
//
// Supports all formats Assimp can read (OBJ, FBX, STEP, GLTF, STL, …).
// Each aiMesh in the scene is returned as one Mesh.
// Node transforms are accumulated so vertices are in world space.
//
// Available only when MESHEXPANDER_BUILD_IO is ON (links mesh_expander_io).
// ---------------------------------------------------------------------------

#include "expander/IModelLoader.hpp"

struct aiScene;
struct aiNode;
struct aiMesh;
struct aiMatrix4x4;

namespace expander {
namespace io {

class AssimpLoader : public IModelLoader {
public:
    // flags: Assimp post-processing flags (aiProcess_*).
    // Defaults apply triangulation and vertex deduplication.
    explicit AssimpLoader(unsigned int flags = 0);

    // Load all mesh parts from file. Returns one Mesh per aiMesh node.
    // Throws std::runtime_error if the file cannot be read.
    std::vector<Mesh> load(const std::string& path) override;

private:
    unsigned int flags_;

    void collectMeshes(const aiScene*     scene,
                       const aiNode*      node,
                       const aiMatrix4x4& parentTransform,
                       std::vector<Mesh>& out) const;

    static Mesh toMesh(const aiMesh* aim, const aiMatrix4x4& transform);
};

} // namespace io
} // namespace expander
