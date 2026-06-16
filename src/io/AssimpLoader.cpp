#include "io/AssimpLoader.hpp"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <stdexcept>

namespace expander {
namespace io {

// ---------------------------------------------------------------------------
// Default post-processing flags
//   aiProcess_Triangulate         — convert polygons to triangles
//   aiProcess_JoinIdenticalVertices — deduplicate vertices
//   aiProcess_SortByPType         — separate non-triangle primitives
// ---------------------------------------------------------------------------
static constexpr unsigned int kDefaultFlags =
    aiProcess_Triangulate |
    aiProcess_JoinIdenticalVertices |
    aiProcess_SortByPType;

AssimpLoader::AssimpLoader(unsigned int flags)
    : flags_(flags == 0 ? kDefaultFlags : flags)
{}

// ---------------------------------------------------------------------------
// load()
// ---------------------------------------------------------------------------
std::vector<Mesh> AssimpLoader::load(const std::string& path)
{
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path, flags_);

    if (!scene || (scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) || !scene->mRootNode)
        throw std::runtime_error("AssimpLoader: " +
                                 std::string(importer.GetErrorString()));

    std::vector<Mesh> result;
    result.reserve(scene->mNumMeshes);

    aiMatrix4x4 identity;  // default-constructed = identity matrix
    collectMeshes(scene, scene->mRootNode, identity, result);
    return result;
}

// ---------------------------------------------------------------------------
// collectMeshes() — depth-first traversal, accumulates node transforms
// ---------------------------------------------------------------------------
void AssimpLoader::collectMeshes(const aiScene*     scene,
                                  const aiNode*      node,
                                  const aiMatrix4x4& parentTransform,
                                  std::vector<Mesh>& out) const
{
    // Accumulated world transform for this node
    aiMatrix4x4 worldTransform = parentTransform * node->mTransformation;

    for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
        const aiMesh* aim = scene->mMeshes[node->mMeshes[i]];
        // Skip non-triangle meshes (e.g. lines, points left after SortByPType)
        if (!(aim->mPrimitiveTypes & aiPrimitiveType_TRIANGLE)) continue;
        out.push_back(toMesh(aim, worldTransform));
    }

    for (unsigned int i = 0; i < node->mNumChildren; ++i)
        collectMeshes(scene, node->mChildren[i], worldTransform, out);
}

// ---------------------------------------------------------------------------
// toMesh() — convert aiMesh + accumulated transform to our Mesh format
// ---------------------------------------------------------------------------
Mesh AssimpLoader::toMesh(const aiMesh* aim, const aiMatrix4x4& transform)
{
    Mesh m;
    m.vertices.resize(static_cast<int>(aim->mNumVertices), 3);

    for (unsigned int i = 0; i < aim->mNumVertices; ++i) {
        // Apply world transform to each vertex
        aiVector3D v = transform * aim->mVertices[i];
        m.vertices(static_cast<int>(i), 0) = static_cast<double>(v.x);
        m.vertices(static_cast<int>(i), 1) = static_cast<double>(v.y);
        m.vertices(static_cast<int>(i), 2) = static_cast<double>(v.z);
    }

    m.faces.reserve(aim->mNumFaces);
    for (unsigned int i = 0; i < aim->mNumFaces; ++i) {
        const aiFace& f = aim->mFaces[i];
        if (f.mNumIndices != 3) continue;  // guard: should all be triangles
        m.faces.push_back({static_cast<int>(f.mIndices[0]),
                           static_cast<int>(f.mIndices[1]),
                           static_cast<int>(f.mIndices[2])});
    }
    return m;
}

} // namespace io
} // namespace expander
