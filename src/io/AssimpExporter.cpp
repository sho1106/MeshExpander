#include "io/AssimpExporter.hpp"

#include <assimp/Exporter.hpp>
#include <assimp/scene.h>

#include <algorithm>
#include <cctype>
#include <memory>
#include <stdexcept>

namespace expander {
namespace io {

// ---------------------------------------------------------------------------
// extensionToFormatId()
// Query Assimp's exporter list at runtime — no hard-coded format map.
// ---------------------------------------------------------------------------
std::string AssimpExporter::extensionToFormatId(const std::string& path)
{
    auto pos = path.rfind('.');
    if (pos == std::string::npos)
        throw std::runtime_error(
            "AssimpExporter: no file extension in '" + path + "'");

    std::string ext = path.substr(pos + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    Assimp::Exporter exporter;
    for (size_t i = 0; i < exporter.GetExportFormatCount(); ++i) {
        const aiExportFormatDesc* desc = exporter.GetExportFormatDescription(i);
        if (desc && ext == desc->fileExtension)
            return desc->id;
    }
    throw std::runtime_error(
        "AssimpExporter: no exporter found for '." + ext + "'");
}

// ---------------------------------------------------------------------------
// listFormats()
// ---------------------------------------------------------------------------
std::vector<std::string> AssimpExporter::listFormats()
{
    Assimp::Exporter exporter;
    std::vector<std::string> result;
    result.reserve(exporter.GetExportFormatCount());
    for (size_t i = 0; i < exporter.GetExportFormatCount(); ++i) {
        const aiExportFormatDesc* desc = exporter.GetExportFormatDescription(i);
        if (desc) result.emplace_back(desc->fileExtension);
    }
    return result;
}

// ---------------------------------------------------------------------------
// buildAiMesh() — convert our Mesh to aiMesh (caller owns the pointer)
// ---------------------------------------------------------------------------
static aiMesh* buildAiMesh(const Mesh& m)
{
    auto* aim           = new aiMesh();
    aim->mPrimitiveTypes = aiPrimitiveType_TRIANGLE;
    aim->mMaterialIndex  = 0;

    // Vertices
    aim->mNumVertices = static_cast<unsigned int>(m.numVertices());
    aim->mVertices    = new aiVector3D[aim->mNumVertices];
    for (unsigned int i = 0; i < aim->mNumVertices; ++i) {
        aim->mVertices[i] = aiVector3D(
            static_cast<ai_real>(m.vertices(static_cast<int>(i), 0)),
            static_cast<ai_real>(m.vertices(static_cast<int>(i), 1)),
            static_cast<ai_real>(m.vertices(static_cast<int>(i), 2)));
    }

    // Faces
    aim->mNumFaces = static_cast<unsigned int>(m.numFaces());
    aim->mFaces    = new aiFace[aim->mNumFaces];
    for (unsigned int i = 0; i < aim->mNumFaces; ++i) {
        aim->mFaces[i].mNumIndices = 3;
        aim->mFaces[i].mIndices    = new unsigned int[3];
        aim->mFaces[i].mIndices[0] = static_cast<unsigned int>(m.faces[static_cast<int>(i)][0]);
        aim->mFaces[i].mIndices[1] = static_cast<unsigned int>(m.faces[static_cast<int>(i)][1]);
        aim->mFaces[i].mIndices[2] = static_cast<unsigned int>(m.faces[static_cast<int>(i)][2]);
    }
    return aim;
}

// ---------------------------------------------------------------------------
// buildScene() — assemble aiScene from mesh list (caller owns the pointer)
// aiScene destructor recursively frees all child objects.
// ---------------------------------------------------------------------------
static aiScene* buildScene(const std::vector<Mesh>& meshes)
{
    auto* scene = new aiScene();
    scene->mFlags = 0;

    // One default material (required even if unused)
    scene->mNumMaterials = 1;
    scene->mMaterials    = new aiMaterial*[1];
    scene->mMaterials[0] = new aiMaterial();

    // Meshes
    const unsigned int numMeshes = static_cast<unsigned int>(meshes.size());
    scene->mNumMeshes = numMeshes;
    scene->mMeshes    = new aiMesh*[numMeshes];
    for (unsigned int i = 0; i < numMeshes; ++i)
        scene->mMeshes[i] = buildAiMesh(meshes[i]);

    // Root node referencing all meshes
    scene->mRootNode              = new aiNode("root");
    scene->mRootNode->mNumMeshes  = numMeshes;
    scene->mRootNode->mMeshes     = new unsigned int[numMeshes];
    for (unsigned int i = 0; i < numMeshes; ++i)
        scene->mRootNode->mMeshes[i] = i;

    return scene;
}

// ---------------------------------------------------------------------------
// write()
// ---------------------------------------------------------------------------
void AssimpExporter::write(const std::string& path,
                           const std::vector<Mesh>& meshes)
{
    if (meshes.empty()) return;

    const std::string fmtId = extensionToFormatId(path);

    // unique_ptr ensures cleanup even if Export throws
    std::unique_ptr<aiScene> scene(buildScene(meshes));

    Assimp::Exporter exporter;
    const aiReturn ret = exporter.Export(scene.get(), fmtId, path);
    if (ret != AI_SUCCESS)
        throw std::runtime_error(
            "AssimpExporter: " + std::string(exporter.GetErrorString()));
}

} // namespace io
} // namespace expander
