#pragma once
#include <string>
#include <vector>
#include <cstdint>

#include "core/vector.h"

namespace COR {
    struct MeshData {
        struct UVec3 { uint32_t x, y, z; };

        std::vector<Vec3> positions;     // unified vertex buffer
        std::vector<Vec3> normals;       // same size as positions (¾øÀ¸¸é 0)
        std::vector<UVec3> indices;      // triangles
        std::vector<int> triMaterialId;  // triangle count == indices.size()
    };

    struct ObjLoadOptions {
        bool triangulate = true; // faces -> triangles
        float scale = 1.0f;
        Vec3 translate = Vec3(0.0f);
    };

    bool LoadObjMesh(const std::string& objPath, MeshData& out, ObjLoadOptions opt = {});
} // namespace COR
