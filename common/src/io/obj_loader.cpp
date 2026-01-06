// src/io/obj_loader.cpp
#include "io/obj_loader.h"

#include "util/tiny_obj_loader.h"
#include <filesystem>
#include <unordered_map>
#include <iostream>

namespace COR {

    struct Key {
        int v, n;
        bool operator==(const Key& o) const { return v == o.v && n == o.n; }
    };
    struct KeyHash {
        size_t operator()(const Key& k) const noexcept {
            size_t h = 1469598103934665603ull;
            auto mix = [&](int x) { h ^= (uint32_t)x + 0x9e3779b9 + (h << 6) + (h >> 2); };
            mix(k.v); mix(k.n);
            return h;
        }
    };

    static inline Vec3 readPos(const tinyobj::attrib_t& a, int vi) {
        return Vec3(
            (float)a.vertices[3 * vi + 0],
            (float)a.vertices[3 * vi + 1],
            (float)a.vertices[3 * vi + 2]
        );
    }
    static inline Vec3 readNrm(const tinyobj::attrib_t& a, int ni) {
        return Vec3(
            (float)a.normals[3 * ni + 0],
            (float)a.normals[3 * ni + 1],
            (float)a.normals[3 * ni + 2]
        );
    }

    bool LoadObjMesh(const std::string& objPath, MeshData& out, ObjLoadOptions opt) {
        out = MeshData{};

        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
        std::string warn, err;

        std::filesystem::path p(objPath);
        std::string baseDir = p.parent_path().string();
        if (!baseDir.empty()) baseDir += std::filesystem::path::preferred_separator;

        bool ret = tinyobj::LoadObj(
            &attrib, &shapes, &materials, &warn, &err,
            objPath.c_str(), baseDir.c_str(),
            opt.triangulate);

        if (!warn.empty()) std::cerr << "[tinyobj warn] " << warn << "\n";
        if (!err.empty())  std::cerr << "[tinyobj err ] " << err << "\n";
        if (!ret) return false;

        std::unordered_map<Key, uint32_t, KeyHash> unify;
        unify.reserve(1024);

        auto getVertex = [&](const tinyobj::index_t& idx) -> uint32_t {
            Key key{ idx.vertex_index, idx.normal_index };
            auto it = unify.find(key);
            if (it != unify.end()) return it->second;

            uint32_t newId = (uint32_t)out.positions.size();

            Vec3 pos = readPos(attrib, idx.vertex_index);
            pos = pos * opt.scale + opt.translate;
            out.positions.push_back(pos);

            if (idx.normal_index >= 0 && !attrib.normals.empty()) {
                out.normals.push_back(readNrm(attrib, idx.normal_index));
            }
            else {
                out.normals.push_back(Vec3(0.0f)); // 없으면 0으로 둠(교차 시 기하법선 사용)
            }

            unify.emplace(key, newId);
            return newId;
            };

        for (const auto& sh : shapes) {
            size_t index_offset = 0;

            for (size_t f = 0; f < sh.mesh.num_face_vertices.size(); ++f) {
                int fv = (int)sh.mesh.num_face_vertices[f];
                if (fv != 3) { index_offset += fv; continue; } // triangulate=true면 보통 3

                auto i0 = sh.mesh.indices[index_offset + 0];
                auto i1 = sh.mesh.indices[index_offset + 1];
                auto i2 = sh.mesh.indices[index_offset + 2];

                uint32_t v0 = getVertex(i0);
                uint32_t v1 = getVertex(i1);
                uint32_t v2 = getVertex(i2);

                out.indices.push_back(MeshData::UVec3{ v0, v1, v2 });

                // main에서 통째로 덮어쓸 예정이라 기본 -1로 둠
                int mid = -1;
                out.triMaterialId.push_back(mid);

                index_offset += fv;
            }
        }

        return true;
    }

} // namespace COR
