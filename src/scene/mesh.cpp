#include "scene/mesh.h"

namespace COR {

    static inline bool intersectTriMT(
        const Ray& r, const Vec3& v0, const Vec3& v1, const Vec3& v2,
        float tMin, float tMax, float& t, float& u, float& v)
    {
        const Vec3 e1 = v1 - v0;
        const Vec3 e2 = v2 - v0;
        const Vec3 p = cross(r.d, e2);
        const float det = dot(e1, p);
        if (std::fabs(det) < 1e-8f) return false;
        const float invDet = 1.0f / det;

        const Vec3 s = r.o - v0;
        u = dot(s, p) * invDet;
        if (u < 0.0f || u > 1.0f) return false;

        const Vec3 q = cross(s, e1);
        v = dot(r.d, q) * invDet;
        if (v < 0.0f || (u + v) > 1.0f) return false;

        t = dot(e2, q) * invDet;
        if (t < tMin || t > tMax) return false;
        return true;
    }

    static inline float triArea(const Vec3& a, const Vec3& b, const Vec3& c) {
        return 0.5f * length(cross(b - a, c - a));
    }

    void TriangleMesh::buildSampling() const {
        totalArea = 0.0f;
        cdf.clear();
        cdf.reserve(data.indices.size());

        for (size_t i = 0; i < data.indices.size(); ++i) {
            auto tri = data.indices[i];
            float A = triArea(data.positions[tri.x], data.positions[tri.y], data.positions[tri.z]);
            totalArea += A;
            cdf.push_back(totalArea);
        }

        if (totalArea > 0.0f) {
            for (auto& x : cdf) x /= totalArea;
        }

        samplingBuilt = true;
    }

    bool TriangleMesh::intersect(const Ray& r, float tMin, float tMax, HitRecord& rec) const {
        bool hit = false;
        float closest = tMax;

        for (size_t i = 0; i < data.indices.size(); ++i) {
            auto tri = data.indices[i];
            const Vec3& v0 = data.positions[tri.x];
            const Vec3& v1 = data.positions[tri.y];
            const Vec3& v2 = data.positions[tri.z];

            float t, u, v;
            if (!intersectTriMT(r, v0, v1, v2, tMin, closest, t, u, v)) continue;

            Vec3 gn = normalize(cross(v1 - v0, v2 - v0));

            Vec3 n0 = data.normals[tri.x];
            Vec3 n1 = data.normals[tri.y];
            Vec3 n2 = data.normals[tri.z];
            Vec3 sn = n0 * (1.0f - u - v) + n1 * u + n2 * v;
            if (dot(sn, sn) < 1e-10f) sn = gn;
            else sn = normalize(sn);

            rec.t = t;
            rec.p = r.at(t);
            rec.setFaceNormal(r, sn);

            closest = t;
            hit = true;
        }

        return hit;
    }

    bool TriangleMesh::bounds(AABB& out) const {
        if (data.positions.empty()) return false;

        out = AABB{};

        for (const Vec3& p : data.positions) {
            out.expand(p);
        }

        const float eps = 1e-6f;
        out.mn = out.mn - Vec3(eps);
        out.mx = out.mx + Vec3(eps);

        return true;
    }

    bool TriangleMesh::sampleSurface(RNG& rng, Vec3& pos, Vec3& normal, float& pdf_area) const {
        ensureSamplingBuild();
        if (totalArea <= 0.0f || cdf.empty()) return false;

        float xi = rng.nextFloat01();
        auto it = std::lower_bound(cdf.begin(), cdf.end(), xi);
        size_t triIdx = (size_t)std::clamp(
            (ptrdiff_t)(it - cdf.begin()),
            (ptrdiff_t)0,
            (ptrdiff_t)cdf.size() - 1);

        auto tri = data.indices[triIdx];
        const Vec3& a = data.positions[tri.x];
        const Vec3& b = data.positions[tri.y];
        const Vec3& c = data.positions[tri.z];

        // uniform barycentric
        float r1 = rng.nextFloat01();
        float r2 = rng.nextFloat01();
        float su = std::sqrt(r1);
        float u = 1.0f - su;
        float v = r2 * su;
        float w = 1.0f - u - v;

        pos = a * u + b * v + c * w;
        normal = normalize(cross(b - a, c - a));

        pdf_area = 1.0f / totalArea;
        return true;
    }

} // namespace COR
