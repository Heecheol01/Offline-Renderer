#pragma once
#include <vector>
#include <algorithm>
#include <cmath>

#include "scene/shape.h"
#include "io/obj_loader.h"

namespace COR {

    class TriangleMesh final : public Shape {
    public:
        explicit TriangleMesh(MeshData&& md) : data(std::move(md)) {}

        bool intersect(const Ray& r, float tMin, float tMax, HitRecord& rec) const override;
        bool bounds(AABB& out) const override;
        float area() const override { return totalArea; }
        bool sampleSurface(RNG& rng, Vec3& pos, Vec3& normal, float& pdf_area) const override;

    private:
        void buildSampling() const;
        void ensureSamplingBuild() const {
            if (!samplingBuilt) buildSampling();
        }

        MeshData data;

        mutable bool samplingBuilt = false;
        mutable float totalArea = 0.0f;
        mutable std::vector<float> cdf;
    };

} // namespace COR
