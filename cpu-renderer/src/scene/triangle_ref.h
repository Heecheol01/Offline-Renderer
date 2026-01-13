#pragma once

#include "scene/shape.h"
#include "io/obj_loader.h"
#include "accel/aabb.h"

#include <memory>
#include <cmath>
#include <algorithm>

namespace COR {
	struct MeshStorage {
		MeshData data;
	};

	class TriangleRef final : public Shape {
	public:
		TriangleRef(std::shared_ptr<MeshStorage> m, uint32_t triIndex) : mesh(std::move(m)), tri(triIndex) {}

		bool intersect(const Ray& r, float tMin, float tMax, HitRecord& rec) const override;
		bool bounds(AABB& out) const override;

		float area() const override;
		bool sampleSurface(RNG& rng, Vec3& pos, Vec3& normal, float& pdf_area) const override;

	private:
		std::shared_ptr<MeshStorage> mesh;
		uint32_t tri = 0;
	};
}