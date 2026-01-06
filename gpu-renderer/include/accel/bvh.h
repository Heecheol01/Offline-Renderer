// include/accel/bvh.h

#pragma once

#include "scene/primitive.h"
#include "scene/aabb.h"

#include <vector>
#include <cstdint>

namespace COR {
	class BVHAccel final : public Primitive {
	public:
		explicit BVHAccel(std::vector<Primitive*> prims, uint32_t leafSize = 4);

		bool intersect(const Ray& r, float tMin, float tMax, HitRecord& rec) const override;
		bool bounds(AABB& out) const override;

		int materialId() const override { return -1; }
		const Shape* shape() const override { return nullptr; }

	private:
		struct Node {
			AABB box;
			int left = -1;
			int right = -1;
			uint32_t start = 0;
			uint32_t count = 0;
			bool leaf = false;
		};

		std::vector<Primitive*> prims_;
		std::vector<uint32_t> order_;
		std::vector<Node> nodes_;
		int root_ = -1;
		uint32_t leafSize_ = 4;

		int buildNode(uint32_t start, uint32_t end);
	};
}