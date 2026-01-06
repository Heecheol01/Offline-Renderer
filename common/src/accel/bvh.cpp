#include "accel/bvh.h"
#include <algorithm>
#include <cmath>

namespace COR {
	static inline Vec3 centroidOf(const AABB& b) {
		return (b.mn + b.mx) * 0.5f;
	}

	BVHAccel::BVHAccel(std::vector<Primitive*> prims, uint32_t leafSize) : prims_(std::move(prims)), leafSize_(leafSize) {
		order_.resize(prims_.size());
		for (uint32_t i = 0; i < (uint32_t)prims_.size(); ++i)
			order_[i] = i;

		nodes_.reserve(prims_.size() * 2);
		root_ = prims_.empty() ? -1 : buildNode(0, (uint32_t)prims_.size());
	}

	bool BVHAccel::bounds(AABB& out) const {
		if (root_ < 0) return false;
		out = nodes_[root_].box;
		return true;
	}

	int BVHAccel::buildNode(uint32_t start, uint32_t end) {
		Node node;
		AABB box;
		AABB cbox;

		uint32_t valid = 0;
		for (uint32_t i = start; i < end; ++i) {
			uint32_t id = order_[i];
			AABB b;
			if (!prims_[id]->bounds(b)) continue;
			box = AABB::merge(box, b);
			cbox.expand(centroidOf(b));
			++valid;
		}

		int my = (int)nodes_.size();
		nodes_.push_back(node);
		nodes_[my].box = box;

		uint32_t count = end - start;
		if (count <= leafSize_ || valid == 0) {
			nodes_[my].leaf = true;
			nodes_[my].start = start;
			nodes_[my].count = count;
			return my;
		}

		Vec3 ext = cbox.mx - cbox.mn;
		int axis = 0;
		if (ext.y > ext.x) axis = 1;
		if (ext.z > (axis == 0 ? ext.x : ext.y)) axis = 2;

		if (std::fabs(ext.x) < 1e-12f && std::fabs(ext.y) < 1e-12f && std::fabs(ext.z) < 1e-12f) {
			nodes_[my].leaf = true;
			nodes_[my].start = start;
			nodes_[my].count = count;
			return my;
		}

		auto key = [&](uint32_t primIndex) {
			AABB b;
			prims_[primIndex]->bounds(b);
			Vec3 c = centroidOf(b);
			return axis == 0 ? c.x : (axis == 1 ? c.y : c.z);
		};

		uint32_t mid = start + count / 2;
		std::nth_element(order_.begin() + start, order_.begin() + mid, order_.begin() + end,
			[&](uint32_t a, uint32_t b) { return key(a) < key(b); });

		int L = buildNode(start, mid);
		int R = buildNode(mid, end);

		nodes_[my].leaf = false;
		nodes_[my].left = L;
		nodes_[my].right = R;
		nodes_[my].box = AABB::merge(nodes_[L].box, nodes_[R].box);
		return my;
	}

	bool BVHAccel::intersect(const Ray& r, float tMin, float tMax, HitRecord& rec) const {
		if (root_ < 0) return false;

		bool hit = false;
		float closest = tMax;

		int stack[64];
		int sp = 0;
		stack[sp++] = root_;

		while (sp) {
			int ni = stack[--sp];
			const Node& n = nodes_[ni];

			if (!n.box.hit(r, tMin, closest)) continue;

			if (n.leaf) {
				for (uint32_t i = 0; i < n.count; ++i) {
					uint32_t primId = order_[n.start + i];
					HitRecord tmp;
					if (prims_[primId]->intersect(r, tMin, closest, tmp)) {
						closest = tmp.t;
						rec = tmp;
						hit = true;
					}
				}
			}
			else {
				if (n.left >= 0) stack[sp++] = n.left;
				if (n.right >= 0) stack[sp++] = n.right;
			}
		}

		return hit;
	}
}