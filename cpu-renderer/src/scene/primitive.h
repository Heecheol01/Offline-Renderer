// include/scene/primitive.h

#pragma once

#include <memory>
#include "scene/shape.h"
#include "scene/hit.h"
#include "accel/aabb.h"

namespace COR {
	struct Primitive {
		virtual ~Primitive() = default;
		virtual bool intersect(const Ray& r, float tMin, float tMax, HitRecord& rec) const = 0;
		virtual bool bounds(AABB& out) const = 0;

		virtual const Shape* shape() const = 0;
		virtual int materialId() const = 0;
	};

	class GeometricPrimitive final : public Primitive {
	public:
		GeometricPrimitive(std::shared_ptr<Shape> s, int mid) : shape_(std::move(s)), mid_(mid) {};

		bool intersect(const Ray& r, float tMin, float tMax, HitRecord& rec) const {
			HitRecord tmp;
			if (!shape_->intersect(r, tMin, tMax, tmp)) return false;
			tmp.materialId = mid_;
			tmp.prim = this;
			rec = tmp;
			return true;
		}

		bool bounds(AABB& out) const override { return shape_->bounds(out); }
		const Shape* shape() const override { return shape_.get(); }
		int materialId() const override { return mid_; }

	private:
		std::shared_ptr<Shape> shape_;
		int mid_ = -1;
	};
}