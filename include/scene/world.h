// include/scene/world.h

#pragma once

#include <vector>
#include "scene/sphere.h"
#include "scene/hit.h"
#include "core/ray.h"

namespace COR {
	struct World {
		std::vector<Sphere> spheres;

		bool intersect(const Ray& r, float tMin, float tMax, HitRecord& rec) const {
			HitRecord tmp;
			bool hitAnything = false;
			float closest = tMax;

			for (const auto& s : spheres) {
				if (s.intersect(r, tMin, closest, tmp)) {
					hitAnything = true;
					closest = tmp.t;
					rec = tmp;
				}
			}

			return hitAnything;
		}
	};
}