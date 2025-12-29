// include/scene/hit.h

#pragma once

#include "core/vector.h"
#include "core/ray.h"

namespace COR {
	struct HitRecord {
		Vec3 p;
		Vec3 n;
		Vec3 albedo{ 1.0f, 1.0f, 1.0f };
		float t = 0.0f;
		bool frontFace = true;

		void setFaceNormal(const Ray& r, const Vec3& outwardNormal) {
			frontFace = dot(r.d, outwardNormal) < 0.0f;
			n = frontFace ? outwardNormal : outwardNormal * -1.0f;
		}
	};
}