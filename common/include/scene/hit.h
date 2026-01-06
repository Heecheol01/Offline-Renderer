// include/scene/hit.h

#pragma once

#include "core/vector.h"
#include "core/ray.h"

namespace COR {
	struct Primitive;

	struct HitRecord {
		Vec3 p;				// hit position
		Vec3 n;				// shading normal
		float t = 0.0f;		// ray parameter
		bool frontFace = true;
		int materialId = -1;
		const Primitive* prim = nullptr;

		void setFaceNormal(const Ray& r, const Vec3& outwardNormal) {
			frontFace = dot(r.d, outwardNormal) < 0.0f;
			n = frontFace ? outwardNormal : outwardNormal * -1.0f;
		}
	};
}