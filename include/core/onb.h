// include/core/onb.h

#pragma once

#include "core/vector.h"
#include <cmath>

namespace COR {
	struct ONB {
		Vec3 u, v, w;

		void buildFromW(const Vec3& n) {
			w = normalize(n);
			Vec3 a = (std::fabs(w.x) > 0.9f) ? Vec3{ 0.0f, 1.0f, 0.0f } : Vec3{ 1.0f, 0.0f, 0.0f };
			v = normalize(cross(w, a));
			u = cross(v, w);
		}

		Vec3 local(float a, float b, float c) const {
			return u * a + v * b + w * c;
		}
	};
}