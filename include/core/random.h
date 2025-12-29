// include/core/random.h

#pragma once

#include "core/vector.h"
#include "core/rng.h"
#include <cmath>

namespace COR {
	inline Vec3 randomInUnitSphere(RNG& rng) {
		while (true) {
			float x = rng.uniform(-1.0f, 1.0f);
			float y = rng.uniform(-1.0f, 1.0f);
			float z = rng.uniform(-1.0f, 1.0f);
			Vec3 p{ x, y, z };
			if (dot(p, p) < 1.0f) return p;
		}
	}

	inline Vec3 randomUnitVector(RNG& rng) {
		return normalize(randomInUnitSphere(rng));
	}

	inline bool nearZero(const Vec3& v) {
		const float s = 1e-8f;
		return (std::fabs(v.x) < s) && (std::fabs(v.y) < s) && (std::fabs(v.z) < s);
	}
}