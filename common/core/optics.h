#pragma once

#include "core/vector.h"
#include "core/rng.h"
#include <cmath>

namespace COR {
	// ---------------------------------- optics helpers ----------------------------------
	inline Vec3 reflect(const Vec3& v, const Vec3& n) {
		return v - n * (2.0f * dot(v, n));
	}

	// Snell's Law
	inline bool refract(const Vec3& uv, const Vec3& n, float etai_over_etat, Vec3& out) {
		float cosTheta = std::fmin(dot(uv * -1.0f, n), 1.0f);
		Vec3 rOutPerp = (uv + n * cosTheta) * etai_over_etat;
		float k = 1.0f - dot(rOutPerp, rOutPerp);
		if (k < 0.0f) return false;
		Vec3 rOutParallel = n * (-std::sqrt(k));
		out = rOutPerp + rOutParallel;
		return true;
	}

	// Schlick approximation for Fresnel refelctance
	inline float reflectance(float cosine, float ref_idx) {
		float r0 = (1.0f - ref_idx) / (1.0f + ref_idx);
		r0 = r0 * r0;
		return r0 + (1.0f - r0) * std::pow(1.0f - cosine, 5.0f);
	}
}