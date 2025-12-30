// include/core/sampling.h

#pragma once

#include "core/vector.h"
#include "core/rng.h"
#include "core/onb.h"
#include "scene/geometry.h"
#include <cmath>

namespace COR {
	// cosine-weighted hemisphere sampling around normal n
	inline Vec3 cosineSampleHemisphere(const Vec3& n, RNG& rng) {
		float r1 = rng.nextFloat01();
		float r2 = rng.nextFloat01();

		float phi = 2.0f * PI * r1;
		float x = std::cos(phi) * std::sqrt(r2);
		float y = std::sin(phi) * std::sqrt(r2);
		float z = std::sqrt(1.0f - r2);

		ONB onb;
		onb.buildFromW(n);
		return normalize(onb.local(x, y, z));
	}

	// Lambert cosine pdf w.r.t. solid angle
	inline float lambertPdf(const Vec3& n, const Vec3& wi) {
		float cosTheta = dot(n, normalize(wi));
		if (cosTheta <= 0.0f) return 0.0f;
		return cosTheta / PI;
	}

	// power heuristic for MIS
	inline float powerHeuristic(float pdfA, float pdfB) {
		float a2 = pdfA * pdfA;
		float b2 = pdfB * pdfB;
		float denom = a2 + b2;
		if (denom <= 0.0f) return 0.0f;
		return a2 / denom;
	}
}