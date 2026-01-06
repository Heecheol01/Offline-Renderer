// src/film/film.cpp

#include "film/film.h"
#include "core/vector.h"

#include <cmath>      // std::isfinite
#include <cstddef>    // std::size_t
#include <cstdint>    // uint32_t

namespace COR {
	static inline bool isFiniteFloat(float x) {
		return std::isfinite(x);
	}

	static inline bool isFiniteVec3(const Vec3& v) {
		return isFiniteFloat(v.x) && isFiniteFloat(v.y) && isFiniteFloat(v.z);
	}

	static inline std::size_t pixelIndex(int x, int y, int w) {
		return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
	}

	Film::Film(int width, int height)
		: w(width), h(height),
		accum(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), Vec3{ 0.0f, 0.0f, 0.0f }),
		spp(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0u) {}

	void Film::addSample(int x, int y, const Vec3& rgb) {
		if (x < 0 || x >= w || y < 0 || y >= h) return;
		if (!isFiniteVec3(rgb)) return;

		const std::size_t i = pixelIndex(x, y, w);
		accum[i] = accum[i] + rgb;
		spp[i] += 1u;
	}

	Vec3 Film::getPixelAverage(int x, int y) const {
		if (x < 0 || x >= w || y < 0 || y >= h) return Vec3{ 0.0f, 0.0f, 0.0f };

		const std::size_t i = pixelIndex(x, y, w);
		const uint32_t n = spp[i];
		if (n == 0u) return Vec3{ 0.0f, 0.0f, 0.0f };

		const float invN = 1.0f / static_cast<float>(n);
		return accum[i] * invN;
	}
}