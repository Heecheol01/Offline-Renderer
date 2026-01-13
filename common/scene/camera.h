// include/scene/camera.h
#pragma once

#include "core/vector.h"
#include "core/ray.h"
#include "core/constants.h"
#include <cmath>

namespace COR {
	struct Camera {
		Vec3 origin;
		Vec3 lower_left;
		Vec3 horizontal;
		Vec3 vertical;

		Camera(const Vec3& lookfrom, const Vec3& lookat, const Vec3& vup, float vfov_degrees, float aspect) {
			const float theta = vfov_degrees * (PI / 180.0f);
			const float h = std::tan(theta * 0.5f);

			const float viewport_height = 2.0f * h;
			const float viewport_width = aspect * viewport_height;

			// Camera basis
			Vec3 w = normalize(lookfrom - lookat);
			Vec3 u = normalize(cross(vup, w));
			Vec3 v = cross(w, u);

			origin = lookfrom;
			horizontal = u * viewport_width;
			vertical = v * viewport_height;
			lower_left = origin - horizontal * 0.5f - vertical * 0.5f - w;	// focus distance = 1
		}

		Ray getRay(float s, float t) const {
			Vec3 dir = lower_left + horizontal * s + vertical * t - origin;
			return Ray{ origin, dir };
		}
	};
}