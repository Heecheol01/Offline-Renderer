// include/scene/aabb.h

#pragma once

#include "core/vector.h"
#include "core/ray.h"
#include <algorithm>
#include <cmath>

namespace COR {
	struct AABB {
		Vec3 mn{ 1e30f }, mx{ -1e30f };

		void expand(const Vec3& p) {
			mn.x = std::min(mn.x, p.x); mn.y = std::min(mn.y, p.y); mn.z = std::min(mn.z, p.z);
			mx.x = std::max(mx.x, p.x); mx.y = std::max(mx.y, p.y); mx.z = std::max(mx.z, p.z);
		}

		static AABB merge(const AABB& a, const AABB& b) {
			AABB r;
			r.mn.x = std::min(a.mn.x, b.mn.x);
			r.mn.y = std::min(a.mn.y, b.mn.y);
			r.mn.z = std::min(a.mn.z, b.mn.z);
			r.mx.x = std::max(a.mx.x, b.mx.x);
			r.mx.y = std::max(a.mx.y, b.mx.y);
			r.mx.z = std::max(a.mx.z, b.mx.z);
			return r;
		}

		Vec3 centroid() const { return (mn + mx) * 0.5f; }

		bool hit(const Ray& r, float tMin, float tMax) const {
			auto slab = [&](float ro, float rd, float mnv, float mxv) {
				float inv = (std::fabs(rd) > 1e-12f) ? (1.0f / rd) : 1e30f;
				float t0 = (mnv - ro) * inv;
				float t1 = (mxv - ro) * inv;
				if (inv < 0.0f) std::swap(t0, t1);
				tMin = std::max(tMin, t0);
				tMax = std::min(tMax, t1);
			};

			slab(r.o.x, r.d.x, mn.x, mx.x); if (tMax <= tMin) return false;
			slab(r.o.y, r.d.y, mn.y, mx.y); if (tMax <= tMin) return false;
			slab(r.o.z, r.d.z, mn.z, mx.z); if (tMax <= tMin) return false;
			return true;
		}

		bool intersectRange(const Ray& r, float tMin, float tMax, float& tEnter, float& tExit) const {
			float t0 = tMin;
			float t1 = tMax;

			auto slab = [&](float ro, float rd, float mnv, float mxv) {
				float inv = (std::fabs(rd) > 1e-12f) ? (1.0f / rd) : 1e30f;
				float a = (mnv - ro) * inv;
				float b = (mxv - ro) * inv;
				if (inv < 0.0f) std::swap(a, b);
				t0 = std::max(t0, a);
				t1 = std::min(t1, b);
			};

			slab(r.o.x, r.d.x, mn.x, mx.x); if (t1 <= t0) return false;
			slab(r.o.y, r.d.y, mn.y, mx.y); if (t1 <= t0) return false;
			slab(r.o.z, r.d.z, mn.z, mx.z); if (t1 <= t0) return false;

			tEnter = t0;
			tExit = t1;
			return true;
		}
	};
}