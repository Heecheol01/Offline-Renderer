# include "scene/triangle_ref.h"

namespace COR {
	static inline bool intersectTriMT(
		const Ray& r, const Vec3& v0, const Vec3& v1, const Vec3& v2,
		float tMin, float tMax, float& t, float& u, float& v)
	{
		const Vec3 e1 = v1 - v0;
		const Vec3 e2 = v2 - v0;
		const Vec3 p = cross(r.d, e2);
		const float det = dot(e1, p);
		if (std::fabs(det) < 1e-8f) return false;
		const float invDet = 1.0f / det;

		const Vec3 s = r.o - v0;
		u = dot(s, p) * invDet;
		if (u < 0.0f || u > 1.0f) return false;

		const Vec3 q = cross(s, e1);
		v = dot(r.d, q) * invDet;
		if (v < 0.0f || (u + v) > 1.0f) return false;

		t = dot(e2, q) * invDet;
		if (t < tMin || t > tMax) return false;

		return true;
	}

	static inline float triArea(const Vec3& a, const Vec3& b, const Vec3& c) {
		return 0.5f * length(cross(b - a, c - a));
	}

	bool TriangleRef::intersect(const Ray& r, float tMin, float tMax, HitRecord& rec) const {
		const auto& md = mesh->data;
		const auto idx = md.indices[tri];

		const Vec3& v0 = md.positions[idx.x];
		const Vec3& v1 = md.positions[idx.y];
		const Vec3& v2 = md.positions[idx.z];

		float t, u, v;
		if (!intersectTriMT(r, v0, v1, v2, tMin, tMax, t, u, v)) return false;

		const Vec3 gn = normalize(cross(v1 - v0, v2 - v0));

		Vec3 sn = gn;
		if (md.normals.size() == md.positions.size()) {
			Vec3 n0 = md.normals[idx.x];
			Vec3 n1 = md.normals[idx.y];
			Vec3 n2 = md.normals[idx.z];
			Vec3 interp = n0 * (1.0f - u - v) + n1 * u + n2 * v;
			if (dot(interp, interp) > 1e-10f) sn = normalize(interp);
		}

		rec.t = t;
		rec.p = r.at(t);
		rec.setFaceNormal(r, sn);

		return true;
	}

	bool TriangleRef::bounds(AABB& out) const {
		const auto& md = mesh->data;
		const auto idx = md.indices[tri];

		out = AABB{};
		out.expand(md.positions[idx.x]);
		out.expand(md.positions[idx.y]);
		out.expand(md.positions[idx.z]);

		const float eps = 1e-6f;
		out.mn = out.mn - Vec3(eps);
		out.mx = out.mx + Vec3(eps);
		return true;
	}

	float TriangleRef::area() const {
		const auto& md = mesh->data;
		const auto idx = md.indices[tri];
		return triArea(md.positions[idx.x], md.positions[idx.y], md.positions[idx.z]);
	}

	bool TriangleRef::sampleSurface(RNG& rng, Vec3& pos, Vec3& normal, float& pdf_area) const {
		const auto& md = mesh->data;
		const auto idx = md.indices[tri];

		const Vec3& a = md.positions[idx.x];
		const Vec3& b = md.positions[idx.y];
		const Vec3& c = md.positions[idx.z];

		float A = triArea(a, b, c);
		if (A <= 0.0f) return false;

		float r1 = rng.nextFloat01();
		float r2 = rng.nextFloat01();
		float su = std::sqrt(r1);
		float u = 1.0f - su;
		float v = r2 * su;
		float w = 1.0f - u - v;

		pos = a * u + b * v + c * w;
		normal = normalize(cross(b - a, c - a));
		pdf_area = 1.0f / A;
		return true;
	}
}