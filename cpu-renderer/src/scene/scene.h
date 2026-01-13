// cpu-renderer/include/scene/scene.h

#pragma once

#include <vector>
#include <memory>

#include "accel/bvh.h"
#include "scene/shape.h"
#include "scene/material.h"
#include "scene/hit.h"
#include "scene/primitive.h"
#include "scene/triangle_ref.h"
#include "scene/medium.h"
#include "core/ray.h"

namespace COR {

	struct LightSample {
		Vec3 wi;		// direction
		float dist;		// distance
		float pdf;		// probability density fuction
		Vec3 Le;		// radiance
	};

	struct Scene {
		std::vector<std::unique_ptr<Primitive>> primitives;
		std::vector<Material> materials;

		std::vector<int> lightIds;

		std::unique_ptr<Primitive> accel;
		std::unique_ptr<HomogeneousMedium> medium;
		AABB mediumBounds;

		template <class ShapeT, class... Args>
		void addShape(int materialId, Args&&... args) {
			auto s = std::make_shared<ShapeT>(std::forward<Args>(args)...);
			primitives.emplace_back(std::make_unique<GeometricPrimitive>(std::move(s), materialId));
		}

		bool intersect(const Ray& r, float tMin, float tMax, HitRecord& rec) const {
			if (accel) return accel->intersect(r, tMin, tMax, rec);

			HitRecord tmp;
			bool hitAnything = false;
			float closest = tMax;

			for (const auto& p : primitives) {
				if (p->intersect(r, tMin, closest, tmp)) {
					hitAnything = true;
					closest = tmp.t;
					rec = tmp;
				}
			}

			return hitAnything;
		}

		// Build light list by scanning spheres whose material is emissive
		void buildLightList() {
			lightIds.clear();
			for (int i = 0; i < (int)primitives.size(); ++i) {
				int mid = primitives[i]->materialId();
				if (mid >= 0 && mid < (int)materials.size()) {
					if (materials[mid].type == MaterialType::Emissive)
						lightIds.push_back(i);
				}
			}
		}

		bool hasLights() const { return !lightIds.empty(); }

		LightSample sampleOneLight(const Vec3& p, RNG& rng) const {
			LightSample ls;
			ls.pdf = 0.0f;
			ls.dist = 0.0f;
			ls.wi = Vec3{ 0.0f };
			ls.Le = Vec3{ 0.0f };

			if (lightIds.empty()) return ls;

			// Pick a light uniformly
			int nLights = (int)lightIds.size();
			int pick = (int)(rng.nextFloat01() * nLights);
			if (pick >= nLights) pick = nLights - 1;

			const Primitive* lightPrim = primitives[lightIds[pick]].get();
			const Shape* lightShape = lightPrim->shape();
			if (!lightShape) return ls;

			const Material& m = materials[lightPrim->materialId()];

			// Sample a point uniformly on sphere surface
			Vec3 xL, nL;
			float pdf_area = 0.0f;
			if (!lightShape->sampleSurface(rng, xL, nL, pdf_area) || pdf_area <= 0.0f)
				return ls;

			Vec3 toL = xL - p;
			float dist2 = dot(toL, toL);
			float dist = std::sqrt(dist2);
			if (dist <= 0.0f) return ls;

			Vec3 wi = toL / dist;

			// convert area pdf to solid angle pdf
			// pdf_solid = dist^2 / (cosThetaLight * Area)
			float cosThetaLight = std::fabs(dot(nL, wi * -1.0f));
			if (cosThetaLight <= 0.0f) return ls;

			// mixture pdf for uniform light selection
			float pdf_solid = (dist2 / cosThetaLight) * pdf_area;

			pdf_solid *= (1.0f / (float)nLights);

			ls.wi = wi;
			ls.dist = dist;
			ls.pdf = pdf_solid;
			ls.Le = m.emitted();
			return ls;
		}

		// Visibility test for direct lighting (shadow ray)
		bool visible(const Vec3& p, const Vec3& wi, float dist) const {
			// Offset along direction to avoid self-intersection
			Ray shadowRay{ p, wi };
			HitRecord tmp;
			// If anything hit before reaching the light point, it's occluded
			return !intersect(shadowRay, 0.001f, dist - 0.002f, tmp);
		}

		// evaluate the light sampling PDF (mixture) for a given direction wi
		float pdfLight(const Vec3& p, const Vec3& wi) const {
			if (lightIds.empty()) return 0.0f;

			Vec3 dir = normalize(wi);
			float sum = 0.0f;

			Ray r{ p,dir };

			for (int idx : lightIds) {
				const Primitive* lightPrim = primitives[idx].get();
				const Shape* lightShape = lightPrim->shape();
				if (!lightShape) continue;

				float A = lightShape->area();
				if (A <= 0.0f) continue;

				HitRecord lr;
				if (!lightShape->intersect(r, 0.001f, 1e30f, lr)) continue;

				float dist2 = lr.t * lr.t;

				float cosThetaLight = std::fabs(dot(lr.n, dir * -1.0f));
				if (cosThetaLight <= 0.0f) continue;

				float pdf_area = 1.0f / A;
				float pdf_solid = (dist2 / cosThetaLight) * pdf_area;

				sum += pdf_solid;
			}

			return sum * (1.0f / (float)lightIds.size());
		}

		void buildBVH(int leafSize = 4) {
			std::vector<Primitive*> ptrs;
			ptrs.reserve(primitives.size());
			for (auto& p : primitives) ptrs.push_back(p.get());

			accel = std::make_unique<BVHAccel>(std::move(ptrs), leafSize);
		}

		void addMeshAsTriangles(int materialId, MeshData&& md) {
			auto storage = std::make_shared<MeshStorage>();
			storage->data = std::move(md);

			const uint32_t nTris = (uint32_t)storage->data.indices.size();
			primitives.reserve(primitives.size() + nTris);

			for (uint32_t i = 0; i < nTris; ++i) {
				auto triShape = std::make_shared<TriangleRef>(storage, i);
				primitives.emplace_back(std::make_unique<GeometricPrimitive>(triShape, materialId));
			}
		}

		bool hasMedium() const {
			return (bool)medium && (medium->sigma_t.x > 0.0f || medium->sigma_t.y > 0.0f || medium->sigma_t.z > 0.0f);
		}
	};
}