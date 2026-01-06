// src/integrator/wavefront.cpp


#include "integrator/wavefront.h"

#include "film/film.h"
#include "scene/camera.h"
#include "scene/world.h"
#include "scene/hit.h"
#include "scene/material.h"

#include "core/ray.h"
#include "core/vector.h"
#include "core/rng.h"
#include "core/random.h"
#include "core/sampling.h"

#include <vector>
#include <algorithm>
#include <cmath>

namespace {
	static constexpr float EPS = 0.001f;

	// ----------------------------
	// SoA Buffers
	// ----------------------------
	struct PathSoA {
		// per-path identity
		std::vector<int> pixX;
		std::vector<int> pixY;

		// ray
		std::vector<COR::Vec3> rayO;
		std::vector<COR::Vec3> rayD;

		// path state
		std::vector<COR::Vec3> L;
		std::vector<COR::Vec3> beta;
		std::vector<uint8_t> alive;

		// MIS bookkeeping for BSDF hit light
		std::vector<uint8_t> prevSpecular;
		std::vector<float> prevPdfBsdf;
		std::vector<COR::Vec3> prevP;
		std::vector<COR::Vec3> prevWi;

		// RNG state (keep your existing RNG object per path for now)
		std::vector<COR::RNG> rng;

		void resize(size_t n) {
			pixX.resize(n); pixY.resize(n);
			rayO.resize(n); rayD.resize(n);
			L.resize(n); beta.resize(n);
			alive.resize(n);

			prevSpecular.resize(n);
			prevPdfBsdf.resize(n);
			prevP.resize(n);
			prevWi.resize(n);

			rng.resize(n);
		}
	};

	struct HitSoA {
		// hit data per path
		std::vector<uint8_t> hitFlag;
		std::vector<float> t;
		std::vector<COR::Vec3> p;
		std::vector<COR::Vec3> n;
		std::vector<int> materialId;

		void resize(size_t k) {
			hitFlag.resize(k);
			t.resize(k);
			p.resize(k);
			n.resize(k);
			materialId.resize(k);
		}
	};

	// shadow query batch
	struct ShadowBatch {
		std::vector<int> pathIdx;
		std::vector<COR::Vec3> o;
		std::vector<COR::Vec3> d;
		std::vector<float> tMax;
		std::vector<COR::Vec3> contrib;

		void clear() {
			pathIdx.clear(); o.clear(); d.clear(); tMax.clear(); contrib.clear();
		}
	};

	// ----------------------------
	// SoA Helpers
	// ----------------------------
	static void intersect_stage(
		const COR::World& world,
		const PathSoA& paths,
		HitSoA& hits,
		const std::vector<int>& active,
		std::vector<int>& hitList,
		std::vector<int>& missList
	) {
		hitList.clear();
		missList.clear();

		for (int i : active) {
			if (!paths.alive[i]) {
				missList.push_back(i);
				continue;
			}

			COR::Ray r{ paths.rayO[i], paths.rayD[i] };

			COR::HitRecord rec;
			if (world.intersect(r, EPS, 1e30f, rec)) {
				hits.hitFlag[i] = 1;
				hits.t[i] = rec.t;
				hits.p[i] = rec.p;
				hits.n[i] = rec.n;
				hits.materialId[i] = rec.materialId;
				hitList.push_back(i);
			}
			else {
				hits.hitFlag[i] = 0;
				missList.push_back(i);
			}
		}
	}

	static void miss_stage(PathSoA& paths, const std::vector<int>& missList) {
		// closed scene: miss => terminate with black
		for (int i : missList) {
			paths.alive[i] = 0;
		}
	}

	static void emissive_stage(
		const COR::World& world,
		PathSoA& paths,
		const HitSoA& hits,
		const std::vector<int>& hitList,
		std::vector<int>& nonEmissiveList
	) {
		nonEmissiveList.clear();

		for (int i : hitList) {
			if (!paths.alive[i]) continue;

			const COR::Material& mat = world.materials[hits.materialId[i]];
			if (!mat.isEmissive()) {
				nonEmissiveList.push_back(i);
				continue;
			}

			// if the current surface is emissive, add emission with MIS
			if (paths.prevSpecular[i]) {
				paths.L[i] += paths.beta[i] * mat.emitted();
			}
			else {
				float pdfLight = world.pdfLight(paths.prevP[i], paths.prevWi[i]);
				float w = COR::powerHeuristic(paths.prevPdfBsdf[i], pdfLight);
				paths.L[i] += paths.beta[i] * mat.emitted() * w;
			}
			paths.alive[i] = 0;
		}
	}

	static void direct_lighting_stage(
		const COR::World& world,
		PathSoA& paths,
		const HitSoA& hits,
		const std::vector<int>& nonEmissiveList,
		ShadowBatch& shadow
	) {
		shadow.clear();
		if (!world.hasLights()) return;

		for (int i : nonEmissiveList) {
			if (!paths.alive[i]) continue;

			const COR::Material& mat = world.materials[hits.materialId[i]];

			// Only do NEE for non-delta BSDFs (diffuse/glossy)
			if (mat.isDelta()) continue;

			COR::Vec3 p = hits.p[i] + hits.n[i] * EPS;

			// wo is direction toward the camera/previous vertex
			COR::Vec3 wo = COR::normalize(-paths.rayD[i]);

			COR::LightSample ls = world.sampleOneLight(p, paths.rng[i]);
			if (ls.pdf <= 0.0f) continue;

			float cosOnSurface = dot(hits.n[i], ls.wi);
			if (cosOnSurface <= 0.0f) continue;

			// BSDF eval + BSDF pdf
			COR::Vec3 f = mat.evalBSDF(wo, ls.wi, hits.n[i]);
			float pdfBsdf = mat.pdfBSDF(wo, ls.wi, hits.n[i]);
			if (pdfBsdf <= 0.0f) continue;

			// MIS weight (light sampling vs BSDF sampling)
			float w = COR::powerHeuristic(ls.pdf, pdfBsdf);

			// Contribution if visible
			// Ld = beta * f * Le * cos / pdf_light * w
			COR::Vec3 c = paths.beta[i] * f * ls.Le * (cosOnSurface / ls.pdf) * w;

			shadow.pathIdx.push_back(i);
			shadow.o.push_back(p);
			shadow.d.push_back(ls.wi);
			shadow.tMax.push_back(ls.dist);
			shadow.contrib.push_back(c);
		}
	}

	static void shadow_trace_stage(
		const COR::World& world,
		PathSoA& paths,
		const ShadowBatch& shadow
	) {
		for (size_t k = 0; k < shadow.pathIdx.size(); ++k) {
			int i = shadow.pathIdx[k];
			if (!paths.alive[i]) continue;

			COR::Ray r{ shadow.o[k], shadow.d[k] };

			COR::HitRecord tmp;
			bool blocked = world.intersect(r, EPS, shadow.tMax[k] - 2.0f * EPS, tmp);
			if (!blocked) {
				paths.L[i] += shadow.contrib[k];
			}
		}
	}

	static void bsdf_sample_spawn_stage(
		const COR::World& world,
		PathSoA& paths,
		const HitSoA& hits,
		const std::vector<int>& nonEmissiveList,
		std::vector<int>& nextActive,
		int depth
	) {
		nextActive.clear();
		nextActive.reserve(nonEmissiveList.size());

		for (int i : nonEmissiveList) {
			if (!paths.alive[i]) continue;

			const COR::Material& mat = world.materials[hits.materialId[i]];

			COR::Vec3 p = hits.p[i] + hits.n[i] * EPS;
			COR::Vec3 n = hits.n[i];

			COR::Vec3 wo = COR::normalize(-paths.rayD[i]);

			COR::BSDFSample bs;
			if (!mat.sampleBSDF(wo, n, paths.rng[i], bs)) {
				// absorb / terminate
				paths.alive[i] = 0;
				continue;
			}

			COR::Vec3 wi = COR::normalize(bs.wi);
			float cosOnSurface = std::fabs(dot(n, wi));
			if (bs.pdf <= 0.0f || cosOnSurface <= 0.0f) {
				paths.alive[i] = 0;
				continue;
			}

			// throughput update: beta *= f * cos / pdf
			paths.beta[i] = paths.beta[i] * (bs.f * (cosOnSurface / bs.pdf));

			// spawn next ray
			paths.rayO[i] = p;
			paths.rayD[i] = wi;

			// update MIS bookkeeping for BSDF hit light on next intersection
			if (bs.isDelta) {
				paths.prevSpecular[i] = 1;
				paths.prevPdfBsdf[i] = 1.0f;
			}
			else {
				paths.prevSpecular[i] = 0;
				paths.prevPdfBsdf[i] = bs.pdf;
			}
			paths.prevP[i] = p;
			paths.prevWi[i] = wi;

			// Russian roulette after a few bounces
			if (depth >= 5) {
				float m = std::fmax(paths.beta[i].x, std::fmax(paths.beta[i].y, paths.beta[i].z));
				float pRR = COR::clamp(m, 0.05f, 0.95f);
				if (paths.rng[i].nextFloat01() > pRR) {
					paths.alive[i] = 0;
					continue;
				}
				paths.beta[i] = paths.beta[i] * (1.0f / pRR);
			}

			nextActive.push_back(i);
		}
	}
}

namespace COR {
	void render_tile_wavefront_sample(
		Film& film,
		const Camera& cam,
		const World& world,
		int W, int H,
		int tileX0, int tileY0, int tileX1, int tileY1,
		int sampleIndex,
		int maxDepth
	) {
		const int tileW = tileX1 - tileX0;
		const int tileH = tileY1 - tileY0;
		const size_t N = (size_t)tileW * (size_t)tileH;

		PathSoA paths;
		paths.resize(N);

		HitSoA hits;
		hits.resize(N);

		std::vector<int> active;
		active.reserve(N);

		// init paths for this (tile, sampleIndex)
		size_t idx = 0;
		for (int y = tileY0; y < tileY1; ++y) {
			for (int x = tileX0; x < tileX1; ++x, ++idx) {
				paths.pixX[idx] = x;
				paths.pixY[idx] = y;

				uint64_t seed = COR::makeSeed2((uint32_t)x, (uint32_t)y, (uint32_t)sampleIndex);
				paths.rng[idx] = COR::RNG(seed, 1);

				float u = (float(x) + paths.rng[idx].nextFloat01()) / float(W - 1);
				float v = (float(y) + paths.rng[idx].nextFloat01()) / float(H - 1);

				COR::Ray r = cam.getRay(u, v);
				paths.rayO[idx] = r.o;
				paths.rayD[idx] = COR::normalize(r.d);

				paths.L[idx] = COR::Vec3{ 0 };
				paths.beta[idx] = COR::Vec3{ 1 };
				paths.alive[idx] = 1;

				// camera ray treated as specular for emission visibility
				paths.prevSpecular[idx] = 1;
				paths.prevPdfBsdf[idx] = 1.0f;
				paths.prevP[idx] = COR::Vec3{ 0 };
				paths.prevWi[idx] = COR::Vec3{ 0 };

				active.push_back((int)idx);
			}
		}

		std::vector<int> hitList, missList, nonEmissiveList, nextActive;
		hitList.reserve(N); missList.reserve(N); nonEmissiveList.reserve(N);  nextActive.reserve(N);

		ShadowBatch shadow;

		for (int depth = 0; depth < maxDepth && !active.empty(); ++depth) {
			intersect_stage(world, paths, hits, active, hitList, missList);
			miss_stage(paths, missList);
			emissive_stage(world, paths, hits, hitList, nonEmissiveList);

			direct_lighting_stage(world, paths, hits, nonEmissiveList, shadow);
			shadow_trace_stage(world, paths, shadow);

			bsdf_sample_spawn_stage(world, paths, hits, nonEmissiveList, nextActive, depth);
			active.swap(nextActive);
		}

		// deposit one-sample contribution per pixel
		for (size_t i = 0; i < N; ++i) {
			film.addSample(paths.pixX[i], paths.pixY[i], paths.L[i]);
		}
	}
}