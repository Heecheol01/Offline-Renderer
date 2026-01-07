// src/integrator/wavefront.cpp


#include "integrator/wavefront.h"

#include "film/film.h"
#include "scene/camera.h"
#include "scene/scene.h"
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
		std::vector<uint8_t> eventType;

		void resize(size_t k) {
			hitFlag.resize(k);
			t.resize(k);
			p.resize(k);
			n.resize(k);
			materialId.resize(k);
			eventType.resize(k);
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

	static float medium_segment_length(const COR::AABB& box, const COR::Ray& r, float tMin, float tMax) {
		float te, tx;
		if (!box.intersectRange(r, tMin, tMax, te, tx)) return 0.0f;
		float len = tx - te;
		return (len > 0.0f) ? len : 0.0f;
	}

	// ----------------------------
	// SoA Helpers
	// ----------------------------
	static void intersect_stage(
		const COR::Scene& scene,
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
			if (scene.intersect(r, EPS, 1e30f, rec)) {
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

	static void medium_stage(
		const COR::Scene& scene,
		PathSoA& paths,
		HitSoA& hits,
		const std::vector<int>& active,
		std::vector<int>& surfaceList,
		std::vector<int>& mediumList,
		std::vector<int>& missListFinal
	) {
		surfaceList.clear();
		mediumList.clear();
		missListFinal.clear();

		if (!scene.hasMedium()) {
			for (int i : active) {
				if (!paths.alive[i]) { hits.eventType[i] = 0; missListFinal.push_back(i); continue; }
				if (hits.hitFlag[i]) { hits.eventType[i] = 1; surfaceList.push_back(i); }
				else { hits.eventType[i] = 0; missListFinal.push_back(i); }
			}
			return;
		}

		const COR::HomogeneousMedium& med = *scene.medium;

		const float sigmaT = med.sigma_t.x;
		if (sigmaT <= 0.0f) {
			for (int i : active) {
				if (!paths.alive[i]) { hits.eventType[i] = 0; missListFinal.push_back(i); }
				if (hits.hitFlag[i]) { hits.eventType[i] = 1; surfaceList.push_back(i); }
				else { hits.eventType[i] = 0; missListFinal.push_back(i); }
			}
			return;
		}

		for (int i : active) {
			if (!paths.alive[i]) { hits.eventType[i] = 0; missListFinal.push_back(i); }

			COR::Ray r{ paths.rayO[i], paths.rayD[i] };

			float tSurf = hits.hitFlag[i] ? hits.t[i] : 1e30f;

			float tEnter, tExit;
			bool inMed = scene.mediumBounds.intersectRange(r, EPS, tSurf, tEnter, tExit);
			if (!inMed) {
				if (hits.hitFlag[i]) { hits.eventType[i] = 1; surfaceList.push_back(i); }
				else { hits.eventType[i] = 0; missListFinal.push_back(i); }
				continue;
			}

			float segLen = tExit - tEnter;
			if (segLen <= 0.0f) {
				if (hits.hitFlag[i]) { hits.eventType[i] = 1; surfaceList.push_back(i); }
				else { hits.eventType[i] = 0; missListFinal.push_back(i); }
				continue;
			}

			float u = paths.rng[i].nextFloat01();
			if (u < 1e-6f) u = 1e-6f;
			float tSample = -std::log(u) / sigmaT;

			if (tSample < segLen) {
				// medium scattering event
				float tEvent = tEnter + tSample;

				// beta *= Tr(tSample) * (sigma_s / sigma_t)
				COR::Vec3 Tr = med.Tr(tSample);

				COR::Vec3 albedo{
					(med.sigma_t.x > 0.0f) ? (med.sigma_s.x / med.sigma_t.x) : 0.0f,
					(med.sigma_t.y > 0.0f) ? (med.sigma_s.y / med.sigma_t.y) : 0.0f,
					(med.sigma_t.z > 0.0f) ? (med.sigma_s.z / med.sigma_t.z) : 0.0f
				};

				paths.beta[i] = paths.beta[i] * albedo;
				//paths.beta[i] = paths.beta[i] * (Tr * albedo);

				hits.eventType[i] = 2;
				hits.hitFlag[i] = 1;
				hits.t[i] = tEvent;
				hits.p[i] = r.at(tEvent);
				hits.n[i] = COR::Vec3{ 0.0f };
				hits.materialId[i] = -1;

				mediumList.push_back(i);
			}
			else {
				// no scattering before surface/miss: beta *= Tr(segLen)
				//paths.beta[i] = paths.beta[i] * med.Tr(segLen);

				if (hits.hitFlag[i]) { hits.eventType[i] = 1; surfaceList.push_back(i); }
				else { hits.eventType[i] = 0; missListFinal.push_back(i); }
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
		const COR::Scene& scene,
		PathSoA& paths,
		const HitSoA& hits,
		const std::vector<int>& hitList,
		std::vector<int>& nonEmissiveList
	) {
		nonEmissiveList.clear();

		for (int i : hitList) {
			if (!paths.alive[i]) continue;

			const COR::Material& mat = scene.materials[hits.materialId[i]];
			if (!mat.isEmissive()) {
				nonEmissiveList.push_back(i);
				continue;
			}

			// if the current surface is emissive, add emission with MIS
			if (paths.prevSpecular[i]) {
				paths.L[i] += paths.beta[i] * mat.emitted();
			}
			else {
				float pdfLight = scene.pdfLight(paths.prevP[i], paths.prevWi[i]);
				float w = COR::powerHeuristic(paths.prevPdfBsdf[i], pdfLight);
				paths.L[i] += paths.beta[i] * mat.emitted() * w;
			}
			paths.alive[i] = 0;
		}
	}

	static void medium_direct_lighting_stage(
		const COR::Scene& scene,
		PathSoA& paths,
		const HitSoA& hits,
		const std::vector<int>& mediumList
	) {
		if (!scene.hasLights()) return;
		if (!scene.hasMedium()) return;

		const COR::HomogeneousMedium& med = *scene.medium;

		for (int i : mediumList) {
			if (!paths.alive[i]) continue;

			COR::Vec3 p = hits.p[i];	// medium point
			COR::Vec3 wo = COR::normalize(-paths.rayD[i]);

			COR::LightSample ls = scene.sampleOneLight(p, paths.rng[i]);
			if (ls.pdf <= 0.0f) continue;

			// phase eval/pdf
			float phaseVal = med.phase.eval(wo, ls.wi);
			float pdfPhase = med.phase.pdf(wo, ls.wi);
			if (pdfPhase <= 0.0f || phaseVal <= 0.0f) continue;

			float w = COR::powerHeuristic(ls.pdf, pdfPhase);

			//visibility
			COR::Vec3 o = p + ls.wi * EPS;
			COR::Ray shadowRay{ o, ls.wi };
			COR::HitRecord tmp;
			bool blocked = scene.intersect(shadowRay, EPS, ls.dist - 2.0f * EPS, tmp);
			if (blocked) continue;

			// medium transmittance alnge shadow segment inside medium bounds
			float lenIn = medium_segment_length(scene.mediumBounds, shadowRay, EPS, ls.dist - 2.0f * EPS);
			COR::Vec3 Tr = med.Tr(lenIn);

			// Ld = beta * phase * Le * Tr / pdf_light * w
			paths.L[i] += paths.beta[i] * (phaseVal * (1.0f / ls.pdf)) * (ls.Le * Tr) * w;
		}
	}

	static void direct_lighting_stage(
		const COR::Scene& scene,
		PathSoA& paths,
		const HitSoA& hits,
		const std::vector<int>& nonEmissiveList,
		ShadowBatch& shadow
	) {
		shadow.clear();
		if (!scene.hasLights()) return;

		const bool useMedium = scene.hasMedium();
		const COR::HomogeneousMedium* med = useMedium ? scene.medium.get() : nullptr;

		for (int i : nonEmissiveList) {
			if (!paths.alive[i]) continue;

			const COR::Material& mat = scene.materials[hits.materialId[i]];

			// Only do NEE for non-delta BSDFs (diffuse/glossy)
			if (mat.isDelta()) continue;

			COR::Vec3 p = hits.p[i] + hits.n[i] * EPS;

			// wo is direction toward the camera/previous vertex
			COR::Vec3 wo = COR::normalize(-paths.rayD[i]);

			COR::LightSample ls = scene.sampleOneLight(p, paths.rng[i]);
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

			if (useMedium) {
				COR::Ray shadowRay{ p, ls.wi };
				float lenIn = medium_segment_length(scene.mediumBounds, shadowRay, EPS, ls.dist - 2.0f * EPS);
				c = c * med->Tr(lenIn);
			}

			shadow.pathIdx.push_back(i);
			shadow.o.push_back(p);
			shadow.d.push_back(ls.wi);
			shadow.tMax.push_back(ls.dist);
			shadow.contrib.push_back(c);
		}
	}

	static void shadow_trace_stage(
		const COR::Scene& scene,
		PathSoA& paths,
		const ShadowBatch& shadow
	) {
		for (size_t k = 0; k < shadow.pathIdx.size(); ++k) {
			int i = shadow.pathIdx[k];
			if (!paths.alive[i]) continue;

			COR::Ray r{ shadow.o[k], shadow.d[k] };

			COR::HitRecord tmp;
			bool blocked = scene.intersect(r, EPS, shadow.tMax[k] - 2.0f * EPS, tmp);
			if (!blocked) {
				paths.L[i] += shadow.contrib[k];
			}
		}
	}

	static void bsdf_sample_spawn_stage(
		const COR::Scene& scene,
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

			const COR::Material& mat = scene.materials[hits.materialId[i]];

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

	static void phase_sample_spawn_stage(
		const COR::Scene& scene,
		PathSoA& paths,
		HitSoA& hits,
		const std::vector<int>& mediumList,
		std::vector<int>& nextActive,
		int depth
	) {
		nextActive.clear();
		nextActive.reserve(mediumList.size());

		if (!scene.hasMedium()) return;
		const COR::HomogeneousMedium& med = *scene.medium;

		for (int i : mediumList) {
			if (!paths.alive[i]) continue;

			COR::Vec3 p = hits.p[i];
			COR::Vec3 wo = COR::normalize(-paths.rayD[i]);

			COR::PhaseSample ps;
			if (!med.phase.sample(wo, paths.rng[i], ps)) {
				paths.alive[i] = 0;
				continue;
			}
			if (ps.pdf <= 0.0f) {
				paths.alive[i] = 0;
				continue;
			}

			COR::Vec3 wi = COR::normalize(ps.wi);

			// throughput update: beta *= p/pdf
			paths.beta[i] = paths.beta[i] * (ps.p / ps.pdf);

			// spawn next ray
			paths.rayO[i] = p + wi * EPS;
			paths.rayD[i] = wi;

			// MIS bookkeeping
			paths.prevSpecular[i] = 0;
			paths.prevPdfBsdf[i] = ps.pdf;
			paths.prevP[i] = p;
			paths.prevWi[i] = wi;

			// Russian roulette
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
		const Scene& scene,
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

		std::vector<int> hitList, missList;
		std::vector<int> surfaceList, mediumList, missListFinal;
		std::vector<int> nonEmissiveList;
		std::vector<int> nextActiveSurface, nextActiveMedium, nextActive;

		hitList.reserve(N); missList.reserve(N); 
		surfaceList.reserve(N); mediumList.reserve(N); missListFinal.reserve(N);
		nonEmissiveList.reserve(N); 
		nextActiveSurface.reserve(N); nextActiveMedium.reserve(N); nextActive.reserve(N);

		for (int depth = 0; depth < maxDepth && !active.empty(); ++depth) {
			intersect_stage(scene, paths, hits, active, hitList, missList);
			medium_stage(scene, paths, hits, active, surfaceList, mediumList, missListFinal);
			miss_stage(paths, missListFinal);
			emissive_stage(scene, paths, hits, surfaceList, nonEmissiveList);

			ShadowBatch shadow;
			direct_lighting_stage(scene, paths, hits, nonEmissiveList, shadow);
			shadow_trace_stage(scene, paths, shadow);

			medium_direct_lighting_stage(scene, paths, hits, mediumList);

			bsdf_sample_spawn_stage(scene, paths, hits, nonEmissiveList, nextActiveSurface, depth);
			phase_sample_spawn_stage(scene, paths, hits, mediumList, nextActiveMedium, depth);

			nextActive.clear();
			nextActive.reserve(nextActiveSurface.size() + nextActiveMedium.size());
			nextActive.insert(nextActive.end(), nextActiveSurface.begin(), nextActiveSurface.end());
			nextActive.insert(nextActive.end(), nextActiveMedium.begin(), nextActiveMedium.end());

			active.swap(nextActive);
		}

		// deposit one-sample contribution per pixel
		for (size_t i = 0; i < N; ++i) {
			film.addSample(paths.pixX[i], paths.pixY[i], paths.L[i]);
		}
	}
}