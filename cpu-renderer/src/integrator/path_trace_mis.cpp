
#include "integrator/path_trace_mis.h"
#include "core/sampling.h"
#include <cmath>

namespace COR {
    Vec3 trace_path_mis(const Ray& r0, const Scene& scene, RNG& rng, int maxDepth) {
        const float eps = 0.001f;

        Ray r = r0;
        Vec3 L{ 0.0f, 0.0f, 0.0f };
        Vec3 beta{ 1.0f, 1.0f, 1.0f };

        bool prevSpecular = true;  // camera ray treated as specular
        float prevPdfBsdf = 1.0f;
        Vec3 prevP{ 0.0f, 0.0f, 0.0f };
        Vec3 prevWi{ 0.0f, 0.0f, 0.0f };

        for (int depth = 0; depth < maxDepth; ++depth) {
            HitRecord rec;
            if (!scene.intersect(r, eps, 1e30f, rec)) {
                break; // closed scene => black
            }

            const Material& mat = scene.materials[rec.materialId];

            // Light hit (emission) with MIS
            if (mat.type == MaterialType::Emissive) {
                if (prevSpecular) {
                    L += beta * mat.emitted();
                }
                else {
                    float pdfLight = scene.pdfLight(prevP, prevWi);
                    float w = powerHeuristic(prevPdfBsdf, pdfLight);
                    L += beta * mat.emitted() * w;
                }
                break;
            }

            // offset position to reduce self-intersection
            Vec3 p = rec.p + rec.n * eps;

            // Direct lighting: light sampling + MIS weight (diffuse only)
            if (mat.type == MaterialType::Lambert && scene.hasLights()) {
                LightSample ls = scene.sampleOneLight(p, rng);
                if (ls.pdf > 0.0f) {
                    float cosOnSurface = dot(rec.n, ls.wi);
                    if (cosOnSurface > 0.0f && scene.visible(p, ls.wi, ls.dist)) {
                        Vec3 f = mat.albedo * (1.0f / PI);
                        float pdfBsdf = cosOnSurface / PI; // Lambert pdf
                        float w = powerHeuristic(ls.pdf, pdfBsdf);

                        L += beta * f * ls.Le * (cosOnSurface / ls.pdf) * w;
                    }
                }
            }

            // Continue path by BSDF sampling
            Ray scattered;
            Vec3 attenuation;
            if (!mat.scatter(r, rec, rng, attenuation, scattered)) {
                break;
            }

            scattered.o = p;
            scattered.d = normalize(scattered.d);

            // MIS bookkeeping for next possible light hit
            if (mat.type == MaterialType::Lambert) {
                prevSpecular = false;
                prevP = p;
                prevWi = scattered.d;
                prevPdfBsdf = lambertPdf(rec.n, scattered.d);
            }
            else {
                prevSpecular = true;
                prevPdfBsdf = 1.0f;
                prevP = p;
                prevWi = scattered.d;
            }

            beta = beta * attenuation;
            r = scattered;

            // Russian roulette
            if (depth >= 5) {
                float pRR = clamp(std::fmax(beta.x, std::fmax(beta.y, beta.z)), 0.05f, 0.95f);
                if (rng.nextFloat01() > pRR) break;
                beta = beta * (1.0f / pRR);
            }
        }

        return L;
    }

}