// apps/render_cli/main.cpp

#include "film/film.h"
#include "io/write_image.h"
#include "core/vector.h"
#include "core/rng.h"
#include "core/ray.h"
#include "core/random.h"
#include "core/sampling.h"
#include "scene/camera.h"
#include "scene/world.h"

#include <chrono>
#include <iomanip>
#include <sstream>

#include <iostream>

static std::string format_hhmmss(double seconds) {
    if (seconds < 0) seconds = 0;

    long long s = (long long)(seconds + 0.5);
    long long h = s / 3600; s %= 3600;
    long long m = s / 60;   s %= 60;

    std::ostringstream oss;
    oss << std::setfill('0')
        << std::setw(2) << h << ":"
        << std::setw(2) << m << ":"
        << std::setw(2) << s;
    return oss.str();
}

struct RenderProgress {
    int W, H, SPP;
    long long totalSamples;
    long long doneSamples = 0;

    std::chrono::steady_clock::time_point t0;
    std::chrono::steady_clock::time_point lastPrint;

    RenderProgress(int w, int h, int spp)
        : W(w), H(h), SPP(spp) {
        totalSamples = (long long)W * (long long)H * (long long)SPP;
        t0 = std::chrono::steady_clock::now();
        lastPrint = t0;
    }

    void addSamples(long long n) {
        doneSamples += n;

        auto now = std::chrono::steady_clock::now();
        double dtSincePrint = std::chrono::duration<double>(now - lastPrint).count();
        if (dtSincePrint < 0.25) return; // print 4 times/sec max
        lastPrint = now;

        double elapsed = std::chrono::duration<double>(now - t0).count();
        double rate = (elapsed > 0.0) ? (double)doneSamples / elapsed : 0.0;
        double remaining = (rate > 0.0) ? (double)(totalSamples - doneSamples) / rate : 0.0;

        double pct = 100.0 * (double)doneSamples / (double)totalSamples;

        std::cout << "\r"
            << "Progress: " << std::fixed << std::setprecision(1) << pct << "% "
            << "(" << doneSamples << "/" << totalSamples << " samples) "
            << "Elapsed " << format_hhmmss(elapsed) << " "
            << "ETA " << format_hhmmss(remaining)
            << std::flush;
    }

    void done() {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - t0).count();
        std::cout << "\r"
            << "Progress: 100.0% "
            << "(" << totalSamples << "/" << totalSamples << " samples) "
            << "Elapsed " << format_hhmmss(elapsed) << " "
            << "ETA 00:00:00"
            << "          \n";
    }
};

static COR::Vec3 sky_color(const COR::Ray& r) {
    COR::Vec3 unit_dir = COR::normalize(r.d);
    float t = 0.5f * (unit_dir.y + 1.0f);
    return COR::Vec3{ 1.0f } * (1.0f - t) + COR::Vec3{ 0.5f, 0.7f, 1.0f } * t;
}

static COR::Vec3 ray_color(const COR::Ray& r, const COR::World& world, COR::RNG& rng, int depth) {
    if (depth <= 0) return COR::Vec3{ 0,0,0 };

    COR::HitRecord rec;
    if (!world.intersect(r, 0.001f, 1e30f, rec))
        return sky_color(r);

    const COR::Material& m = world.materials[rec.materialId];

    COR::Ray scattered;
    COR::Vec3 attenuation;
    if (!m.scatter(r, rec, rng, attenuation, scattered))
        return COR::Vec3{ 0.0f };

    return attenuation * ray_color(scattered, world, rng, depth - 1);
}

static COR::Vec3 trace_path(const COR::Ray& r0, const COR::World& world, COR::RNG& rng, int maxDepth) {
    const float PI = 3.1415926535f;

    COR::Ray r = r0;
    COR::Vec3 L{ 0.0f, 0.0f, 0.0f };               // Accumulated radiance
    COR::Vec3 beta{ 1.0f, 1.0f, 1.0f };            // Path throughput
    bool specularBounce = true;                  // Treat camera ray as "specular" to see lights directly

    for (int depth = 0; depth < maxDepth; ++depth) {
        COR::HitRecord rec;
        if (!world.intersect(r, 0.001f, 1e30f, rec)) {
            // For closed scenes, use black environment
            // If you want skylight, replace with sky gradient here.
            break;
        }

        const COR::Material& mat = world.materials[rec.materialId];

        // Add emission only if this path segment is specular (or primary)
        // This avoids double counting when using NEE for diffuse surfaces.
        if (mat.type == COR::MaterialType::Emissive) {
            if (specularBounce) {
                L += beta * mat.emitted();
            }
            break;
        }

        // Next Event Estimation for diffuse only
        if (mat.type == COR::MaterialType::Lambert && world.hasLights()) {
            COR::LightSample ls = world.sampleOneLight(rec.p, rng);
            if (ls.pdf > 0.0f) {
                float cosOnSurface = dot(rec.n, ls.wi);
                if (cosOnSurface > 0.0f && world.visible(rec.p, ls.wi, ls.dist)) {
                    COR::Vec3 f = mat.albedo * (1.0f / PI); // Lambert BRDF
                    L += beta * f * ls.Le * (cosOnSurface / ls.pdf);
                }
            }
        }

        // Continue path by scattering
        COR::Ray scattered;
        COR::Vec3 attenuation;
        if (!mat.scatter(r, rec, rng, attenuation, scattered)) {
            break;
        }

        // Update throughput
        beta = beta * attenuation;

        // Determine whether next bounce is specular
        specularBounce = (mat.type == COR::MaterialType::Metal || mat.type == COR::MaterialType::Dielectric);

        r = scattered;

        // Optional: Russian roulette (recommended after a few bounces)
        if (depth >= 5) {
            float p = COR::clamp(std::fmax(beta.x, std::fmax(beta.y, beta.z)), 0.05f, 0.95f);
            if (rng.nextFloat01() > p) break;
            beta = beta * (1.0f / p);
        }
    }

    return L;
}

static COR::Vec3 trace_path_mis(const COR::Ray& r0, const COR::World& world, COR::RNG& rng, int maxDepth) {
    const float PI = 3.1415926535f;
    const float eps = 0.001f;

    COR::Ray r = r0;
    COR::Vec3 L{ 0.0f, 0.0f, 0.0f };
    COR::Vec3 beta{ 1.0f, 1.0f, 1.0f };

    // English comment: previous bounce info for MIS when we hit a light by BSDF sampling
    bool prevSpecular = true;     // camera ray treated as specular
    float prevPdfBsdf = 1.0f;
    COR::Vec3 prevP{ 0.0f, 0.0f, 0.0f };
    COR::Vec3 prevWi{ 0.0f, 0.0f, 0.0f };

    for (int depth = 0; depth < maxDepth; ++depth) {
        COR::HitRecord rec;
        if (!world.intersect(r, eps, 1e30f, rec)) {
            // English comment: closed scene => black environment
            break;
        }

        const COR::Material& mat = world.materials[rec.materialId];

        // ---- Light hit (emission) with MIS ----
        if (mat.type == COR::MaterialType::Emissive) {
            if (prevSpecular) {
                // English comment: specular paths (or primary ray) take full emission
                L += beta * mat.emitted();
            }
            else {
                // English comment: apply MIS weight between BSDF sampling and light sampling
                float pdfLight = world.pdfLight(prevP, prevWi);
                float w = COR::powerHeuristic(prevPdfBsdf, pdfLight);
                L += beta * mat.emitted() * w;
            }
            break;
        }

        // English comment: offset position to reduce self-intersection
        COR::Vec3 p = rec.p + rec.n * eps;

        // ---- Direct lighting: light sampling + MIS weight (diffuse only) ----
        if (mat.type == COR::MaterialType::Lambert && world.hasLights()) {
            COR::LightSample ls = world.sampleOneLight(p, rng);
            if (ls.pdf > 0.0f) {
                float cosOnSurface = dot(rec.n, ls.wi);
                if (cosOnSurface > 0.0f && world.visible(p, ls.wi, ls.dist)) {
                    COR::Vec3 f = mat.albedo * (1.0f / PI);
                    float pdfBsdf = cosOnSurface / PI; // Lambert pdf
                    float w = COR::powerHeuristic(ls.pdf, pdfBsdf);

                    L += beta * f * ls.Le * (cosOnSurface / ls.pdf) * w;
                }
            }
        }

        // ---- Continue path by BSDF sampling ----
        COR::Ray scattered;
        COR::Vec3 attenuation;
        if (!mat.scatter(r, rec, rng, attenuation, scattered)) {
            break;
        }

        // English comment: override origin with offset point
        scattered.o = p;
        scattered.d = COR::normalize(scattered.d);

        // English comment: update MIS bookkeeping for next possible light hit
        if (mat.type == COR::MaterialType::Lambert) {
            prevSpecular = false;
            prevP = p;
            prevWi = scattered.d;
            prevPdfBsdf = COR::lambertPdf(rec.n, scattered.d);
        }
        else {
            // English comment: delta/specular => no MIS on light hit
            prevSpecular = true;
            prevPdfBsdf = 1.0f;
            prevP = p;
            prevWi = scattered.d;
        }

        beta = beta * attenuation;
        r = scattered;

        // ---- Russian roulette ----
        if (depth >= 5) {
            float pRR = COR::clamp(std::fmax(beta.x, std::fmax(beta.y, beta.z)), 0.05f, 0.95f);
            if (rng.nextFloat01() > pRR) break;
            beta = beta * (1.0f / pRR);
        }
    }

    return L;
}

int main() {
    const int W = 1024, H = 768, SPP = 64;

    COR::Film film(W, H);
    float aspect = float(W) / float(H);

    COR::Camera cam(
        COR::Vec3{ 0.0f, 0.0f, 0.0f },       // lookfrom
        COR::Vec3{ 0.0f, 0.0f, -1.0f },      // lookat
        COR::Vec3{ 0.0f, 1.0f, 0.0f },       // vup
        /*vfov=*/90.0f,
        aspect);

    COR::World world;

    // Materials
    COR::Material left;
    left.type = COR::MaterialType::Lambert;
    left.albedo = COR::Vec3{ 0.75f, 0.25f, 0.25f };
    world.materials.push_back(left);     //id = 0

    COR::Material right;
    right.type = COR::MaterialType::Lambert;
    right.albedo = COR::Vec3{ 0.25f, 0.25f, 0.75f };
    world.materials.push_back(right);     //id = 1

    COR::Material back;
    back.type = COR::MaterialType::Lambert;
    back.albedo = COR::Vec3{ 0.75f, 0.75f, 0.75f };
    world.materials.push_back(back);     //id = 2

    COR::Material bottom;
    bottom.type = COR::MaterialType::Lambert;
    bottom.albedo = COR::Vec3{ 0.75f, 0.75f, 0.75f };
    world.materials.push_back(bottom);     //id = 3

    COR::Material top;
    top.type = COR::MaterialType::Lambert;
    top.albedo = COR::Vec3{ 0.75f, 0.75f, 0.75f };
    world.materials.push_back(top);     //id = 4

    COR::Material light;
    light.type = COR::MaterialType::Emissive;
    light.emission = COR::Vec3{ 12.0f };
    world.materials.push_back(light);     //id = 5

    COR::Material glass;
    glass.type = COR::MaterialType::Dielectric;
    glass.albedo = COR::Vec3{ 1.f, 1.f, 1.f } * 0.999f;
    world.materials.push_back(glass);     //id = 6

    COR::Material metal;
    metal.type = COR::MaterialType::Metal;
    metal.albedo = COR::Vec3{ 1.f, 1.f, 1.f } *0.999f;
    world.materials.push_back(metal);     //id = 7

    const float x0 = -1.0f, x1 = 1.0f;
    const float y0 = -1.0f, y1 = 1.0f;
    const float z0 = -3.0f, z1 = -1.0f;

    world.add<COR::Quad>(
        COR::Vec3{ x0, y0, z1 },                                            // left
        COR::Vec3{ 0, 0, z0 - z1 },
        COR::Vec3{ 0, y1 - y0, 0 },
        0);
    world.add<COR::Quad>(                                                   // right
        COR::Vec3{ x1, y0, z0 },
        COR::Vec3{ 0, 0, z1 - z0 },
        COR::Vec3{ 0, y1 - y0, 0 },
        1);
    world.add<COR::Quad>(                                                   // back
        COR::Vec3{ x0, y0, z0 },
        COR::Vec3{ x1 - x0, 0, 0 },
        COR::Vec3{ 0, y1 - y0, 0 },
        2);       
    world.add<COR::Quad>(                                                   // floor
        COR::Vec3{ x0, y0, z1 },
        COR::Vec3{ x1 - x0, 0, 0 },
        COR::Vec3{ 0, 0, z0 - z1 },
        3);
    world.add<COR::Quad>(                                                   // ceiling
        COR::Vec3{ x0, y1, z0 },
        COR::Vec3{ x1 - x0, 0, 0 },
        COR::Vec3{ 0, 0, z1 - z0 },
        4);
    world.add<COR::Quad>(                                                 // light
        COR::Vec3{ -0.25f, 0.99f, -2.25f },
        COR::Vec3{ 0.5f, 0, 0 },
        COR::Vec3{ 0, 0, 0.5f },
        5);
    world.add<COR::Sphere>(COR::Vec3{ -0.4f, -0.6f, -2.2f }, 0.4f, 6);      // glass
    world.add<COR::Sphere>(COR::Vec3{ 0.5f, -0.6f, -1.8f }, 0.4f, 7);       // metal

    world.buildLightList();

    const int maxDepth = 40;

    RenderProgress prog(W, H, SPP);

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            for (int s = 0; s < SPP; ++s) {
                uint64_t seed = COR::makeSeed2((uint32_t)x, (uint32_t)y, (uint32_t)s);
                COR::RNG rng(seed, 1);

                float u = (float(x) + rng.nextFloat01()) / float(W - 1);
                float v = (float(y) + rng.nextFloat01()) / float(H - 1);

                COR::Ray r = cam.getRay(u, v);
                COR::Vec3 c = trace_path_mis(r, world, rng, maxDepth);
                film.addSample(x, y, c);

                prog.addSamples(1);
            }
        }
    }

    prog.done();

    const bool ok = COR::writePPM("cornell_emmitter_test.ppm", film, /*flipY=*/true, /*gamma=*/2.2f);
    if (!ok) {
        std::cerr << "Failed to write out.ppm\n";
        return 1;
    }

    std::cout << "Wrote out.ppm\n";
    return 0;
}
