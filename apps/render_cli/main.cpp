// apps/render_cli/main.cpp
#include "film/film.h"
#include "io/write_image.h"
#include "core/vector.h"
#include "core/rng.h"
#include "core/ray.h"
#include "scene/camera.h"
#include "scene/hit.h"
#include "scene/sphere.h"
#include "scene/world.h"
#include "core/random.h"

#include <iostream>

static COR::Vec3 ray_color(const COR::Ray& r, const COR::Sphere& sph) {
    COR::HitRecord rec;
    if (sph.intersect(r, 0.001f, 1e30f, rec)) {
        return (rec.n + COR::Vec3{ 1,1,1 }) * 0.5f;
    }

    // sky gradient
    COR::Vec3 unit_dir = COR::normalize(r.d);
    float t = 0.5f * (unit_dir.y + 1.0f);
    COR::Vec3 white{ 1.0f, 1.0f, 1.0f };
    COR::Vec3 sky{ 0.5f, 0.7f, 1.0f };
    return white * (1.0f - t) + sky * t;
}

static COR::Vec3 ray_color(const COR::Ray& r, const COR::World& world, COR::RNG& rng, int depth) {
    if (depth <= 0) return COR::Vec3{ 0,0,0 };

    COR::HitRecord rec;
    if (world.intersect(r, 0.001f, 1e30f, rec)) {
        // Lambert scatter
        COR::Vec3 scatter_dir = rec.n + COR::randomUnitVector(rng);
        if (COR::nearZero(scatter_dir)) scatter_dir = rec.n;

        COR::Ray scattered{ rec.p, scatter_dir };
        // 여기서 "히트된 오브젝트의 albedo"가 필요함:
        // 현재 HitRecord에 material 정보가 없으니, 임시로 rec에 albedo를 넣거나,
        // sphere hit 시 rec에 albedo를 복사하는 방식이 필요.
        //
        // 가장 쉬운 방법: HitRecord에 albedo 추가:
        // Vec3 albedo;
        return rec.albedo * ray_color(scattered, world, rng, depth - 1);
    }

    // sky gradient
    COR::Vec3 unit_dir = COR::normalize(r.d);
    float t = 0.5f * (unit_dir.y + 1.0f);
    return COR::Vec3{ 1,1,1 } *(1.0f - t) + COR::Vec3{ 0.5f,0.7f,1.0f } *t;
}

int main() {
    const int W = 512, H = 512, SPP = 32;

    COR::Film film(W, H);

    float aspect = float(W) / float(H);

    COR::Vec3 lookfrom  { 0.0f, 0.0f, 0.0f };
    COR::Vec3 lookat    { 0.0f, 0.0f, -1.0f };
    COR::Vec3 vup       { 0.0f, 1.0f, 0.0f };
    COR::Camera cam(lookfrom, lookat, vup, /*vfov=*/90.0f, aspect);

    COR::World world;
    world.spheres.emplace_back(COR::Vec3{ 0, 0, -1 }, 0.5f, COR::Vec3{ 0.7f, 0.3f, 0.3f }); // 빨강
    world.spheres.emplace_back(COR::Vec3{ 0, -100.5f, -1 }, 100.0f, COR::Vec3{ 0.8f, 0.8f, 0.0f }); // 바닥

    const int maxDepth = 20;

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            // RNG based Pixel/Sample
            uint64_t seed = COR::makeSeed((uint32_t)x, (uint32_t)y, 0);
            COR::RNG rng(seed, 1);

            for (int s = 0; s < SPP; ++s) {
                float u = (float(x) + rng.nextFloat01()) / float(W - 1);
                float v = (float(y) + rng.nextFloat01()) / float(H - 1);

                COR::Ray r = cam.getRay(u, v);
                COR::Vec3 c = ray_color(r, world, rng, maxDepth);
                film.addSample(x, y, c);
            }
        }
    }

    const bool ok = COR::writePPM("world.ppm", film, /*flipY=*/true, /*gamma=*/2.2f);
    if (!ok) {
        std::cerr << "Failed to write out.ppm\n";
        return 1;
    }

    std::cout << "Wrote out.ppm\n";
    return 0;
}
