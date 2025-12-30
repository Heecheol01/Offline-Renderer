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
#include "util/render_progress.h"
#include "integrator/path_trace_mis.h"

#include <iostream>

int main() {
    const int W = 1024, H = 768, SPP = 32;

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

    const int maxDepth = 100;

    COR::RenderProgress prog(W, H, SPP);

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            for (int s = 0; s < SPP; ++s) {
                uint64_t seed = COR::makeSeed2((uint32_t)x, (uint32_t)y, (uint32_t)s);
                COR::RNG rng(seed, 1);

                float u = (float(x) + rng.nextFloat01()) / float(W - 1);
                float v = (float(y) + rng.nextFloat01()) / float(H - 1);

                COR::Ray r = cam.getRay(u, v);
                COR::Vec3 c = COR::trace_path_mis(r, world, rng, maxDepth);
                film.addSample(x, y, c);

                prog.addSamples(1);
            }
        }
    }

    prog.done();

    const bool ok = COR::writePPM("test.ppm", film, /*flipY=*/true, /*gamma=*/2.2f);
    if (!ok) {
        std::cerr << "Failed to write out.ppm\n";
        return 1;
    }

    std::cout << "Wrote out.ppm\n";
    return 0;
}
