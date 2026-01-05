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
#include "integrator/wavefront.h"

#include "io/obj_loader.h"
#include "scene/mesh.h"

#include <iostream>

int main() {
    const int W = 512, H = 384, SPP = 512;

    COR::Film film(W, H);
    float aspect = float(W) / float(H);

    COR::Camera cam(
        COR::Vec3{ 0.0f, 0.0f, 0.0f },       // lookfrom
        COR::Vec3{ 0.0f, 0.0f, -1.0f },      // lookat
        COR::Vec3{ 0.0f, 1.0f, 0.0f },       // vup
        /*vfov=*/90.0f,
        aspect);

    COR::World world;

    // Cornell Box Materials (0~5)
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

    // NEW: bunny / teapot materials (6,7)
    COR::Material bunnyMat;
    bunnyMat.type = COR::MaterialType::Lambert;
    bunnyMat.albedo = COR::Vec3{ 0.85f, 0.85f, 0.85f };
    world.materials.push_back(bunnyMat);  //id = 6

    COR::Material teapotMat;
    teapotMat.type = COR::MaterialType::Lambert;
    teapotMat.albedo = COR::Vec3{ 0.85f, 0.80f, 0.70f };
    world.materials.push_back(teapotMat); //id = 7

    // Cornell box geometry
    world.addShape<COR::Quad>(                  // left
        0,
        COR::Vec3{ -1.0f, -1.0f, -1.0f },
        COR::Vec3{ 0.0f, 0.0f, -2.0f },
        COR::Vec3{ 0.0f, 2.0f, 0.0f });

    world.addShape<COR::Quad>(                  // right
        1, 
        COR::Vec3{ 1.0f, -1.0f, -3.0f },
        COR::Vec3{ 0.0f, 0.0f, 2.0f },
        COR::Vec3{ 0.0f, 2.0f, 0.0f });

    world.addShape<COR::Quad>(                  // back
        2,
        COR::Vec3{ -1.0f, -1.0f, -3.0f },
        COR::Vec3{ 2.0f, 0.0f, 0.0f },
        COR::Vec3{ 0.0f, 2.0f, 0.0f });

    world.addShape<COR::Quad>(                  // floor
        3,
        COR::Vec3{ -1.0f, -1.0f, -1.0f },
        COR::Vec3{ 2.0f, 0.0f, 0.0f },
        COR::Vec3{ 0.0f, 0.0f, -2.0f });

    world.addShape<COR::Quad>(                  // ceiling
        4,
        COR::Vec3{ -1.0f, 1.0f, -3.0f },
        COR::Vec3{ 2.0f, 0.0f, 0.0f },
        COR::Vec3{ 0.0f, 0.0f, 2.0f });

    world.addShape<COR::Quad>(                  // light
        5,
        COR::Vec3{ -0.25f, 0.99f, -2.25f },
        COR::Vec3{ 0.5f, 0.0f, 0.0f },
        COR::Vec3{ 0.0f, 0.0f, 0.5f });

    {
        COR::MeshData bunny;
        COR::ObjLoadOptions opt;
        opt.triangulate = true;
        opt.scale = 3.f;
        opt.translate = COR::Vec3{ -0.4f, -1.0f, -2.2f };

        if (!COR::LoadObjMesh("C:\\KHC\\OfflineRenderer\\assets\\stanford-bunny.obj", bunny, opt)) {
            std::cerr << "Failed to load assets/bunny.obj\n";
            return 1;
        }
        world.addMeshAsTriangles(6, std::move(bunny));
    }

    {
        COR::MeshData teapot;
        COR::ObjLoadOptions opt;
        opt.triangulate = true;
        opt.scale = 0.1f;
        opt.translate = COR::Vec3{ 0.5f, -1.0f, -1.8f };

        if (!COR::LoadObjMesh("C:\\KHC\\OfflineRenderer\\assets\\teapot.obj", teapot, opt)) {
            std::cerr << "Failed to load assets/teapot.obj\n";
            return 1;
        }
        world.addMeshAsTriangles(7, std::move(teapot));
    }

    world.buildLightList();
    world.buildBVH(4);

    const int maxDepth = 100;
    COR::RenderProgress prog(W, H, SPP);
    const int tileSize = 32;

    for (int ty = 0; ty < H; ty += tileSize) {
        for (int tx = 0; tx < W; tx += tileSize) {
            int x0 = tx, y0 = ty;
            int x1 = std::min(tx + tileSize, W);
            int y1 = std::min(ty + tileSize, H);

            for (int s = 0; s < SPP; ++s) {
                COR::render_tile_wavefront_sample(
                    film, cam, world,
                    W, H,
                    x0, y0, x1, y1,
                    s, maxDepth);

                prog.addSamples((long long)(x1 - x0) * (long long)(y1 - y0));
            }
        }
    }

    prog.done();

    const bool ok = COR::writePPM("test2.ppm", film, /*flipY=*/true, /*gamma=*/2.2f);
    if (!ok) {
        std::cerr << "Failed to write out.ppm\n";
        return 1;
    }

    std::cout << "Wrote out.ppm\n";
    return 0;
}
