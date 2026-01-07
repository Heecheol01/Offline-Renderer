# CUDA를 쓸 경우
project(OfflineRendererGPU LANGUAGES CXX CUDA)

add_library(or_gpu STATIC)

target_sources(or_gpu
  PRIVATE
  src/accel/bvh.cpp
  src/film/film.cpp
  src/integrator/path_trace_mis.cpp
  src/integrator/wavefront.cpp
  src/io/write_image.cpp
  src/io/obj_loader.cpp
  src/scene/camera.cpp
  src/scene/material.cpp
  src/scene/mesh.cpp
  src/scene/triangle_ref.cpp
  src/util/render_progress.cpp
  src/util/tinyobjloader_impl.cpp
)

target_include_directories(or_gpu
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(or_gpu
  PUBLIC
    or_common
    or_build_options
)

target_compile_definitions(or_gpu PUBLIC OR_BACKEND_GPU=1)

# CUDA 세팅(필요시)
set_target_properties(or_gpu PROPERTIES
  CUDA_STANDARD 17
  CUDA_STANDARD_REQUIRED ON
)

# 필요하면 find_package(CUDAToolkit REQUIRED) 후 링크 추가
# find_package(CUDAToolkit REQUIRED)
# target_link_libraries(or_gpu PUBLIC CUDA::cudart)

add_subdirectory(apps)
