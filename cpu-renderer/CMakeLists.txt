add_library(or_cpu STATIC)

target_sources(or_cpu
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

target_include_directories(or_cpu
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(or_cpu
  PUBLIC
    or_common
    or_build_options
)

target_compile_definitions(or_cpu PUBLIC OR_BACKEND_CPU=1)

add_subdirectory(apps)
