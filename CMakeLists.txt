cmake_minimum_required(VERSION 3.20)
project(cpu_renderer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(renderer_core
  src/core/dummy.cpp
  src/film/film.cpp
  src/integrator/path_trace_mis.cpp
  src/integrator/wavefront.cpp
  src/io/write_image.cpp
  src/io/load_obj.cpp
  src/scene/camera.cpp
  src/scene/material.cpp
  src/util/render_progress.cpp
)

target_include_directories(renderer_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(render_cli apps/render_cli/main.cpp)
target_link_libraries(render_cli PRIVATE renderer_core)

# (권장) 경고 옵션: 컴파일러별로 최소만
if (MSVC)
  target_compile_options(renderer_core PRIVATE /W4 /permissive-)
  target_compile_options(render_cli PRIVATE /W4 /permissive-)
else()
  target_compile_options(renderer_core PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(render_cli PRIVATE -Wall -Wextra -Wpedantic)
endif()
