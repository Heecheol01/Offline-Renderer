add_library(or_common STATIC)

target_sources(or_common
  PRIVATE
    src/util/render_progress.cpp
    src/util/tinyobjloader_impl.cpp
)

target_include_directories(or_common
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(or_common
  PUBLIC
    or_build_options
)
