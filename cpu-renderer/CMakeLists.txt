# cpu-renderer/CMakeLists.txt

# 소스 수집 (apps/main.cpp는 별도로 주는 걸 권장)
file(GLOB_RECURSE CPU_RENDER_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cxx"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
)

add_executable(render_cpu
  apps/main.cpp
  ${CPU_RENDER_SOURCES}
)

target_link_libraries(render_cpu PRIVATE common)

target_include_directories(render_cpu
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)