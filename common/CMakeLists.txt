# common/CMakeLists.txt
add_library(common STATIC)

# common 폴더 구조가 아래 둘 중 무엇이든 동작하도록 처리:
# 1) common/include + common/src
# 2) common/ 아래 모듈별 폴더(core/scene/...)에 .h/.cpp가 섞여 있음

# 헤더 include 경로
target_include_directories(common
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 소스 자동 수집(초기 리팩터링 단계에 편함)
file(GLOB_RECURSE COR_COMMON_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cxx"
)

target_sources(common PRIVATE ${COR_COMMON_SOURCES})