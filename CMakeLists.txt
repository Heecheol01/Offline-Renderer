cmake_minimum_required(VERSION 3.20)
project(OfflineRenderer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_CPU "Build CPU renderer" ON)
option(BUILD_GPU "Build GPU renderer" ON)

# (선택) 빌드 산출물 폴더 정리
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# (권장) 공통 컴파일 옵션/정의는 인터페이스 타깃으로 모아두면 편함
add_library(or_build_options INTERFACE)
target_compile_features(or_build_options INTERFACE cxx_std_17)

# MSVC 예시(선택)
if (MSVC)
  target_compile_options(or_build_options INTERFACE /W4)
else()
  target_compile_options(or_build_options INTERFACE -Wall -Wextra)
endif()

# 공통 모듈
add_subdirectory(common)

if (BUILD_CPU)
  add_subdirectory(cpu-renderer)
endif()

if (BUILD_GPU)
  add_subdirectory(gpu-renderer)
endif()
