cmake_minimum_required(VERSION 3.22)
project(OfflineRenderer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# (선택) VS에서 폴더 정리
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

add_subdirectory(common)
add_subdirectory(cpu-renderer)