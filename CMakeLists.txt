cmake_minimum_required(VERSION 3.16)
cmake_policy(VERSION "3.16")

project(similarity-transformation CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 CONFIG REQUIRED)
find_package(GTest CONFIG REQUIRED)
find_package(manif CONFIG REQUIRED)

enable_testing()
add_subdirectory(tests)
