project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++14)

set(sources
    src/tools.cpp
    src/io.cpp
    src/unscented_kalman_filter.cpp
    src/main.cpp)

add_executable(UnscentedKF ${sources})
