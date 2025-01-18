cmake_minimum_required(VERSION 3.30)
project(raytracer)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)

set(SDL2_PATH "C:/Users/treyb/Desktop/CLibraries/SDL2-2.30.11/x86_64-w64-mingw32")

#find_package(SDL2 REQUIRED)
#include_directories(${SDL2_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/include)

set(SOURCE_FILES main.cpp
        include/vec3.h
        include/color.h
        include/ray.h)
add_executable(raytracer ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${SDL2_LIBRARY})




