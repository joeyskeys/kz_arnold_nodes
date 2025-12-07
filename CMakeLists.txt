cmake_minimum_required(VERSION 3.25)

project(kz_arnold_nodes LANGUAGES CXX)

cmake_policy(SET CMP0074 NEW)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "set build type")
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)

list(APPEND CMAKE_MODULE_PATH
    "${CMAKE_SOURCE_DIR}/cmake")

find_package(Arnold REQUIRED)
include_directories(${Arnold_INCLUDE_DIRS})

add_subdirectory(arnold)