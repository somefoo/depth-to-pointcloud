# FetchContent_MakeAvailable is available since CMake version 3.14
cmake_minimum_required(VERSION 3.14)

# Hunter setup:
set(HUNTER_PACKAGES OpenEXR)

include(FetchContent)
FetchContent_Declare(SetupHunter GIT_REPOSITORY https://github.com/cpp-pm/gate)
FetchContent_MakeAvailable(SetupHunter)

# Normal CMake content:

project(depth-to-pointcloud)
set (CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-Og")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(OpenEXR CONFIG REQUIRED)


add_executable(run-depth-to-pointcloud
  src/main.cpp
  )

target_link_libraries(run-depth-to-pointcloud PUBLIC OpenEXR::OpenEXR)
