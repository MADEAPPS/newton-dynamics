cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR)

project("bvh2fbx")
set (projectName "bvh2fbx")
set (libraryPath "$ENV{FBX_SDK}/Lib/vs2017/$(Platform)/$(Configuration)/")

message ("fbx lib path: ${libraryPath}")

file(GLOB CPP_SOURCE *.h *.cpp)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

add_definitions(-D_CRT_SECURE_NO_WARNINGS)
include_directories ($ENV{FBX_SDK}/include)

link_directories(${libraryPath})

set(CMAKE_C_FLAGS_DEBUG "/MTd")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "/MT")
set(CMAKE_C_FLAGS_RELEASE "/MT")
set(CMAKE_CXX_FLAGS_DEBUG "/MTd")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "/MT")
set(CMAKE_CXX_FLAGS_RELEASE "/MT")

add_executable(${projectName} ${CPP_SOURCE})

target_link_libraries (${projectName} "zlib-mt.lib")
target_link_libraries (${projectName} "libxml2-mt.lib")
target_link_libraries (${projectName} "libfbxsdk-mt.lib")
