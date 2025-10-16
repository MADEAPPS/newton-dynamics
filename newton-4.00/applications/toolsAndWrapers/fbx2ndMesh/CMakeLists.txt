cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR)

project("fbx2ndMesh")
set (projectName "fbx2ndMesh")

#set (libraryPath "$ENV{FBX_SDK}/Lib/vs2017/$(Platform)/$(Configuration)/")
#message ("fbx lib path: ${libraryPath}")

set (libraryPath "${CMAKE_CURRENT_SOURCE_DIR}/../../../../newton-4.00-Binaries/static_mt")
message (${libraryPath})

set (libraryPath "${CMAKE_CURRENT_SOURCE_DIR}/../../../../newton-4.00-Binaries/static_mt")
include_directories(${libraryPath}/include/ndNewton)

file(GLOB CPP_SOURCE *.h *.cpp)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

#include_directories ($ENV{FBX_SDK}/include)
#link_directories(${libraryPath})

set(CMAKE_C_FLAGS_DEBUG "/MTd")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "/MT")
set(CMAKE_C_FLAGS_RELEASE "/MT")
set(CMAKE_CXX_FLAGS_DEBUG "/MTd")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "/MT")
set(CMAKE_CXX_FLAGS_RELEASE "/MT")

add_executable(${projectName} ${CPP_SOURCE})

target_link_libraries (${projectName} "${libraryPath}/lib/ndNewton_d.lib")

