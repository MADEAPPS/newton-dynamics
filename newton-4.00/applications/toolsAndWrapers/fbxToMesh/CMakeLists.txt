cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR)

project("fbx2ndMesh")
set (projectName "fbx2ndMesh")

file(GLOB CPP_SOURCE 
	*.h 
	*.cpp
	openFBX/src/*.h
	openFBX/src/*.c
	openFBX/src/*.cpp
	install/source/*.h
	install/source/*.c
	install/source/*.cpp)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

include_directories(openFBX/src)
include_directories(install/source)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Zi /Od")

add_executable(${projectName} ${CPP_SOURCE})


