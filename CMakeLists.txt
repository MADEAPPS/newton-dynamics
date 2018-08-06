# Copyright (c) <2014-2017> <Newton Game Dynamics>
#
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

cmake_minimum_required(VERSION 3.12.0)

option("GENERATE_DLL" "build dll libraries" ON)
option("BUILD_SANDBOX_DEMOS" "generates demos projects" ON)
option("DOUBLE_PRECISION" "Generate double precision" OFF)
option("STATIC_RUNTIME_LIBRARIES" "use windows static libraries" ON)
option("WITH_AVX_PLUGIN" "adding avx parallel solver" ON)
option("WITH_AVX2_PLUGIN" "adding avx parallel solver" OFF)
option("WITH_SSE4_PLUGIN" "adding avx parallel solver" OFF)

set(CMAKE_CONFIGURATION_TYPES Debug Release)
set(CMAKE_DEBUG_POSTFIX "_d")

project(NewtonSDK)

add_definitions(-D_CRT_SECURE_NO_WARNINGS)
if (DOUBLE_PRECISION)
	add_definitions(-D_NEWTON_USE_DOUBLE)
endif ()

if (MSVC)
	message ("build visual studio projects")

	if(CMAKE_CL_64)
		message ("build 64 bit")
		add_definitions(-D_WIN_64_VER)
	else()
		message ("build 32 bit")
		add_definitions(-D_WIN_32_VER)
	endif()

	if(STATIC_RUNTIME_LIBRARIES)
		message ("link to static runtime libraries")
		set(CMAKE_CXX_FLAGS_DEBUG "/MTd")
		set(CMAKE_CXX_FLAGS_RELEASE "/MT")
		set(CMAKE_C_FLAGS_DEBUG "/MTd")
		set(CMAKE_C_FLAGS_RELEASE "/MT")
	else()
		message ("link to dynamic runtime libraries")
		set(CMAKE_C_FLAGS_DEBUG "/MDd")
		set(CMAKE_C_FLAGS_RELEASE "/MD")
		set(CMAKE_CXX_FLAGS_DEBUG "/MDd")
		set(CMAKE_CXX_FLAGS_RELEASE "/MD")
	endif()

	set(CMAKE_C_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Od /fp:fast")
	set(CMAKE_C_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox /Oi /Ot /Ob2 /fp:fast")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Od /fp:fast")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox /Oi /Ot /Ob2 /fp:fast")

	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -W4")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -W4")

	set(CMAKE_C_FLAGS_DEBUG  "${CMAKE_C_FLAGS_DEBUG} -Zi")
	set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Zi")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Zi")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Zi")

	message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
	message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")
endif()

set (dllPath "${PROJECT_BINARY_DIR}/build/bin")
set (libraryPath "${PROJECT_BINARY_DIR}/build/lib")
set (includePath "${PROJECT_BINARY_DIR}/build/include")


add_subdirectory(sdk)
if (BUILD_SANDBOX_DEMOS)
	add_subdirectory(applications/demosSandbox)
endif()

add_dependencies (dgPhysics dgCore)
add_dependencies (dContainers dMath dTimeTracker)
add_dependencies (newton dgCore dgPhysics dTimeTracker)
add_dependencies (dAnimation dMath dContainers dTimeTracker)
add_dependencies (dCustomJoints dMath dContainers dTimeTracker)
add_dependencies (dNewton newton dMath dContainers dCustomJoints dTimeTracker)
if (BUILD_SANDBOX_DEMOS)
	add_dependencies (demosSandbox newton dMath dScene dNewton dContainers dCustomJoints dTimeTracker tinyxml imgui glfw)
endif()

