# Copyright (c) <2014-2017> <Newton Game Dynamics>
#
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

cmake_minimum_required(VERSION 3.10.0)

option("GENERATE_DLL" "build dll libraries" ON)
option("BUILD_SANDBOX_DEMOS" "generates demos projects" ON)
option("DOUBLE_PRECISION" "Generate double precision" OFF)
option("STATIC_RUNTIME_LIBRARIES" "use windows static libraries" ON)
option("WITH_SSE_PLUGIN" "adding asse parallel solver" OFF)
option("WITH_SSE4_PLUGIN" "adding avx parallel solver" OFF)
option("WITH_AVX_PLUGIN" "adding avx parallel solver" ON)
option("WITH_AVX2_PLUGIN" "adding avx parallel solver" OFF)
option("WITH_DX12_PLUGIN" "adding direct compute 12 parallel solver" OFF)
option("NEWTON_BUILD_SHARED_LIBS" "Build shared library POSIX only option" OFF)

set(CMAKE_CONFIGURATION_TYPES Debug Release)
set(CMAKE_DEBUG_POSTFIX "_d")

project(NewtonSDK)

if (UNIX)
    # Specify build paths
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

    set(CMAKE_INSTALL_LIBDIR "lib")
    set(CMAKE_INSTALL_INCLUDEDIR "include")

    if (BUILD_64)
        add_definitions(-D_POSIX_VER_64)
    else (BUILD_64)
        add_definitions(-D_POSIX_VER)
    endif (BUILD_64)

    add_definitions(-msse4.1 -fPIC)

    set(GENERATE_DLL OFF CACHE BOOL "" FORCE)
    set(STATIC_RUNTIME_LIBRARIES OFF CACHE BOOL "" FORCE)
    set(WITH_AVX_PLUGIN OFF CACHE BOOL "" FORCE)

    if(NOT CMAKE_BUILD_TYPE)
      set(CMAKE_BUILD_TYPE "Release" CACHE STRING
          "Choose the type of build, options are: Debug Release"
          FORCE)
    endif(NOT CMAKE_BUILD_TYPE)

    find_package(glfw3 REQUIRED)

    if(NOT_glfw3_FOUND)
        set(BUILD_SANDBOX_DEMOS OFF CACHE BOOL "" FORCE)
        message(WARNING "GLFW headers were not found. Sandbox_Demos disabled")
    else(NOT_glfw3_FOUND)
        message("Found GLFW library")
    endif(NOT_glfw3_FOUND)

endif (UNIX)

add_definitions(-D_CRT_SECURE_NO_WARNINGS)
if (DOUBLE_PRECISION)
	add_definitions(-D_NEWTON_USE_DOUBLE)
endif ()

if (MSVC)
	message ("build visual studio projects")

	if(STATIC_RUNTIME_LIBRARIES)
		message ("link to static runtime libraries")
		set(CMAKE_C_FLAGS_DEBUG "/MTd")
		set(CMAKE_C_FLAGS_RELEASE "/MT")
		set(CMAKE_CXX_FLAGS_DEBUG "/MTd")
		set(CMAKE_CXX_FLAGS_RELEASE "/MT")
	else()
		message ("link to dynamic runtime libraries")
		set(CMAKE_C_FLAGS_DEBUG "/MDd")
		set(CMAKE_C_FLAGS_RELEASE "/MD")
		set(CMAKE_CXX_FLAGS_DEBUG "/MDd")
		set(CMAKE_CXX_FLAGS_RELEASE "/MD")
	endif()

	if(CMAKE_CL_64)
		message ("build 64 bit")
		add_definitions(-D_WIN_64_VER)
	else()
		message ("build 32 bit")
		add_definitions(-D_WIN_32_VER)
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /arch:SSE2")
	endif()

	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Zi /W4 /Od")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi /W4 /Ox /Oi /Ot /Ob2 /fp:fast")

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

if (MSVC)
add_dependencies (dgPhysics dgCore)
add_dependencies (dContainers dMath dTimeTracker)
add_dependencies (newton dgCore dgPhysics dTimeTracker)
add_dependencies (dAnimation dMath dContainers dTimeTracker)
add_dependencies (dCustomJoints dMath dContainers dTimeTracker)
add_dependencies (dNewton newton dMath dContainers dCustomJoints dTimeTracker)
add_dependencies (dVehicle newton dMath dContainers dCustomJoints dTimeTracker)

endif (MSVC)

if (UNIX)
    add_dependencies (dgPhysics dgCore)
    add_dependencies (dContainers dMath)
    add_dependencies (newton dgCore dgPhysics)
    add_dependencies (dAnimation dMath dContainers)
    add_dependencies (dCustomJoints dMath dContainers)
    add_dependencies (dNewton newton dMath dContainers dCustomJoints)
    add_dependencies (dVehicle newton dMath dContainers dCustomJoints)
endif (UNIX)

if (BUILD_SANDBOX_DEMOS)
    if (MSVC)
        add_dependencies (demosSandbox newton dMath dScene dNewton dContainers dVehicle dCustomJoints dTimeTracker tinyxml imgui glfw)
    endif (MSVC)

    if (UNIX)
        add_dependencies (demosSandbox newton dMath dScene dNewton dContainers dVehicle dCustomJoints tinyxml imgui)
    endif (UNIX)

endif()

