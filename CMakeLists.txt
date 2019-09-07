# Copyright (c) <2014-2017> <Newton Game Dynamics>
#
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

cmake_minimum_required(VERSION 3.4.0)

option("NEWTON_BUILD_SANDBOX_DEMOS" "generates demos projects" ON)
option("NEWTON_BUILD_PROFILER" "build profiler" OFF)
option("NEWTON_BUILD_SINGLE_THREADED" "multi threaded" OFF)
option("NEWTON_DOUBLE_PRECISION" "generate double precision" OFF)
option("NEWTON_STATIC_RUNTIME_LIBRARIES" "use windows static libraries" ON)
#option("NEWTON_WITH_SSE_PLUGIN" "adding sse parallel solver" OFF)
option("NEWTON_WITH_SSE4_PLUGIN" "adding sse4 parallel solver (forces shared libs)" OFF)
option("NEWTON_WITH_AVX_PLUGIN" "adding avx parallel solver (forces shared libs)" ON)
option("NEWTON_WITH_AVX2_PLUGIN" "adding avx2 parallel solver (forces shared libs)" OFF)
#option("NEWTON_WITH_DX12_PLUGIN" "adding direct compute 12 parallel solver" OFF)
option("NEWTON_BUILD_SHARED_LIBS" "build shared library" OFF)
option("NEWTON_BUILD_CORE_ONLY" "build the core newton library only" OFF)

set(CMAKE_CONFIGURATION_TYPES Debug RelWithDebInfo Release)
set(CMAKE_DEBUG_POSTFIX "_d")

project(NewtonSDK)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Don't use e.g. GNU extension (like -std=gnu++11) for portability
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions(-D_CRT_SECURE_NO_WARNINGS)
if (NEWTON_DOUBLE_PRECISION)
	add_definitions(-D_NEWTON_USE_DOUBLE)
endif ()

if (NEWTON_BUILD_SINGLE_THREADED)
	add_definitions(-DDG_USE_THREAD_EMULATION)
endif ()

#If no build type set, Release as default
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
          "Choose the type of build, options are: Debug RelWithDebInfo Release"
          FORCE)
endif(NOT CMAKE_BUILD_TYPE)

# determine if we are compiling for a 32bit or 64bit system
include(CheckTypeSize)
CHECK_TYPE_SIZE("void*" PTR_SIZE BUILTIN_TYPES_ONLY)
if (PTR_SIZE EQUAL 8)
  set(BUILD_64 TRUE)
else ()
  set(BUILD_64 FALSE)
endif ()

#check for MAC
if(APPLE)
  add_definitions(-D_MACOSX_VER)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fms-extensions")
endif(APPLE)

#check for UNIX
if (UNIX)
    if (BUILD_64)
      add_definitions(-D_POSIX_VER_64)
    else (BUILD_64)
      add_definitions(-D_POSIX_VER)
    endif (BUILD_64)
endif(UNIX)

if (UNIX)
    include("GNUInstallDirs")

    # uninstall target
    if(NOT TARGET uninstall)
        configure_file(
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
            "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
            IMMEDIATE @ONLY)

        add_custom_target(uninstall
            COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
    endif()

    add_compile_options(-msse3)

    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

    set(NEWTON_GENERATE_DLL OFF CACHE BOOL "" FORCE)
    set(NEWTON_STATIC_RUNTIME_LIBRARIES OFF CACHE BOOL "" FORCE)
    set(NEWTON_BUILD_PROFILER OFF CACHE BOOL "" FORCE)

elseif (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "build" CACHE PATH "" FORCE)
endif ()

if (MSVC)
	message ("build visual studio projects")

	if(NEWTON_STATIC_RUNTIME_LIBRARIES)
		message ("link to static runtime libraries")
		set(CMAKE_C_FLAGS_DEBUG "/MTd")
                set(CMAKE_C_FLAGS_RELWITHDEBINFO "/MT")
		set(CMAKE_C_FLAGS_RELEASE "/MT")
		set(CMAKE_CXX_FLAGS_DEBUG "/MTd")
		set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "/MT")
		set(CMAKE_CXX_FLAGS_RELEASE "/MT")
	else()
		message ("link to dynamic runtime libraries")
		set(CMAKE_C_FLAGS_DEBUG "/MDd")
		set(CMAKE_C_FLAGS_RELWITHDEBINFO "/MD")
		set(CMAKE_C_FLAGS_RELEASE "/MD")
		set(CMAKE_CXX_FLAGS_DEBUG "/MDd")
		set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "/MD")
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
	set(CMAKE_CXX_FLAGS_RELWITHDEBINFO ${CMAKE_CXX_FLAGS_RELEASE})
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi /GS- /W4 /Ox /Oi /Ot /Ob2 /fp:fast")

	message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
	message("CMAKE_CXX_FLAGS_RELWITHDEBINFO is ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
	message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")
endif()

if(NEWTON_BUILD_SANDBOX_DEMOS)
  set(NEWTON_BUILD_CORE_ONLY OFF)
endif()

if(NEWTON_WITH_SSE4_PLUGIN OR NEWTON_WITH_AVX_PLUGIN OR NEWTON_WITH_AVX2_PLUGIN)
  # If building any of the plugins, then switch to shared libraries.
  set(NEWTON_BUILD_SHARED_LIBS ON)
endif()

if(NEWTON_BUILD_PROFILER)
	#add_definitions(-D_DG_USE_PROFILER)
	add_definitions(-DD_PROFILER)
endif()

add_subdirectory(sdk)

if (NEWTON_BUILD_SANDBOX_DEMOS)
	add_subdirectory(applications/demosSandbox)
	
     if(UNIX)
        find_package(glfw3 REQUIRED)

        if(NOT_glfw3_FOUND)
            message(WARNING "GLFW headers were not found. Disable NEWTON_BUILD_SANDBOX_DEMOS or install GLFW")
        else(NOT_glfw3_FOUND)
            message("Found GLFW library")
        endif(NOT_glfw3_FOUND)
        
        find_package(OpenAL REQUIRED)

        if(NOT_OpenAL_FOUND)
            message(WARNING "OpenAL headers were not found. Disable NEWTON_BUILD_SANDBOX_DEMOS or install OpenAL")
        else(NOT_OpenAL_FOUND)
            message("Found OpenAL library")
        endif(NOT_OpenAL_FOUND)
        
        find_package(OpenGL REQUIRED)
        
        if(NOT_OpenGL_FOUND)
            message(WARNING "OpenGL headers were not found. Disable NEWTON_BUILD_SANDBOX_DEMOS or install OpenGL")
        else(NOT_OpenGL_FOUND)
            message("Found OpenGL library")
        endif(NOT_OpenGL_FOUND)
        
      endif(UNIX)
endif()

