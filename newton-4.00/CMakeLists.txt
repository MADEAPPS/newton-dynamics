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

option("NEWTON_BUILD_TEST" "generate test project" "ON")
option("NEWTON_BUILD_SANDBOX_DEMOS" "generates demos projects" "OFF")
option("NEWTON_BUILD_PROFILER" "build profiler" OFF)
option("NEWTON_ENABLE_AVX" "enable AVX float instructions" ON)
option("NEWTON_BUILD_SINGLE_THREADED" "multi threaded" OFF)
option("NEWTON_DOUBLE_PRECISION" "generate double precision" OFF)
option("NEWTON_STATIC_RUNTIME_LIBRARIES" "use windows static libraries" ON)
option("NEWTON_USE_PLUGIN_DLL" "use plugins dlls" OFF)
#option("NEWTON_WITH_SSE_PLUGIN" "adding sse parallel solver" OFF)
option("NEWTON_WITH_SSE4_PLUGIN" "adding sse4 parallel solver (forces shared libs)" OFF)
option("NEWTON_WITH_AVX_PLUGIN" "adding avx parallel solver (forces shared libs)" OFF)
option("NEWTON_WITH_AVX2_PLUGIN" "adding avx2 parallel solver (forces shared libs)" OFF)
option("NEWTON_WITH_REFERENCE_GPU_PLUGIN" "adding gpu reference solver, gpu solve using avx2" OFF)
option("NEWTON_BUILD_SHARED_LIBS" "build shared library" OFF)
option("NEWTON_BUILD_CORE_ONLY" "build the core newton library only" OFF)

set(CMAKE_CONFIGURATION_TYPES Debug RelWithDebInfo Release)
set(CMAKE_DEBUG_POSTFIX "_d")

project(NewtonSDK)

# determine if we are compiling for a 32bit or 64bit system
include(CheckTypeSize)
CHECK_TYPE_SIZE("void*" PTR_SIZE BUILTIN_TYPES_ONLY)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Don't use e.g. GNU extension (like -std=gnu++11) for portability
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

if(NEWTON_DOUBLE_PRECISION)
	add_definitions(-DD_NEWTON_USE_DOUBLE)
endif()

if(NEWTON_ENABLE_AVX)
	add_definitions(-DD_USE_VECTOR_AVX)
endif()


if(NEWTON_BUILD_SINGLE_THREADED)
	add_definitions(-DD_USE_THREAD_EMULATION)
endif()

if(NEWTON_BUILD_SHARED_LIBS)
    add_definitions(-D_D_CORE_DLL)
	add_definitions(-D_D_NEWTON_DLL)
endif(NEWTON_BUILD_SHARED_LIBS)

#If no build type set, Release as default
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
          "Choose the type of build, options are: Debug RelWithDebInfo Release"
          FORCE)
endif(NOT CMAKE_BUILD_TYPE)

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
if(UNIX)
	if(NOT NEWTON_BUILD_SHARED_LIBS)
		add_compile_options(-fPIC)
	endif()
	if (BUILD_64)
		add_definitions(-D_POSIX_VER_64)
	else (BUILD_64)
		add_definitions(-D_POSIX_VER)
	endif (BUILD_64)

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

	if(NOT ANDROID)
		add_compile_options(-msse3)
	endif()
	add_compile_options(-fpermissive)

	set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
endif ()

if (MSVC)
	if (PTR_SIZE EQUAL 8)
		set(CMAKE_INSTALL_PREFIX "win64sdk" CACHE PATH "..." FORCE)
	else()
		set(CMAKE_INSTALL_PREFIX "win32sdk" CACHE PATH "..." FORCE)
	endif ()

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

	if(WIN32) 
		if(CMAKE_CL_64)
			message ("build 64 bit")
			add_definitions(-D_WIN_64_VER)
		else()
			message ("build 32 bit")
			add_definitions(-D_WIN_32_VER)
			set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /arch:SSE2")
		endif()
	else()
		message ("build for arm configuration")
		add_definitions(-D_ARM_VER)

		set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /EHsc")
		set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /EHsc")
		set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /EHsc")
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /EHsc")
	endif()

	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Zi /W4 /Od")
	set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} /Zi /W4 /O2")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi /GS- /W4 /Ox /Oi /Ot /Ob2 /Oy /fp:fast")

	if (NEWTON_ENABLE_AVX)
		set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /arch:AVX")
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /arch:AVX")
		set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /arch:AVX")
	endif()

	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /wd4100 /wd4201 /wd4324 /wd4714")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /wd4100 /wd4201 /wd4324 /wd4714")
	set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /wd4100 /wd4201 /wd4324 /wd4714")

	message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
	message("CMAKE_CXX_FLAGS_RELWITHDEBINFO is ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
	message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")

elseif(MINGW)
	add_compile_options(-msse3)
	if(CMAKE_CL_64)
		message("build 64 bit")
		add_definitions(-D_MINGW_64_VER)
		add_definitions(-D_WIN_64_VER)
	else()
		message("build 32 bit")
		add_definitions(-D_MINGW_32_VER)
		add_definitions(-D_WIN_32_VER)
	endif()
endif()

if(NEWTON_BUILD_SANDBOX_DEMOS STREQUAL "ON")
  message("NEWTON_BUILD_SANDBOX_DEMOS is ${NEWTON_BUILD_SANDBOX_DEMOS}")
  message("NEWTON_BUILD_CORE_ONLY is ${NEWTON_BUILD_CORE_ONLY}")
  #message("setting core only OFF")
  #set(NEWTON_BUILD_CORE_ONLY OFF)
endif()

#if(NEWTON_WITH_SSE4_PLUGIN OR NEWTON_WITH_AVX_PLUGIN OR NEWTON_WITH_AVX2_PLUGIN)
# If building any of the plugins, then switch to shared libraries.
# set(NEWTON_BUILD_SHARED_LIBS ON)
#endif()

if(NEWTON_BUILD_PROFILER)
	#add_definitions(-D_DG_USE_PROFILER)
	add_definitions(-DD_PROFILER)
endif()

add_subdirectory(sdk)
add_subdirectory(applications)



