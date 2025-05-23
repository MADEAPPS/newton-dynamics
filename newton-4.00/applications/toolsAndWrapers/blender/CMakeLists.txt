# Copyright (c) <2014-2017> <Newton Game Dynamics>
#
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR)

option("NEWTON_BUILD_TEST" "generate test project" "OFF")
option("NEWTON_BUILD_SANDBOX_DEMOS" "generates demos projects" "ON")
option("NEWTON_BUILD_PROFILER" "build profiler" OFF)
option("NEWTON_BUILD_CREATE_SUB_PROJECTS" "generate independent subproject" OFF)
option("NEWTON_BUILD_SHARED_LIBS" "build shared library" ON)
option("NEWTON_ENABLE_GPU_SOLVER" "enable gpu solver" OFF)
option("NEWTON_ENABLE_AVX2_SOLVER" "enable AVX2 solver" ON)
option("NEWTON_BUILD_SINGLE_THREADED" "single threaded" OFF)
option("NEWTON_DOUBLE_PRECISION" "generate double precision" OFF)
option("NEWTON_STATIC_RUNTIME_LIBRARIES" "use windows static libraries" OFF)
option("NEWTON_USE_DEFAULT_NEW_AND_DELETE" "overload new and delete when building dll" OFF)

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

if(NEWTON_ENABLE_AVX2_SOLVER)
	add_definitions(-D_D_USE_AVX2_SOLVER)
endif()

if(NEWTON_BUILD_SINGLE_THREADED)
	add_definitions(-DD_USE_THREAD_EMULATION)
endif()

if(NEWTON_BUILD_SHARED_LIBS)
    add_definitions(-D_D_CORE_DLL)
	add_definitions(-D_D_NEWTON_DLL)
	add_definitions(-D_D_COLLISION_DLL)
endif(NEWTON_BUILD_SHARED_LIBS)

#If no build type set, Release as default
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
          "Choose the type of build, options are: Debug RelWithDebInfo Release"
          FORCE)
endif(NOT CMAKE_BUILD_TYPE)

#check for MAC
if(APPLE)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fms-extensions")
endif(APPLE)

#check for UNIX
if(UNIX)
    set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)
	if(NOT NEWTON_BUILD_SHARED_LIBS)
		add_compile_options(-fPIC)
	endif()

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

	if(${CMAKE_VS_PLATFORM_NAME} STREQUAL "x64")
		message ("build 64 bit")
	elseif(${CMAKE_VS_PLATFORM_NAME} STREQUAL "Win32")
		message ("build 32 bit")
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /arch:SSE2")
	elseif(${CMAKE_VS_PLATFORM_NAME} STREQUAL "ARM" OR ${CMAKE_VS_PLATFORM_NAME} STREQUAL "ARM64")
		message ("build for arm configuration")
		set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /EHsc")
		set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /EHsc")
		set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /EHsc")
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /EHsc")
	endif()

	message("Compiler Id: ${CMAKE_CXX_COMPILER_ID}") 
	
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Zi /W4 /Od")
	set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} /Zi /W4 /O2 /fp:fast")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi /GS- /W4 /Ox /Oi /Ot /Ob2 /Oy /fp:fast")

	if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
		message("for visual studio compiler we assume sse3 at a minimum") 
		#add_definitions(-DD_USE_SSE3)
	endif()

	#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /wd4100 /wd4201 /wd4324 /wd4714")
	#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /wd4100 /wd4201 /wd4324 /wd4714")
	#set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /wd4100 /wd4201 /wd4324 /wd4714")

	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /wd4201 /wd4324")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /wd4201 /wd4324")
	set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /wd4201 /wd4324")

	message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
	message("CMAKE_CXX_FLAGS_RELWITHDEBINFO is ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
	message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")

elseif(MINGW)
	add_compile_options(-msse3)

	if(CMAKE_CL_64)
		message("build 64 bit")
	else()
		message("build 32 bit")
	endif()
endif()

if(NEWTON_BUILD_PROFILER)
	add_definitions(-DD_PROFILER)
endif()

add_subdirectory(sdk)
add_subdirectory(applications)

if (NEWTON_BUILD_SANDBOX_DEMOS)
	add_subdirectory(thirdParty)
endif()



