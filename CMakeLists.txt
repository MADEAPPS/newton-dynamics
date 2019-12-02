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


option("NEWTON_WIN32CC" "Cross compile to 32 bit Windows" OFF)
option("NEWTON_ARM32" "Cross compile to 32 bit Armv7-A" OFF)
option("NEWTON_ARM64" "Cross compile to 64 bit Armv8-A" OFF)
option("NEWTON_BUILD_SANDBOX_DEMOS" "generates demos projects" "OFF")
option("NEWTON_BUILD_PROFILER" "build profiler" OFF)
option("NEWTON_BUILD_SINGLE_THREADED" "multi threaded" OFF)
option("NEWTON_DOUBLE_PRECISION" "generate double precision" OFF)
option("NEWTON_STATIC_RUNTIME_LIBRARIES" "use windows static libraries" ON)
#option("NEWTON_WITH_SSE_PLUGIN" "adding sse parallel solver" OFF)
option("NEWTON_WITH_SSE4_PLUGIN" "adding sse4 parallel solver (forces shared libs)" OFF)
option("NEWTON_WITH_AVX_PLUGIN" "adding avx parallel solver (forces shared libs)" ON)
option("NEWTON_WITH_AVX2_PLUGIN" "adding avx2 parallel solver (forces shared libs)" OFF)
#option("NEWTON_WITH_DX12_PLUGIN" "adding direct compute 12 parallel solver" OFF)
option("NEWTON_BUILD_SHARED_LIBS" "build shared library" ON)
option("NEWTON_BUILD_CORE_ONLY" "build the core newton library only" ON)

set(CMAKE_CONFIGURATION_TYPES Debug RelWithDebInfo Release)
set(CMAKE_DEBUG_POSTFIX "_d")

project(NewtonSDK)

message("NEWTON_WIN32CC is ${NEWTON_WIN32CC}")
message("NEWTON_ARM32 is ${NEWTON_ARM32}")
message("NEWTON_ARM64 is ${NEWTON_ARM64}")

# determine if we are compiling for a 32bit or 64bit system
include(CheckTypeSize)
CHECK_TYPE_SIZE("void*" PTR_SIZE BUILTIN_TYPES_ONLY)

if (NEWTON_WIN32CC STREQUAL "ON")
	
	message("Cross compiling to 32 bit windows")
	
	# todo:
	# SET(CMAKE_SYSTEM_PROCESSOR arm)
	
	# specify the cross compiler
	set(CMAKE_CROSSCOMPILING "ON")
	
	# 32 bit windows
	SET(CMAKE_C_COMPILER i686-w64-mingw32-g++)
	SET(CMAKE_CXX_COMPILER i686-w64-mingw32-g++)
	SET(CMAKE_ASM_COMPILER i686-w64-mingw32-g++)
	
	SET(PTR_SIZE 4)
	
	set(NEWTON_ARM32 OFF FORCE)
	set(NEWTON_ARM64 OFF FORCE)
	
	# needed to convince code that we are building win32
	add_definitions(-D_WIN_32_VER)
	add_definitions(-D_MINGW_32_VER)
	
	# need this as otherwise dgtypes won't cross compile
	# DO WE REALLY NEED THIS?
	
	#add_definitions(-DDG_USE_THREAD_EMULATION)
	
	# todo: use NEWTON_STATIC_RUNTIME_LIBRARIES ?
	# link statically - no libstdc dll dependancies
	add_compile_options(-static)
	
	add_compile_options(-gdwarf-2)
	
	set(NEWTON_GENERATE_DLL "ON" FORCE)
	
	SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -static" )
	SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -static" )
	
endif ()

# cross compiling to arm forces variables to trigger arm via cross compiler:

if (NEWTON_ARM32 STREQUAL "ON")
	
	message("Cross compiling to 32 bit arm linux")
	
	# specify the cross compiler
	set(CMAKE_CROSSCOMPILING TRUE)
	
	SET(PTR_SIZE 4)
	
	# 32 bit arm # 32 bit arm / ELF 32-bit LSB shared object, ARM, EABI5 version 1 (SYSV)
	SET(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
	SET(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)
	SET(CMAKE_ASM_COMPILER arm-linux-gnueabihf-gcc)
	SET(CMAKE_SYSTEM_PROCESSOR arm)
	# this is needed for arm
	add_definitions(-DDG_SCALAR_VECTOR_CLASS)
	
endif ()

if (NEWTON_ARM64 STREQUAL "ON")
	
	message("Cross compiling to 64 bit arm linux")
	
	# specify the cross compiler
	set(CMAKE_CROSSCOMPILING TRUE)
	
	# 64 bit arm # 64-bit LSB shared object, ARM aarch64, version 1 (SYSV)
	SET(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
	SET(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
	SET(CMAKE_ASM_COMPILER aarch64-linux-gnu-gcc)
	SET(CMAKE_SYSTEM_PROCESSOR aarch64)
	# this is needed for arm
	add_definitions(-DDG_SCALAR_VECTOR_CLASS)
endif ()

message("CMAKE_SYSTEM_PROCESSOR is ${CMAKE_SYSTEM_PROCESSOR}")

# enable 32 / 64 defines so that x86-only stuff is properly disabled

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm.*|ARM.*)")
	set(NEWTON_ARM32 ON)
	# no vector class for arm32
	add_definitions(-DDG_SCALAR_VECTOR_CLASS)
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64.*|AARCH64.*)")
	set(NEWTON_ARM64 ON)
	# no vector class for arm64
	add_definitions(-DDG_SCALAR_VECTOR_CLASS)
endif()

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

if (PTR_SIZE EQUAL 8)
  set(BUILD_64 TRUE)
  message("64 bit build")
else ()
  set(BUILD_64 FALSE)
  message("32 bit build")
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

    if (NEWTON_ARM64 STREQUAL "ON")
	#message("NEWTON_ARM64 is ${NEWTON_ARM64}")
	# arm64 specific extensions?
    elseif(NEWTON_ARM32 STREQUAL "ON")
	#message("NEWTON_ARM32 is ${NEWTON_ARM32}")
	# arm32 specific extensions?
    else()
	# extensions for x86 variants
	message ("With SSE3")
    	add_compile_options(-msse3)
    endif()

    add_compile_options(-fpermissive)
    #add_compile_options(-Wignored-attributes)
    # no warnings
    #add_compile_options(-w)

    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

    if (NOT NEWTON_WIN32CC STREQUAL "ON")
	message ("DLL OFF")
	set(NEWTON_GENERATE_DLL OFF CACHE BOOL "" FORCE)
    endif()


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

if(NEWTON_BUILD_SANDBOX_DEMOS STREQUAL "ON")
  message("NEWTON_BUILD_SANDBOX_DEMOS is ${NEWTON_BUILD_SANDBOX_DEMOS}")
  message("NEWTON_BUILD_CORE_ONLY is ${NEWTON_BUILD_CORE_ONLY}")
  message("setting core only OFF")
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

if (NEWTON_BUILD_SANDBOX_DEMOS STREQUAL "ON")
	
	message("BUILDING DEMOS.")
	
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

