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

option("STATIC_RUNTIME_LIBRARIES" "use windows static libraries" ON)
option("DOUBLE_PRECISION" "Generate double precision" OFF)
option("GENERATE_DLL" "build dll libraries" ON)


set(CMAKE_CONFIGURATION_TYPES Debug Release)
set(CMAKE_DEBUG_POSTFIX "_d")
set(CMAKE_CXX_FLAGS "-W4" ) 
set(CMAKE_CXX_FLAGS_DEBUG "-W4")
set(CMAKE_CXX_FLAGS_RELEASE "-W4")

project(NewtonSDK)




add_definitions(-D_CRT_SECURE_NO_WARNINGS)
if (DOUBLE_PRECISION)
	add_definitions(-D_NEWTON_USE_DOUBLE)
endif(DOUBLE_PRECISION)

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
	else()
		message ("link to dynamic runtime libraries")
		set(CMAKE_CXX_FLAGS_DEBUG "/MDd")
		set(CMAKE_CXX_FLAGS_RELEASE "/MD")
	endif()

	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Od /fp:fast")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox /Oi /Ot /Ob2 /fp:fast")

	message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
	message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")
endif()

add_subdirectory(sdk)

add_dependencies (dgPhysics dgCore)
add_dependencies (newton dgCore dgPhysics dTimeTracker)