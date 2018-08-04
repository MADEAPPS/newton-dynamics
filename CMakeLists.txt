# Copyright (c) <2014-2017> <Newton Game Dynamics>
#
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

cmake_minimum_required(VERSION 2.8.8)

option("CONFIG_STATIC_SYSTEM_LIBRARIES" "use windows static libraries" ON)
option("DOUBLE_PRECISION" "Use Double Precision" OFF)

set(CMAKE_DEBUG_POSTFIX "_d")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -W4")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -W4")

add_definitions(-D_CRT_SECURE_NO_WARNINGS)
if (DOUBLE_PRECISION)
	add_definitions(-D_NEWTON_USE_DOUBLE)
endif(DOUBLE_PRECISION)

project(NewtonSDK)

add_subdirectory(sdk/dgCore)