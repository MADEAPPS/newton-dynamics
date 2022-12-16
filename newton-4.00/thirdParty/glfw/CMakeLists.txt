# Copyright (c) <2014-2017> <Newton Game Dynamics>
#
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

cmake_minimum_required(VERSION 3.9.0 FATAL_ERROR)

set (projectName "glfw")
message (${projectName})

add_definitions(-D_GLFW_WIN32)

if (MSVC)
	include_directories(include/GLFW)

	set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /Wall /wd4018 /wd4054 /wd4055 /wd4100 /wd4152 /wd4204 /wd4244 /wd4255 /wd4389 /wd4458 /wd4505")
	set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Wall /wd4018 /wd4054 /wd4055 /wd4100 /wd4152 /wd4204 /wd4244 /wd4255 /wd4389 /wd4458 /wd4505")
	set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} /Wall /wd4018 /wd4054 /wd4055 /wd4100 /wd4204 /wd4152 /wd4244 /wd4255 /wd4389 /wd4458 /wd4505")

	set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /Od")
	set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} /Zi /O2 /fp:fast")
	set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Zi /GS- /Ox /Oi /Ot /Ob2 /Oy /fp:fast")

endif()

 # low level core
file(GLOB source src/init.c src/input.c src/monitor.c src/vulkan.c 
				 src/context.c src/egl_context.c src/egl_context.c
				 src/wgl_context.c src/win32_*.c src/window.c)

add_library(${projectName} STATIC ${source})

