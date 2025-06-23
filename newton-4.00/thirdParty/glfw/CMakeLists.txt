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

set (projectName "glfw")
message (${projectName})

if (MSVC)
	add_definitions(-D_GLFW_WIN32)
elseif(MINGW)
	add_definitions(-D__CYGWIN__)
elseif(APPLE)
	add_definitions(-D_GLFW_COCOA)
else(UNIX)
	add_definitions(-D__OpenBSD__)
endif()

if (MSVC)
	include_directories(include/GLFW)

	set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /Wall /wd4018 /wd4054 /wd4055 /wd4061 /wd4100 /wd4152 /wd4191 /wd4201 /wd4204 /wd4211 /wd4244 /wd4255 /wd4389 /wd4458 /wd4505 /wd4668 /wd4706 /wd4710 /wd4711 /wd4820 /wd5045")
	set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Wall /wd4018 /wd4054 /wd4055 /wd4061 /wd4100 /wd4152 /wd4191 /wd4201 /wd4204 /wd4211 /wd4244 /wd4255 /wd4389 /wd4458 /wd4505 /wd4668 /wd4706 /wd4710 /wd4711 /wd4820 /wd5045")
	set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} /Wall /wd4018 /wd4054 /wd4061 /wd4055 /wd4100 /wd4191 /wd4201 /wd4204 /wd4152 /wd4211 /wd4244 /wd4255 /wd4389 /wd4458 /wd4505 /wd4668 /wd4706 /wd4710 /wd4711 /wd4820 /wd5045")

	set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /Od")
	set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} /Zi /O2 /fp:fast")
	set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Zi /GS- /Ox /Oi /Ot /Ob2 /Oy /fp:fast")

endif()


file(GLOB source 
    include/GLFW/glfw3.h
    include/GLFW/glfw3native.h
	src/init.c 
	src/input.c 
	src/monitor.c 
	src/vulkan.c 
	src/context.c 
	src/egl_context.c 
	src/egl_context.c
	src/wgl_context.c 
	src/window.c)

add_library(${projectName} STATIC ${source})

if (MSVC)
     target_sources(${projectName} PRIVATE 
		#src/win32_time.h 
		#src/win32_thread.h 
		src/win32_joystick.h
		src/win32_platform.h
		src/win32_init.c
		src/win32_time.c
		src/win32_monitor.c
		src/win32_joystick.c
		src/win32_thread.c
		src/win32_window.c)
elseif(APPLE)
     target_sources(${projectName} PRIVATE 
		src/cocoa_joystick.h
		src/cocoa_platform.h
		src/cocoa_init.m
		src/cocoa_monitor.m
		src/cocoa_window.c
		src/cocoa_joystick.m
		src/nsgl_context.m)
endif()

set_target_properties(${projectName} PROPERTIES FOLDER "thirdParty")
