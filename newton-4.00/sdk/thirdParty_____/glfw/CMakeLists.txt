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

set (projectName "glfw")
message (${projectName})

add_definitions(-D_GLFW_WIN32)

 # low level core
file(GLOB source src/init.c src/input.c src/monitor.c src/vulkan.c 
				 src/context.c src/egl_context.c src/egl_context.c
				 src/wgl_context.c src/win32_*.c src/window.c)

add_library(${projectName} STATIC ${source})

