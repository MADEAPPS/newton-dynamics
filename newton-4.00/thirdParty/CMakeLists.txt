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

if(NEWTON_TEST_NEW_V_HACD)
	add_subdirectory(hacdNew)
endif(NEWTON_TEST_NEW_V_HACD)

add_subdirectory(png)
add_subdirectory(imgui)
add_subdirectory(openFBX/src)
add_subdirectory(hacd/src/VHACD_Lib)

if (NOT APPLE)
	add_subdirectory(glfw)
	add_subdirectory(glatter)
endif()
