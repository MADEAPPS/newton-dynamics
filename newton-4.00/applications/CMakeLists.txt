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

add_subdirectory(toolsAndWrapers)

if (NEWTON_BUILD_TEST)
	add_subdirectory(ndTest)
endif()

if (NEWTON_BUILD_PHYSIC_EDITOR)
    add_subdirectory(ndAuthor)
endif()

if (NEWTON_BUILD_SANDBOX_DEMOS)
	add_subdirectory(ndSandbox)

    find_package(OpenGL REQUIRED)
    if(NOT_OpenGL_FOUND)
        message(WARNING "OpenGL headers were not found. Disable NEWTON_BUILD_SANDBOX_DEMOS or install OpenGL")
    else(NOT_OpenGL_FOUND)
        message("Found OpenGL library")
    endif(NOT_OpenGL_FOUND)
	
     # common libraries for unix systems
     if(CMAKE_SYSTEM_NAME STREQUAL "Linux")

        find_package(libcursorx REQUIRED)
        if(NOT_libcursorx_FOUND)
            message(WARNING "libcursorx headers were not found. Disable NEWTON_BUILD_SANDBOX_DEMOS or install libcursorx")
        else(NOT_libcursorx_FOUND)
            message("Found libcursorx library")
        endif(NOT_libcursorx_FOUND)

     endif()
        
endif()



	
