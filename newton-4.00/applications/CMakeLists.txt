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

        find_package(X11 REQUIRED)
        if (X11_FOUND)
            message(STATUS "X11 found.")
            # Add include directories for Xcursor
            include_directories(${X11_INCLUDE_DIR})
        else()
            message(FATAL_ERROR "X11 not found. Please install libx11 or similar package or Disable NEWTON_BUILD_SANDBOX_DEMOS")
        endif()
 
        find_package(X11 REQUIRED COMPONENTS Xcursor)
        if (X11_Xcursor_FOUND)
            message(STATUS "Xcursor found.")
            # Add include directories for Xcursor
            include_directories(${X11_Xcursor_INCLUDE_DIR})
        else()
            message(FATAL_ERROR "Xcursor not found. Please install libxcursor-dev or similar package or Disable NEWTON_BUILD_SANDBOX_DEMOS")
        endif()

        find_package(Xinerama REQUIRED)
        #find_package(X11 REQUIRED COMPONENTS Xinerama)
        #find_library(XINERAMA_LIBRARY NAMES Xinerama xinerama)
        #if (X11_Xinerama_FOUND)
        #    message(STATUS "Xinerama found.")
        #    # Add include directories for Xinerama
        #    include_directories(${X11_Xinerama_INCLUDE_DIR})
        #else()
        #    message(FATAL_ERROR "Xinerama not found. Please install libXinerama-dev or similar package or Disable NEWTON_BUILD_SANDBOX_DEMOS")
        #endif()
     
        #find_library(XINERAMA_LIBRARY NAMES Xinerama xinerama)
        #find_path(XINERAMA_INCLUDE_DIR NAMES X11/extensions/Xinerama.h)
        #if(XINERAMA_LIBRARY AND XINERAMA_INCLUDE_DIR)
        #    include_directories(${XINERAMA_INCLUDE_DIR})
        #    #target_link_libraries(YourTarget ${XINERAMA_LIBRARY})
        #else()
        #    message(FATAL_ERROR "Xinerama library or headers not found!")
        #endif()

     endif()
        
endif()



	
