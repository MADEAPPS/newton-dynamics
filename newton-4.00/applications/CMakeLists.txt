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

        #find_package(xcursor-dev REQUIRED)
        #if(NOT_xcursor-dev_FOUND)
        #    message(WARNING "xcursor-dev headers were not found. Disable NEWTON_BUILD_SANDBOX_DEMOS or install xcursor-dev")
        #else(NOT_xcursor-dev_FOUND)
        #    message("Found xcursor-dev library")
        #endif(NOT_xcursor-dev_FOUND)

        find_package(X11 REQUIRED COMPONENTS Xcursor)
        # Check if Xcursor was found
        if (X11_Xcursor_FOUND)
            message(STATUS "Xcursor found.")
            # Add include directories for Xcursor
            include_directories(${X11_Xcursor_INCLUDE_DIR})
            # Link libraries for Xcursor
            #target_link_libraries(YourTargetName PRIVATE ${X11_Xcursor_LIBRARIES})
        else()
            message(FATAL_ERROR "Xcursor not found. Please install libxcursor-dev or similar package or Disable NEWTON_BUILD_SANDBOX_DEMOS")
        endif()

     endif()
        
endif()



	
