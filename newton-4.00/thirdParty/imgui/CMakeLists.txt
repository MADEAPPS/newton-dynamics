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

set (projectName "imgui")
message (${projectName})

include_directories(.)
include_directories(backends)
if (APPLE)

    file(GLOB CPP_SOURCE 
	    *.h 
	    *.cpp
        backends/imgui_impl_osx.h
        backends/imgui_impl_osx.mm
        backends/imgui_impl_metal.h
	    backends/imgui_impl_metal.mm
    )

else()
    include_directories(../glfw/include)

    # low level core
    file(GLOB CPP_SOURCE 
	    *.h 
	    *.cpp 
	    backends/imgui_impl_glfw.h
	    backends/imgui_impl_glfw.cpp
	    backends/imgui_impl_opengl3.h
	    backends/imgui_impl_opengl3.cpp
    )
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

add_library(${projectName} STATIC ${CPP_SOURCE})

if (MSVC)
	target_compile_options(${projectName} PRIVATE "/W4")
endif()

if (MSVC)
	set_target_properties(${projectName} PROPERTIES FOLDER "thirdParty")
endif()
