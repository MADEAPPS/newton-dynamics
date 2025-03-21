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

set (projectName "dAnimation")
message (${projectName})

# low level core
file(GLOB CPP_SOURCE *.cpp)
file(GLOB HEADERS *.h)

if(NEWTON_BUILD_SHARED_LIBS AND NOT MSVC)
    add_library(${projectName} SHARED ${CPP_SOURCE})
else()
    add_library(${projectName} STATIC ${CPP_SOURCE})
endif()

if (MSVC)
	if(NOT NEWTON_BUILD_SHARED_LIBS)
	    add_definitions(-D_NEWTON_STATIC_LIB)
	    add_definitions(-D_CUSTOM_JOINTS_STATIC_LIB)
	endif(NOT NEWTON_BUILD_SHARED_LIBS)

    if(CMAKE_VS_MSBUILD_COMMAND OR CMAKE_VS_DEVENV_COMMAND)
        set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YudAnimationStdAfx.h")
        set_source_files_properties(dAnimationStdAfx.cpp PROPERTIES COMPILE_FLAGS "/YcdAnimationStdAfx.h")
    endif()
endif(MSVC)

target_include_directories(${projectName} PUBLIC .)
target_link_libraries(${projectName} dCustomJoints)
if (NEWTON_BUILD_PROFILER)
    target_link_libraries (${projectName} dProfiler)
endif()

install(TARGETS ${projectName} 
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib
		RUNTIME DESTINATION bin)

install(FILES ${HEADERS} DESTINATION include/${projectName})
