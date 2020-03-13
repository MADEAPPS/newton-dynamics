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

set (projectName "dgCore")
message (${projectName})

# source and header files
file(GLOB CPP_SOURCE *.h *.cpp)
file(GLOB HEADERS *.h)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

if(NEWTON_BUILD_SHARED_LIBS STREQUAL "ON")
	if(MSVC)
		add_library(${projectName} STATIC ${CPP_SOURCE})
	else()
		add_library(${projectName} SHARED ${CPP_SOURCE})
	endif()
else()
	add_library(${projectName} STATIC ${CPP_SOURCE})
endif()

if (MSVC)
	if(CMAKE_VS_MSBUILD_COMMAND OR CMAKE_VS_DEVENV_COMMAND)
		set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YudgStdAfx.h")
		set_source_files_properties(dgTypes.cpp PROPERTIES COMPILE_FLAGS "/YcdgStdAfx.h")
	endif()
endif(MSVC)

target_include_directories(${projectName} PUBLIC .)
if (NEWTON_BUILD_PROFILER)
    target_link_libraries (${projectName} dProfiler)
endif()

install(TARGETS ${projectName} 
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib
		RUNTIME DESTINATION bin)

install(FILES ${HEADERS} DESTINATION include/${projectName})
