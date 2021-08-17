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

set (projectName "ndTinyxml")
message (${projectName})

# low level core
file(GLOB CPP_SOURCE *.cpp *.h)
file(GLOB HEADERS *.h)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

#include_directories(../dCore)
#include_directories(../dNewton)
include_directories(../dTinyxml/)
#include_directories(../dCollision)

if(NEWTON_BUILD_SHARED_LIBS STREQUAL "ON")
	add_definitions(-D_D_TINYXML_EXPORT_DLL)
	add_library(${projectName} SHARED ${CPP_SOURCE})
else()
	add_library(${projectName} STATIC ${CPP_SOURCE})
endif()

if(UNIX)
    target_compile_options(${projectName} PRIVATE -fPIC)
endif()

install(TARGETS ${projectName} 
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib
		RUNTIME DESTINATION bin)

install(FILES ${HEADERS} DESTINATION include/${projectName})
