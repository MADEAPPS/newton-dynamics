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

set (projectName "newtonC")
project(${projectName})

message (${projectName})

if(MSVC OR MINGW)
	set (swig "${CMAKE_SOURCE_DIR}/../../../thirdParty/swigwin/swigwin-4.4.1/swig.exe")
else()
	set (swig "/usr/bin/swig")
endif()

set (wrapper "${PROJECT_BINARY_DIR}")
set (swigScript "${CMAKE_SOURCE_DIR}")

execute_process(COMMAND ${swig} -c++ -c ${swigScript}/newton.i)

# source and header files
file(GLOB CPP_SOURCE 
	*.h
	*.cpp
	*.cxx
	*.i
	install/source/*.h
	install/source/*.c
	install/source/*.cpp
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

include_directories(.)
include_directories(install/source)

add_library(${projectName} SHARED ${CPP_SOURCE})

#set (pySrc "${CMAKE_CURRENT_SOURCE_DIR}")
#list(APPEND PY_SOURCE 
#	ndbpy.py
#	newton.py
#	newtonWorld.py)

if (MSVC OR MINGW)
  set (cSuffix ".dll")
else()
  set (cSuffix ".so")
endif()

#foreach(file IN LISTS PY_SOURCE)
#	message ("copy ${pySrc}/${file} to ${wrapper}/${projectName}/${file}")
#	add_custom_command(
#	TARGET ${projectName} POST_BUILD COMMAND ${CMAKE_COMMAND}
#	ARGS -E copy ${pySrc}/${file} ${wrapper}/${projectName}/${file})
#endforeach()

add_custom_command(
	TARGET ${projectName} POST_BUILD COMMAND ${CMAKE_COMMAND}
	ARGS -E copy $<TARGET_FILE:${projectName}> ${wrapper}/${projectName}/${projectName}${CMAKE_DEBUG_POSTFIX}${cSuffix})
