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

set (projectName "newtonJava")
message (${projectName})

if(MSVC OR MINGW)
	set (swig "${CMAKE_SOURCE_DIR}/thirdParty/swigwin/swigwin-4.0.2/swig.exe")
else()
	set (swig "/usr/bin/swig")
endif()

set (wrapper "${PROJECT_BINARY_DIR}/applications/toolsAndWrapers/${projectName}")
set (swigScript "${CMAKE_SOURCE_DIR}/applications/toolsAndWrapers/${projectName}")

file(MAKE_DIRECTORY ${wrapper}/classes)
execute_process(COMMAND ${swig} -c++ -outdir "${wrapper}/classes" -java ${swigScript}/newton.i)

# source and header files
file(GLOB CPP_SOURCE 
	*.h
	*.cpp
	*.cxx
	*.i
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

include_directories(.)
include_directories(../../../sdk/dCore)
include_directories(../../../sdk/dNewton)
include_directories(../../../sdk/dTinyxml)
include_directories(../../../sdk/dCollision)
include_directories(../../../sdk/dNewton/dJoints)
include_directories(../../../sdk/dNewton/dModels)
include_directories(../../../sdk/dNewton/dIkSolver)
include_directories(../../../sdk/dNewton/dParticles)
include_directories(../../../sdk/dNewton/dModels/dVehicle)
include_directories(../../../sdk/dNewton/dModels/dCharacter)

add_library(${projectName} SHARED ${CPP_SOURCE})

if (MSVC OR MINGW)
	#message ("$ENV{Java}")
	target_include_directories(${projectName} BEFORE PRIVATE "$ENV{Java}/include")
	target_include_directories(${projectName} BEFORE PRIVATE "$ENV{Java}/include/win32")
	target_link_directories(${projectName} BEFORE PRIVATE "$ENV{Java}/lib")
#else()
#	find_package (Python COMPONENTS Development REQUIRED)
#	target_include_directories(${projectName} PUBLIC ${Python_INCLUDE_DIRS})
#	target_link_libraries(${projectName} ${Python_LIBRARIES})
endif()

if (NEWTON_BUILD_CREATE_SUB_PROJECTS)
	target_link_libraries (${projectName} ndNewton ndCollision ndCore)
else()
	target_link_libraries (${projectName} ndNewton)
endif()

if (NEWTON_ENABLE_CUDA_SOLVER)
	target_link_libraries (${projectName} ndSolverCuda)
endif()

if(MSVC OR MINGW)
    if(CMAKE_VS_MSBUILD_COMMAND OR CMAKE_VS_DEVENV_COMMAND)
		set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YunewtonStdafx.h")
		set_source_files_properties(newtonStdafx.cpp PROPERTIES COMPILE_FLAGS "/YcnewtonStdafx.h")
		set_source_files_properties(newton_wrap.cxx PROPERTIES COMPILE_FLAGS "/Y-")
    endif()
endif()
