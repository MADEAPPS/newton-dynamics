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

set (projectName "ndTest")
message (${projectName})

# source and header files
file(GLOB CPP_SOURCE *.h *.cpp)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

include_directories(../../sdk/dCore/)
include_directories(../../sdk/dNewton/)
include_directories(../../sdk/dTinyxml/)
include_directories(../../sdk/dCollision/)
include_directories(../../sdk/dNewton/dJoints)
include_directories(../../sdk/dNewton/dModels)
include_directories(../../sdk/dNewton/dIkSolver)
include_directories(../../sdk/dNewton/dParticles)
include_directories(../../sdk/dNewton/dModels/dVehicle)
include_directories(../../sdk/dNewton/dModels/dCharacter)

if(MSVC)
	set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /DEBUG") 
	message (${CMAKE_EXE_LINKER_FLAGS})
endif ()

if (NEWTON_BUILD_PROFILER)
	include_directories(../../sdk/dProfiler/dProfiler/)
endif ()

add_executable(${projectName} ${CPP_SOURCE})

if (NEWTON_BUILD_CREATE_SUB_PROJECTS)
	target_link_libraries (${projectName} ndCore ndTinyxml ndCollision ndNewton)
else()
	target_link_libraries (${projectName} ndNewton)
endif()

if (NEWTON_BUILD_PROFILER)
    target_link_libraries (${projectName} dProfiler)
endif ()
