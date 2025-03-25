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

set (projectName "ndNewton")
message (${projectName})
project(ProjectName)

include_directories(.)
include_directories(dCore)
include_directories(dNewton)
include_directories(dTinyxml)
include_directories(dCollision)
include_directories(dNewton/dJoints)
include_directories(dNewton/dModels)
include_directories(dNewton/dIkSolver)
include_directories(dNewton/dParticles)
include_directories(dNewton/dModels/dVehicle)
include_directories(dNewton/dModels/dCharacter)

# source and header files
file(GLOB HEADERS 
	*.h
	dCore/*.h 
	dNewton/*.h 
	dTinyxml/*.h 
	dCollision/*.h 
	dNewton/dJoints/*.h 
	dNewton/dModels/*.h 
	dNewton/dIkSolver/*.h
	dNewton/dParticles/*.h 
	dNewton/dModels/dVehicle/*.h
	dNewton/dModels/dCharacter/*.h)

file(GLOB CPP_SOURCE 
	*.h
	*.cpp
	*.cxx
	dCore/*.h 
	dCore/*.cpp
	dNewton/*.h 
	dNewton/*.cpp
	dTinyxml/*.h
	dTinyxml/*.cpp
	dCollision/*.h 
	dCollision/*.cpp
	dNewton/dModels/*.h 
	dNewton/dModels/*.cpp
	dNewton/dJoints/*.h 
	dNewton/dJoints/*.cpp
	dNewton/dIkSolver/*.h
	dNewton/dIkSolver/*.cpp
	dNewton/dParticles/*.h
	dNewton/dParticles/*.cpp
	dNewton/dModels/dVehicle/*.h 
	dNewton/dModels/dVehicle/*.cpp
	dNewton/dModels/dCharacter/*.h
	dNewton/dModels/dCharacter/*.cpp)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

add_definitions(-D_D_SINGLE_LIBRARY)
#add_definitions(-DD_SCALAR_VECTOR_CLASS)

add_library(${projectName} SHARED ${CPP_SOURCE})
   



