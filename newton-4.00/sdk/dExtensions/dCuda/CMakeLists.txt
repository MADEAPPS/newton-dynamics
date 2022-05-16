# Copyright (c) <2014-2017> <Newton Game Dynamics>
#
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

cmake_minimum_required(VERSION 3.9.0 FATAL_ERROR)

set (projectName "ndSolverCuda")
message (${projectName})

set(CMAKE_CUDA_STANDARD 14)
CMAKE_POLICY(SET CMP0104 OLD)
project(cmake_and_cuda LANGUAGES CXX CUDA)
SET(CMAKE_CUDA_COMPILER $ENV{CUDA_PATH}/bin/nvcc.exe.exe)

include_directories(../../.)
include_directories(../../dCore)
include_directories(../../dNewton)
include_directories(../../dTinyxml)
include_directories(../../dProfiler)
include_directories(../../dCollision)
include_directories(../../dNewton/dJoints)
include_directories(../../dNewton/dModels)
include_directories(../../dNewton/dIkSolver)
include_directories(../../dNewton/dParticles)
include_directories(../../dNewton/dModels/dVehicle)
include_directories(../../dNewton/dModels/dCharacter)

include_directories(./)
include_directories(cuNatives/)

file(GLOB CPP_SOURCE *.h *.cu *.cpp cuNatives/*.h cuNatives/*.cu  cuNatives/*.cpp)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

Set(CMAKE_CUDA_FLAGS_DEBUG "-Xcompiler=\"-MDd -W4 -wd4201 -wd4324 -Zi -Ob0 -Od /RTC1\" --extended-lambda")
Set(CMAKE_CUDA_FLAGS_RELEASE "-Xcompiler=\"-MD -W4 -wd4201 -wd4324 -Zi -O2 -Ob2\" -DNDEBUG --extended-lambda") 
Set(CMAKE_CUDA_FLAGS_RELWITHDEBINFO "-Xcompiler=\"-MD -W4 -wd4201 -wd4324 -Zi -O2 -Ob1\" -DNDEBUG --extended-lambda")

#Set(CMAKE_CUDA_FLAGS_DEBUG "${CMAKE_CUDA_FLAGS_DEBUG} -lineinfo")
Set(CMAKE_CUDA_FLAGS_RELEASE "${CMAKE_CUDA_FLAGS_RELEASE} -lineinfo")
#Set(CMAKE_CUDA_FLAGS_RELWITHDEBINFO "${CMAKE_CUDA_FLAGS_RELWITHDEBINFO} -lineinfo")

# this does not fucking works, unless yo umak ethe p[ropject a cuda solution.]
#Set(CMAKE_CUDA_FLAGS_DEBUG "${CMAKE_CUDA_FLAGS_DEBUG} -rdc=true")

Set(CMAKE_CUDA_FLAGS_DEBUG "${CMAKE_CUDA_FLAGS_DEBUG} --device-debug")
Set(CMAKE_CUDA_FLAGS_RELWITHDEBINFO "${CMAKE_CUDA_FLAGS_RELWITHDEBINFO} --device-debug")

#Set(CMAKE_CUDA_FLAGS_DEBUG "${CMAKE_CUDA_FLAGS_DEBUG} -use_fast_math")
#Set(CMAKE_CUDA_FLAGS_RELEASE "${CMAKE_CUDA_FLAGS_RELEASE} -use_fast_math")
#Set(CMAKE_CUDA_FLAGS_RELWITHDEBINFO "${CMAKE_CUDA_FLAGS_RELWITHDEBINFO} -use_fast_math")

message ("cuda debug options ${CMAKE_CUDA_FLAGS_DEBUG}")
message ("cuda release options ${CMAKE_CUDA_FLAGS_RELEASE}")
message ("cuda release with debug options ${CMAKE_CUDA_FLAGS_RELWITHDEBINFO}")

if(MSVC)
	add_library(${projectName} STATIC ${CPP_SOURCE})
endif()

if(MINGW)
	add_library(${projectName} STATIC ${CPP_SOURCE})
endif()

if(UNIX)
	add_library(${projectName} SHARED ${CPP_SOURCE})
endif()

install(TARGETS ${projectName} 
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib
		RUNTIME DESTINATION bin)

install(FILES ${HEADERS} DESTINATION include/${projectName})
