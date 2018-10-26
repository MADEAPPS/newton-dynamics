# Copyright (c) <2014-2017> <Newton Game Dynamics>
#
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

cmake_minimum_required(VERSION 3.12.0)

include_directories(dMath/)
include_directories(dgCore/)
include_directories(dScene/)
include_directories(dNewton/)
include_directories(dgNewton/)
include_directories(dVehicle/)
include_directories(dgPhysics/)
include_directories(dAnimation/)
include_directories(dgMeshUtil/)
include_directories(dContainers/)
include_directories(dCustomJoints/)
include_directories(thirdParty/tinyxml/)
include_directories(dProfiler/dTimeTracker/)

add_subdirectory(dMath)
add_subdirectory(dgCore)
add_subdirectory(dScene)
add_subdirectory(dNewton)
add_subdirectory(dgNewton)
add_subdirectory(dVehicle)
add_subdirectory(dgPhysics)
add_subdirectory(dAnimation)
add_subdirectory(dContainers)
add_subdirectory(dCustomJoints)
add_subdirectory(thirdParty)

if (BUILD_PROFILER)
	add_subdirectory(dProfiler)
endif (BUILD_PROFILER)

if (WITH_SSE_PLUGIN)
	add_subdirectory(dgNewtonSse)
endif()

if (WITH_SSE4_PLUGIN)
	add_subdirectory(dgNewtonSse4.2)
endif()

if (WITH_AVX_PLUGIN)
	add_subdirectory(dgNewtonAvx)
endif()

if (WITH_AVX2_PLUGIN)
	add_subdirectory(dgNewtonAvx2)
endif()

if (WITH_DX12_PLUGIN)
	add_subdirectory(dgNewtonDx12)
endif()
