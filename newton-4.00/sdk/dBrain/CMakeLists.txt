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

set (projectName "ndBrain")
message (${projectName})

include_directories(.)
include_directories(../dCore)
include_directories(../dCore/tinyxml)
include_directories(../../thirdParty/png)

if(NEWTON_ENABLE_OPENCL_SDK)
	include_directories(dOpencl)
	include_directories(../../thirdParty/opencl/OpenCL-SDK-v2024.10.24-Win-x64/include)
	file(GLOB CPP_SOURCE *.cpp dOpenCl/*.cpp *.h dOpenCl/*.h)
else()
	include_directories(../../sdk/dBrain/cpuEmulation)
	file(GLOB CPP_SOURCE *.cpp cpuEmulation/*.cpp *.h cpuEmulation/*.h)
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

add_library(${projectName} STATIC ${CPP_SOURCE})

if (MSVC)
	#set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /Od")
	#set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELEASE} /Zi /W4 /O2 /fp:fast")
	#set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Zi /GS- /W4 /Ox /Oi /Ot /Ob2 /Oy /fp:fast")

	set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YundBrainStdafx.h")
	set_source_files_properties(ndBrainStdafx.cpp PROPERTIES COMPILE_FLAGS "/YcndBrainStdafx.h")
endif()

if (MSVC)
	set_target_properties(${projectName} PROPERTIES FOLDER "deepLearningSDK")
endif()

