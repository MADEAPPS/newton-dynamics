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

set (projectName "ndTinyxml")
message (${projectName})

include_directories(.)

file(GLOB CPP_SOURCE *.cpp *.h)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

add_library(${projectName} STATIC ${CPP_SOURCE})

if (MSVC)
	#set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /Od")
	#set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELEASE} /Zi /W4 /O2 /fp:fast")
	#set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Zi /GS- /W4 /Ox /Oi /Ot /Ob2 /Oy /fp:fast")

	#set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YundTinyxmlStdafx.h")
	#set_source_files_properties(ndBrainStdafx.cpp PROPERTIES COMPILE_FLAGS "/YcndTinyxmlStdafx.h")

endif()
