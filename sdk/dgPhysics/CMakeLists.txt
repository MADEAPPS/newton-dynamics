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

set (projectName "dgPhysics")
message (${projectName})

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /fp:fast")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /fp:fast")

 # low level core
file(GLOB source *.cpp *.h ../dgMeshUtil/*.cpp ../dgMeshUtil/*.h)

add_library(${projectName} STATIC ${source})

if (MSVC)
	set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YudgPhysicsStdafx.h")
	set_source_files_properties(dgWorld.cpp PROPERTIES COMPILE_FLAGS "/YcdgPhysicsStdafx.h")
endif(MSVC)

if (NOT GENERATE_DLL)
	install(TARGETS ${projectName} ARCHIVE DESTINATION ${libraryPath})
endif()