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

set (projectName "ndRender")
message (${projectName})

include_directories(.)
include_directories(../../sdk/dCore)
include_directories(../../sdk/dModel)
#include_directories(../../sdk/dNewton)
include_directories(../../sdk/dCollision)
include_directories(../../sdk/dCore/tinyxml)

include_directories(../../thirdParty/png)
include_directories(../../thirdParty/imgui/)

if (APPLE) 
	message ("render library with metal backend")
	include_directories(Backends/RenderMacOS)
	file(GLOB CPP_SOURCE 
		*.h
		*.cpp 
		Backends/RenderMacOS/*.h
		Backends/RenderMacOS/*.cpp)
elseif(WIN32 OR UNIX)
	message ("render library with opengl backend")
	include_directories(Backends/RenderOpenGl)
	include_directories(../../thirdParty/glfw/)
	include_directories(../../thirdParty/glfw/include/)
	include_directories(../../thirdParty/glatter/include/glatter/)
	
	file(GLOB CPP_SOURCE 
		*.h
		*.cpp 
		Backends/RenderOpenGl/*.h
		Backends/RenderOpenGl/*.cpp)
else()
	include_directories(Backends/RenderBlank)
	message ("!!!!!!!!!!!!! add apply support !!!!!!!!")
		file(GLOB CPP_SOURCE 
		*.h
		*.cpp 
		Backends/RenderBlank/*.h
		Backends/RenderBlank/*.cpp)
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

add_library(${projectName} STATIC ${CPP_SOURCE})

if (MSVC)
	set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YundRenderStdafx.h")
	set_source_files_properties(ndRenderStdafx.cpp PROPERTIES COMPILE_FLAGS "/YcndRenderStdafx.h")
endif()

if (MSVC)
	set_target_properties(${projectName} PROPERTIES FOLDER "render")
endif()

