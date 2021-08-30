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

set (projectName "newtonPy")
message (${projectName})

#execute_process(COMMAND ${CMAKE_SOURCE_DIR}/thirdParty/swigwin/swigwin-4.0.2/swig.exe -c++ -python ${CMAKE_SOURCE_DIR}/applications/toolsAndWrapers/newtonPy/newtonPy.i)

set (swig "${CMAKE_SOURCE_DIR}/thirdParty/swigwin/swigwin-4.0.2/swig.exe")
set (script "${CMAKE_SOURCE_DIR}/applications/toolsAndWrapers/newtonPy/newtonPy.i")
execute_process(COMMAND ${swig} -c++ -python ${script})

# source and header files
file(GLOB CPP_SOURCE 
	*.h
	*.cpp
	*.cxx
)

#source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" PREFIX source/ FILES ${CPP_SOURCE})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

include_directories(./)
include_directories(../../../sdk/dCore/)
include_directories(../../../sdk/dNewton/)
include_directories(../../../sdk/dTinyxml/)
include_directories(../../../sdk/dCollision/)
include_directories(../../../sdk/dNewton/dJoints)
include_directories(../../../sdk/dNewton/dModels)
include_directories(../../../sdk/dNewton/dModels/dVehicle)
include_directories(../../../sdk/dNewton/dModels/dCharacter)

set(CMAKE_DEBUG_POSTFIX "")
add_library(${projectName} SHARED ${CPP_SOURCE})

#message ($ENV{PythonInclude})
target_include_directories(${projectName} BEFORE PRIVATE "$ENV{Python}/Include")

#message ("$ENV{Python}")
target_link_directories(${projectName} BEFORE PRIVATE "$ENV{Python}/Libs")

if (NEWTON_BUILD_CREATE_SUB_PROJECTS)
	target_link_libraries (${projectName} ndNewton ndCollision ndCore)
else()
	target_link_libraries (${projectName} ndNewton)
endif()

if(MSVC OR MINGW)
    if(CMAKE_VS_MSBUILD_COMMAND OR CMAKE_VS_DEVENV_COMMAND)
		set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YunewtonPyStdafx.h")
		set_source_files_properties(newtonPyStdafx.cpp PROPERTIES COMPILE_FLAGS "/YcnewtonPyStdafx.h")
		set_source_files_properties(newtonPy_wrap.cxx PROPERTIES COMPILE_FLAGS "/Y-")
    endif()
endif()



