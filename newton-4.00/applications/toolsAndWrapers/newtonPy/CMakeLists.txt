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

set (swig "${CMAKE_SOURCE_DIR}/thirdParty/swigwin/swigwin-4.0.2/swig.exe")
set (script "${CMAKE_SOURCE_DIR}/applications/toolsAndWrapers/newtonPy/newtonPy.i")
execute_process(COMMAND ${swig} -c++ -python ${script})

# source and header files
file(GLOB CPP_SOURCE 
	*.h
	*.cpp
	*.cxx
	*.py
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

#install(FILES ${projectName}.py DESTINATION ${projectName})
#install(TARGETS ${projectName} RUNTIME DESTINATION ${projectName})

message ("${CMAKE_CURRENT_SOURCE_DIR}/${projectName}.py")
add_custom_command(
	TARGET ${projectName} POST_BUILD COMMAND ${CMAKE_COMMAND}
	ARGS -E copy ${CMAKE_CURRENT_SOURCE_DIR}/${projectName}.py ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_PREFIX}/${projectName}/${projectName}.py)

add_custom_command(
	TARGET ${projectName} POST_BUILD COMMAND ${CMAKE_COMMAND}
	ARGS -E copy ${CMAKE_CURRENT_SOURCE_DIR}/ndbpy.py ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_PREFIX}/${projectName}/ndbpy.py)

add_custom_command(
	TARGET ${projectName} POST_BUILD COMMAND ${CMAKE_COMMAND}
	ARGS -E copy $<TARGET_FILE:${projectName}> ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_PREFIX}/${projectName}/_$<TARGET_FILE_BASE_NAME:${projectName}>.pyd)

add_custom_command(
	TARGET ${projectName} POST_BUILD COMMAND ${CMAKE_COMMAND}
	ARGS -E copy ${CMAKE_CURRENT_SOURCE_DIR}/${projectName}.py $ENV{Blender}/2.93/scripts/addons/${projectName}/${projectName}.py)

add_custom_command(
	TARGET ${projectName} POST_BUILD COMMAND ${CMAKE_COMMAND}
	ARGS -E copy ${CMAKE_CURRENT_SOURCE_DIR}/ndbpy.py $ENV{Blender}/2.93/scripts/addons/${projectName}/ndbpy.py)

add_custom_command(
	TARGET ${projectName} POST_BUILD COMMAND ${CMAKE_COMMAND}
	ARGS -E copy $<TARGET_FILE:${projectName}> $ENV{Blender}/2.93/scripts/addons/${projectName}/_$<TARGET_FILE_BASE_NAME:${projectName}>.pyd)
