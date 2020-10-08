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


set (projectName "ndNewtonC")
message (${projectName})

include_directories(../dCore/)
include_directories(../dNewton/)
include_directories(../dCollision/)
include_directories(../dNewton/dJoints/)
	
# source and header files
file(GLOB HEADERS *.h *.cpp ../dCore/*.h ../dNewton/*.h ../dCollision/*.h ../dNewton/dJoints/*.h)
file(GLOB CPP_SOURCE 
	*.h
	*.cpp
	../dCore/*.h 
	../dCore/*.cpp
	../dNewton/*.h 
	../dNewton/*.cpp
	../dCollision/*.h 
	../dCollision/*.cpp
	../dNewton/dJoints/*.h 
	../dNewton/dJoints/*.cpp)

#message("${CMAKE_CURRENT_SOURCE_DIR}/../")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../" FILES ${CPP_SOURCE})

add_definitions(-D_D_SINGLE_LIBRARY)
add_definitions(-D_D_NEWTON_BUILD_DLL)

#if(NEWTON_BUILD_SHARED_LIBS STREQUAL "ON")
#	add_definitions(-D_D_CORE_EXPORT_DLL)
#	add_definitions(-D_D_COLLISION_EXPORT_DLL)
#	add_definitions(-D_D_NEWTON_EXPORT_DLL)
#	add_library(${projectName} SHARED ${CPP_SOURCE})
#else()
#	add_library(${projectName} STATIC ${CPP_SOURCE})
#endif()

add_library(${projectName} SHARED ${CPP_SOURCE})
    
#if (MSVC)
#	if(CMAKE_VS_MSBUILD_COMMAND OR CMAKE_VS_DEVENV_COMMAND)
#		set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YudCoreStdAfx.h")
#		set_source_files_properties(dCore/dCoreStdAfx.cpp PROPERTIES COMPILE_FLAGS "/YcdCoreStdAfx.h")
#	endif()
#endif(MSVC)

target_include_directories(${projectName} PUBLIC .)
if (NEWTON_BUILD_PROFILER)
	target_link_libraries (${projectName} dProfiler)
endif()


add_custom_command(
	TARGET ${projectName} POST_BUILD
	COMMAND ${CMAKE_COMMAND}
	ARGS -E copy $<TARGET_FILE:${projectName}> ${PROJECT_BINARY_DIR}/applications/ndTestC/${CMAKE_CFG_INTDIR}/$<TARGET_FILE_NAME:${projectName}>)


install(TARGETS ${projectName} 
		LIBRARY DESTINATION libc
		ARCHIVE DESTINATION libc
		RUNTIME DESTINATION binc)

install(FILES ${HEADERS} DESTINATION includec/${projectName})



	
