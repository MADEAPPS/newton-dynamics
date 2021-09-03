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

set (projectName "ndCollision")
message (${projectName})

#source and header files
file(GLOB CPP_SOURCE *.h *.cpp)
file(GLOB HEADERS *.h)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

include_directories(../dCore/)
include_directories(../dTinyxml/)

if (MSVC)
       set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /fp:fast")
       set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /fp:fast")
endif(MSVC)

if(NEWTON_BUILD_SHARED_LIBS)
	add_definitions(-D_D_COLLISION_EXPORT_DLL)
	add_library(${projectName} SHARED ${CPP_SOURCE})

	if(MSVC OR MINGW)
		target_link_options(${projectName} PUBLIC "/DEBUG") 
	endif()
else()
	add_library(${projectName} STATIC ${CPP_SOURCE})
endif()

if (MSVC)
	if(CMAKE_VS_MSBUILD_COMMAND OR CMAKE_VS_DEVENV_COMMAND)
		set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YundCollisionStdafx.h")
		set_source_files_properties(ndCollisionStdafx.cpp PROPERTIES COMPILE_FLAGS "/YcndCollisionStdafx.h")
	endif()

	if (NEWTON_BUILD_TEST AND NEWTON_BUILD_SHARED_LIBS)
		add_custom_command(
			TARGET ${projectName} POST_BUILD
			COMMAND ${CMAKE_COMMAND}
			ARGS -E copy $<TARGET_FILE:${projectName}> ${PROJECT_BINARY_DIR}/applications/ndTest/${CMAKE_CFG_INTDIR}/$<TARGET_FILE_NAME:${projectName}>)
	endif ()

	if (NEWTON_BUILD_SANDBOX_DEMOS AND NEWTON_BUILD_SHARED_LIBS)
		add_custom_command(
			TARGET ${projectName} POST_BUILD
			COMMAND ${CMAKE_COMMAND}
			ARGS -E copy $<TARGET_FILE:${projectName}> ${PROJECT_BINARY_DIR}/applications/ndSandbox/${CMAKE_CFG_INTDIR}/$<TARGET_FILE_NAME:${projectName}>)
	endif ()

endif(MSVC)

target_link_libraries(${projectName} ndCore ndTinyxml)

install(TARGETS ${projectName} 
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib
		RUNTIME DESTINATION bin)

install(FILES ${HEADERS} DESTINATION include/${projectName})

