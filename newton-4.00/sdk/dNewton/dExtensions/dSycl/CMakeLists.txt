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

set (projectName "ndSolverSycl")
message (${projectName})

include_directories(./)
include_directories ($ENV{ONEAPI_ROOT}/compiler/latest/windows/include)

file(GLOB CPP_SOURCE *.h *.cpp)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

add_definitions(-D_D_SYCL_EXPORT_DLL)
add_library(${projectName} SHARED ${CPP_SOURCE})

#Intel(R) oneAPI DPC++ Compiler 2023
set_target_properties(${projectName} PROPERTIES VS_PLATFORM_TOOLSET "Intel(R) oneAPI DPC++ Compiler 2023" )

install(FILES ${HEADERS} DESTINATION include/${projectName})
install(FILES $<TARGET_PDB_FILE:${projectName}> DESTINATION bin OPTIONAL)
install(TARGETS ${projectName} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

if (NEWTON_BUILD_SANDBOX_DEMOS)
	add_custom_command(
		TARGET ${projectName} POST_BUILD
		COMMAND ${CMAKE_COMMAND}
		ARGS -E copy $<TARGET_FILE:${projectName}> ${PROJECT_BINARY_DIR}/applications/ndSandbox/${CMAKE_CFG_INTDIR}/$<TARGET_FILE_NAME:${projectName}>)

	add_custom_command(
		TARGET ${projectName} POST_BUILD
		COMMAND ${CMAKE_COMMAND}
		ARGS -E copy $ENV{ONEAPI_ROOT}/compiler/latest/windows/bin/sycl6.dll ${PROJECT_BINARY_DIR}/applications/ndSandbox/${CMAKE_CFG_INTDIR}/sycl6.dll)

	add_custom_command(
		TARGET ${projectName} POST_BUILD
		COMMAND ${CMAKE_COMMAND}
		ARGS -E copy $ENV{ONEAPI_ROOT}/compiler/latest/windows/bin/sycl6d.dll ${PROJECT_BINARY_DIR}/applications/ndSandbox/${CMAKE_CFG_INTDIR}/sycl6d.dll)
endif ()

#if (NEWTON_BUILD_TEST)
#	add_custom_command(
#		TARGET ${projectName} POST_BUILD
#		COMMAND ${CMAKE_COMMAND}
#		ARGS -E copy $<TARGET_FILE:${projectName}> ${PROJECT_BINARY_DIR}/../../../../applications/ndTest/${CMAKE_CFG_INTDIR}/$<TARGET_FILE_NAME:${projectName}>)
#endif ()