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

set (projectName "dgNewtonVulkan")
message (${projectName})

# low level core
file(GLOB source *.cpp *.h)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /fp:fast")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /fp:fast")

if(!CMAKE_CL_64)
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /arch:SSE2")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /arch:SSE2")
endif ()

add_definitions(-DNEWTONCPU_EXPORTS)
add_library(${projectName} SHARED ${source})
#target_link_libraries (${projectName} dProfiler mincore.lib d3d12.lib dxgi.lib dxguid.lib)
target_include_directories(${projectName} PUBLIC ../dgCore ../dgPhysics)
if (NEWTON_BUILD_PROFILER)
		target_link_libraries (${projectName} dProfiler)
endif()

if (MSVC)
	if(CMAKE_VS_MSBUILD_COMMAND OR CMAKE_VS_DEVENV_COMMAND)
		set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YudgNewtonPluginStdafx.h")
		set_source_files_properties(dgNewtonPluginStdafx.cpp PROPERTIES COMPILE_FLAGS "/YcdgNewtonPluginStdafx.h")
	endif()
endif(MSVC)


#install(TARGETS ${projectName} RUNTIME DESTINATION ${dllPath})
#install(TARGETS ${projectName} ARCHIVE DESTINATION ${libraryPath})

add_custom_command(
	TARGET ${projectName} POST_BUILD
	COMMAND ${CMAKE_COMMAND}
	ARGS -E copy $<TARGET_FILE:${projectName}> ${PROJECT_BINARY_DIR}/applications/demosSandbox/${CMAKE_CFG_INTDIR}/$<TARGET_FILE_NAME:${projectName}>)


if (NEWTON_BUILD_SANDBOX_DEMOS)
	add_custom_command(
		TARGET ${projectName} POST_BUILD
		COMMAND ${CMAKE_COMMAND}
		ARGS -E copy $<TARGET_FILE:${projectName}> ${PROJECT_BINARY_DIR}/applications/demosSandbox/${CMAKE_CFG_INTDIR}/newtonPlugins/${CMAKE_CFG_INTDIR}/$<TARGET_FILE_NAME:${projectName}>)
endif ()
