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

set (projectName "ndSandbox")
message (${projectName})

# source and header files
file(GLOB CPP_SOURCE 
	*.h
	*.cpp
	demos/*.h
	demos/*.cpp
	toolbox/*.h
	toolbox/*.cpp 
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

include_directories(.)
include_directories(toolbox)

include_directories(../../sdk/dCore)
include_directories(../../sdk/dModel)
include_directories(../../sdk/dNewton)
include_directories(../../sdk/dCollision)
include_directories(../../sdk/dFileFormat)

include_directories(../../sdk/dBrain)

include_directories(../../thirdParty/png/)
include_directories(../../thirdParty/glfw/)
include_directories(../../thirdParty/imgui/)
include_directories(../../thirdParty/openFBX/src/)
include_directories(../../thirdParty/glfw/include/)
include_directories(../../thirdParty/imgui/backends)
include_directories(../../thirdParty/glatter/include/glatter/)
include_directories(../../thirdParty/hacd/src/VHACD_Lib/public/)
include_directories(../../thirdParty/openalRuntime/win32sdk/include)

if(NEWTON_ENABLE_VULKAN_SDK)
	include_directories ($ENV{VULKAN_SDK}/Include)
endif()

add_executable(${projectName} ${CPP_SOURCE})
target_link_libraries (${projectName} ndNewton ndBrain ndModel vhacd openfbx lodepng glfw imgui glatter)

if (NEWTON_BUILD_CREATE_SUB_PROJECTS)
	target_link_libraries (${projectName} ndCollision ndCore)
endif()
 
if (NEWTON_BUILD_PROFILER)
	target_link_libraries (${projectName} dProfiler)
endif ()


if(MSVC OR MINGW)
	if(NEWTON_ENABLE_VULKAN_SDK)
		target_link_libraries (${projectName} $ENV{VULKAN_SDK}/lib/vulkan-1.lib)
	endif(NEWTON_ENABLE_VULKAN_SDK)

	if(NEWTON_ENABLE_AVX2_SOLVER)
		if (NOT NEWTON_BUILD_SHARED_LIBS)
			target_link_libraries (${projectName} ndSolverAvx2)
		endif()
	endif(NEWTON_ENABLE_AVX2_SOLVER)

	if (NEWTON_ENABLE_CUDA_SOLVER)
		if (NOT NEWTON_BUILD_SHARED_LIBS)
			target_link_libraries (${projectName} ndSolverCuda)
		endif()
	endif(NEWTON_ENABLE_CUDA_SOLVER)

	if (PTR_SIZE EQUAL 8)
		target_link_libraries (${projectName} ${CMAKE_SOURCE_DIR}/thirdParty/openalRuntime/win64sdk/lib/OpenAL_32.lib)
		add_custom_command(
			TARGET ${projectName} POST_BUILD
			COMMAND ${CMAKE_COMMAND}
			ARGS -E copy ${CMAKE_SOURCE_DIR}/thirdParty/openalRuntime/win64sdk/bin/OpenAL_32.dll ${PROJECT_BINARY_DIR}/applications/ndSandbox/${CMAKE_CFG_INTDIR})
	else()
		target_link_libraries (${projectName} ${CMAKE_SOURCE_DIR}/thirdParty/openalRuntime/win32sdk/lib/OpenAL_32.lib)
		add_custom_command(
			TARGET ${projectName} POST_BUILD
			COMMAND ${CMAKE_COMMAND}
			ARGS -E copy ${CMAKE_SOURCE_DIR}/thirdParty/openalRuntime/win32sdk/bin/OpenAL_32.dll ${PROJECT_BINARY_DIR}/applications/ndSandbox/${CMAKE_CFG_INTDIR})
	endif()

    target_link_libraries (${projectName} glu32 opengl32)

	target_link_options(${projectName} PUBLIC "/DEBUG") 

    if(CMAKE_VS_MSBUILD_COMMAND OR CMAKE_VS_DEVENV_COMMAND)
		set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YundSandboxStdafx.h")
		set_source_files_properties(ndSandboxStdafx.cpp PROPERTIES COMPILE_FLAGS "/YcndSandboxStdafx.h")
		set_source_files_properties(toolbox/ndFileBrowser.cpp PROPERTIES COMPILE_FLAGS "/Y-")
    endif()
endif()

if(UNIX)
	target_link_libraries (${projectName} GL GLU dl X11 pthread openal)

	if(NEWTON_ENABLE_VULKAN_SDK)
		target_link_libraries (${projectName} vulkan)
	endif(NEWTON_ENABLE_VULKAN_SDK)

	if(NEWTON_ENABLE_AVX2_SOLVER)
		target_link_libraries (${projectName} ndSolverAvx2)
	endif(NEWTON_ENABLE_AVX2_SOLVER)

	if (NEWTON_ENABLE_CUDA_SOLVER)
		target_link_libraries (${projectName} ndSolverCuda)
	endif(NEWTON_ENABLE_CUDA_SOLVER)

    set (source "${CMAKE_SOURCE_DIR}/applications/media")
    set (destination "${PROJECT_BINARY_DIR}/applications/media")
    add_custom_command(
        TARGET ${projectName} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E create_symlink ${source} ${destination}
        DEPENDS ${destination}
        COMMENT "symbolic link resources folder from ${source} => ${destination}")
endif()

#if(WIN32)
#    file(COPY ../media DESTINATION "${PROJECT_BINARY_DIR}/applications")
#endif()

install(DIRECTORY ../media DESTINATION "${PROJECT_BINARY_DIR}/applications")

if (MSVC)
	set_target_properties(${projectName} PROPERTIES FOLDER "demos")
endif()
