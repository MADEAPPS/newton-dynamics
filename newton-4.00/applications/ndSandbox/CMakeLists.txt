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
if (APPLE) 
	file(GLOB CPP_SOURCE 
		*.h
		*.cpp
		demos/*.h
		demos/*.cpp
		toolbox/*.h
		toolbox/*.cpp 
		plaforms/macos/*.mm 	
	)
else()
	file(GLOB CPP_SOURCE 
		*.h
		*.cpp
		demos/*.h
		demos/*.cpp
		toolbox/*.h
		toolbox/*.cpp 
		plaforms/opengl/*.cpp 
	)
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

include_directories(.)
include_directories(toolbox)

include_directories(../../sdk/dCore)
include_directories(../../sdk/dModel)
include_directories(../../sdk/dNewton)
include_directories(../../sdk/dCollision)
include_directories(../../sdk/dFileFormat)

include_directories(../../sdk/dBrain)
include_directories(../../sdk/dRender)

include_directories(../../thirdParty/png/)
include_directories(../../thirdParty/imgui/)
include_directories(../../thirdParty/openFBX/src/)
include_directories(../../thirdParty/imgui/backends)
include_directories(../../thirdParty/hacd/src/VHACD_Lib/public/)

if (APPLE) 
	message ("apple headers here")	
else()
	include_directories(../../thirdParty/glfw/)
	include_directories(../../thirdParty/glfw/include/)
	include_directories(../../thirdParty/glatter/include/glatter/)
endif()

set (openclSDKPath ${CMAKE_SOURCE_DIR}/thirdParty/opencl/OpenCL-SDK-v2024.10.24-Win-x64)

if(NEWTON_ENABLE_OPENCL_SDK)
	include_directories(${openclSDKPath}/include)
	include_directories(../../sdk/dBrain/dOpencl)
else()
	include_directories(../../sdk/dBrain/cpuEmulation)
endif()

add_executable(${projectName} ${CPP_SOURCE})

message ("adding project libraries")
target_link_libraries (${projectName} PUBLIC ndRender ndNewton ndBrain ndModel vhacd openfbx lodepng imgui)

if (MSVC OR MINGW)
	message ("adding required third party opengl libraries")
	target_link_libraries (${projectName} PUBLIC glfw glatter glu32 opengl32)

elseif(APPLE)
	message ("adding required Apple frameworks")
	#target_link_libraries(${projectName} PUBLIC "-framework Metal")
	#target_link_libraries(${projectName} PUBLIC "-framework, CoreFoundation")

	find_library(CoreFoundationFramework CoreFoundation)
    if (NOT CoreFoundationFramework)
        message(FATAL_ERROR "CoreFoundation not found")
    endif()
    target_link_libraries(${projectName} PRIVATE ${CoreFoundationFramework})

	find_library(CoreGraphicsFramework CoreGraphics)
    if (NOT CoreGraphicsFramework)
        message(FATAL_ERROR "CoreGraphics not found")
    endif()
    target_link_libraries(${projectName} PRIVATE ${CoreGraphicsFramework})

	find_library(MetalFramework Metal)
    if (NOT MetalFramework)
        message(FATAL_ERROR "Metal not found")
    endif()
    target_link_libraries(${projectName} PRIVATE ${MetalFramework})

		find_library(MetalKitFramework MetalKit)
    if (NOT MetalKitFramework)
        message(FATAL_ERROR "MetalKit not found")
    endif()
    target_link_libraries(${projectName} PRIVATE ${MetalKitFramework})

else(UNIX)
	message ("adding required posit system libraries")
	target_link_libraries (${projectName} PUBLIC glatter glfw dl GL X11 pthread ${X11_Xcursor_LIBRARIES} ${X11_Xinerama_LIBRARIES})
endif()

if (NEWTON_BUILD_CREATE_SUB_PROJECTS)
	target_link_libraries (${projectName} PUBLIC ndCollision ndCore)
endif()
 
if (NEWTON_BUILD_PROFILER)
	target_link_libraries (${projectName} PUBLIC dProfiler)
endif ()

if(MSVC OR MINGW)
	if(NEWTON_ENABLE_OPENCL_SDK)
		target_link_libraries (${projectName} PUBLIC ${openclSDKPath}/lib/OpenCL.lib)
	endif()

	if(NEWTON_ENABLE_AVX2_SOLVER)
		if (NOT NEWTON_BUILD_SHARED_LIBS)
			target_link_libraries (${projectName} PUBLIC ndSolverAvx2)
		endif()
	endif(NEWTON_ENABLE_AVX2_SOLVER)

	if (NEWTON_ENABLE_CUDA_SOLVER)
		if (NOT NEWTON_BUILD_SHARED_LIBS)
			target_link_libraries (${projectName} PUBLIC ndSolverCuda)
		endif()
	endif(NEWTON_ENABLE_CUDA_SOLVER)
	    

	target_link_options(${projectName} PUBLIC "/DEBUG") 

    if(CMAKE_VS_MSBUILD_COMMAND OR CMAKE_VS_DEVENV_COMMAND)
		set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YundSandboxStdafx.h")
		set_source_files_properties(ndSandboxStdafx.cpp PROPERTIES COMPILE_FLAGS "/YcndSandboxStdafx.h")
		set_source_files_properties(toolbox/ndFileBrowser.cpp PROPERTIES COMPILE_FLAGS "/Y-")
    endif()
endif()

if(UNIX)
	if(NEWTON_ENABLE_AVX2_SOLVER)
		target_link_libraries (${projectName} PUBLIC ndSolverAvx2)
	endif(NEWTON_ENABLE_AVX2_SOLVER)

	if (NEWTON_ENABLE_CUDA_SOLVER)
		target_link_libraries (${projectName} PUBLIC ndSolverCuda)
	endif(NEWTON_ENABLE_CUDA_SOLVER)

    set (source "${CMAKE_SOURCE_DIR}/applications/media")
    set (destination "${PROJECT_BINARY_DIR}/applications/media")

    add_custom_command(
        TARGET ${projectName} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E create_symlink ${source} ${destination}
        COMMENT "symbolic link resources folder from ${source} => ${destination}")
endif()

install(DIRECTORY ../media DESTINATION "${PROJECT_BINARY_DIR}/applications")

if (MSVC)
	set_target_properties(${projectName} PROPERTIES FOLDER "demos")
endif()
