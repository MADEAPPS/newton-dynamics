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
include_directories(../../sdk/dCore/tinyxml)
include_directories(../../sdk/dNewton/dJoints)
include_directories(../../sdk/dNewton/dModels)
include_directories(../../sdk/dNewton/dIkSolver)
include_directories(../../sdk/dNewton/dModels/dVehicle)

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

# for now opencl only for windows, I need to figure out the path for linux systems
if(NEWTON_ENABLE_OPENCL_SDK AND MSVC)
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

    find_library(AppkitFramework Appkit)
    if (NOT AppkitFramework)
        message(FATAL_ERROR "Appkit not found")
    endif()
    
	find_library(CoreFoundationFramework CoreFoundation)
    if (NOT CoreFoundationFramework)
        message(FATAL_ERROR "CoreFoundation not found")
    endif()

	find_library(CoreGraphicsFramework CoreGraphics)
    if (NOT CoreGraphicsFramework)
        message(FATAL_ERROR "CoreGraphics not found")
    endif()
   
    find_library(GameControllerFramework GameController)
    if (NOT GameControllerFramework)
        message(FATAL_ERROR "GameController not found")
    endif()

	find_library(MetalFramework Metal)
    if (NOT MetalFramework)
        message(FATAL_ERROR "Metal not found")
    endif()

	find_library(MetalKitFramework MetalKit)
    if (NOT MetalKitFramework)
        message(FATAL_ERROR "MetalKit not found")
    endif()
    
    find_library(QuartzCoreFramework QuartzCore)
    if (NOT QuartzCoreFramework)
        message(FATAL_ERROR "QuartzCore not found")
    endif()

    target_link_libraries(${projectName} PRIVATE ${MetalFramework})
	target_link_libraries(${projectName} PRIVATE ${AppkitFramework})
    target_link_libraries(${projectName} PRIVATE ${MetalKitFramework})
    target_link_libraries(${projectName} PRIVATE ${QuartzCoreFramework})
	target_link_libraries(${projectName} PRIVATE ${CoreGraphicsFramework})
	target_link_libraries(${projectName} PRIVATE ${CoreFoundationFramework})
    target_link_libraries(${projectName} PRIVATE ${GameControllerFramework})

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

	target_link_options(${projectName} PUBLIC "/DEBUG") 

    if(CMAKE_VS_MSBUILD_COMMAND OR CMAKE_VS_DEVENV_COMMAND)
		set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YundSandboxStdafx.h")
		set_source_files_properties(ndSandboxStdafx.cpp PROPERTIES COMPILE_FLAGS "/YcndSandboxStdafx.h")
		set_source_files_properties(toolbox/ndFileBrowser.cpp PROPERTIES COMPILE_FLAGS "/Y-")
    endif()
endif()

if(UNIX)
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
