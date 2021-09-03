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

set (projectName "ndNewton")
message (${projectName})

if(NEWTON_USE_PLUGIN_DLL)
	add_definitions(-DDG_USE_PLUGINS)
endif()

#source and header files
file(GLOB CPP_SOURCE 
	*.h 
	*.cpp 
	dModels/*.h	
	dModels/*.cpp
	dJoints/*.h
	dJoints/*.cpp
	dModels/dVehicle*.h	
	dModels/dVehicle/*.cpp
	dModels/dCharacter/*.h
	dModels/dCharacter/*.cpp)

file(GLOB HEADERS *.h dJoints/*.h)
file(GLOB HEADERS *.h dModels/*.h)
file(GLOB HEADERS *.h dModels/dVehicle/*.h)
file(GLOB HEADERS *.h dModels/dCharacter/*.h)

# exclude AVX files when disabled
#if(NOT NEWTON_ENABLE_AVX)
#	list(FILTER HEADERS EXCLUDE REGEX "(Avx)")
#	list(FILTER CPP_SOURCE EXCLUDE REGEX "(Avx)")
#endif()

if(MSVC AND NEWTON_ENABLE_AVX2_SOLVER)
	set_source_files_properties(dNewton/ndDynamicsUpdateAvx2.cpp PROPERTIES COMPILE_FLAGS " /arch:AVX2 " )
endif()

if(MINGW AND NEWTON_ENABLE_AVX2_SOLVER)
	set_source_files_properties(dNewton/ndDynamicsUpdateAvx2.cpp PROPERTIES COMPILE_FLAGS " -mavx2 -mfma  " )
endif()

if(UNIX AND NEWTON_ENABLE_AVX2_SOLVER)
	set_source_files_properties(dNewton/ndDynamicsUpdateAvx2.cpp PROPERTIES COMPILE_FLAGS " -march=haswell " )
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

include_directories(./)
include_directories(dJoints/)
include_directories(dModels/)
include_directories(dModels/dVehicle/)
include_directories(../dCore/)
include_directories(../dTinyxml/)
include_directories(../dCollision/)

if (NEWTON_ENABLE_GPU_SOLVER)
	add_definitions(-D_D_NEWTON_OPENCL)
	add_definitions(-DCL_TARGET_OPENCL_VERSION=120)
	add_definitions(-DCL_KERNEL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../dOpencl/")
	include_directories(../dOpencl/)

	if(NEWTON_BUILD_SHARED_LIBS)
		if(BUILD_64)
			link_libraries ("${CMAKE_CURRENT_SOURCE_DIR}/../dOpencl/CL/lib/x64/OpenCL.lib")
		else(BUILD_64)
			link_libraries ("${CMAKE_CURRENT_SOURCE_DIR}/../dOpencl/CL/lib/x86/OpenCL.lib")
		endif(BUILD_64)
	endif()
endif(NEWTON_ENABLE_GPU_SOLVER)

if (MSVC)
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /fp:fast")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /fp:fast")
	set_source_files_properties(./ndDynamicsUpdateAvx2.cpp PROPERTIES COMPILE_FLAGS " /arch:AVX2 " )
endif(MSVC)

if(UNIX)
	set_source_files_properties(./ndDynamicsUpdateAvx2.cpp PROPERTIES COMPILE_FLAGS " -march=haswell " )
endif(UNIX)

if(NEWTON_BUILD_SHARED_LIBS)
	add_definitions(-D_D_NEWTON_EXPORT_DLL)
	add_library(${projectName} SHARED ${CPP_SOURCE})

	if(MSVC OR MINGW)
		target_link_options(${projectName} PUBLIC "/DEBUG") 
	endif()

else()
	add_library(${projectName} STATIC ${CPP_SOURCE})
endif()

if (MSVC)
	if(CMAKE_VS_MSBUILD_COMMAND OR CMAKE_VS_DEVENV_COMMAND)
		set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YundNewtonStdafx.h")
		set_source_files_properties(ndNewtonStdafx.cpp PROPERTIES COMPILE_FLAGS "/YcndNewtonStdafx.h")
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

target_link_libraries(${projectName} ndTinyxml ndCore ndCollision)

install(TARGETS ${projectName} 
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib
		RUNTIME DESTINATION bin)

install(FILES ${HEADERS} DESTINATION include/${projectName})

