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

if (NEWTON_BUILD_PROFILER)
	add_subdirectory(dProfiler)
endif()

if (NEWTON_BUILD_CREATE_SUB_PROJECTS)
	add_subdirectory(dCore)
	add_subdirectory(dNewton)
	add_subdirectory(dTinyxml)
	add_subdirectory(dCollision)

else()

	set (projectName "ndNewton")
	message (${projectName})

	include_directories(dCore/)
	include_directories(dNewton/)
	include_directories(dTinyxml/)
	include_directories(dCollision/)
	include_directories(dNewton/dJoints/)
	include_directories(dNewton/dModels/)
	include_directories(dNewton/dModels/dVehicle)

	if (NEWTON_ENABLE_GPU_SOLVER)
		add_definitions(-D_D_NEWTON_OPENCL)
		add_definitions(-DCL_TARGET_OPENCL_VERSION=120)
		add_definitions(-DCL_KERNEL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/dOpencl/")
		include_directories(dOpencl/)

		if(NEWTON_BUILD_SHARED_LIBS STREQUAL "ON")
			if (BUILD_64)
				#link_libraries ("$(NEWTON_OPENCL_RUNTIME)/lib/x64/OpenCL.lib")
				link_libraries ("${CMAKE_CURRENT_SOURCE_DIR}/dOpencl/CL/lib/x64/OpenCL.lib")
			else(BUILD_64)
				#link_libraries ("$(NEWTON_OPENCL_RUNTIME)/lib/x86/OpenCL.lib")
				link_libraries ("${CMAKE_CURRENT_SOURCE_DIR}/dOpencl/CL/lib/x86/OpenCL.lib")
			endif(BUILD_64)
		endif ()
	endif(NEWTON_ENABLE_GPU_SOLVER)

	# source and header files
	file(GLOB HEADERS dTinyxml/*.h dCore/*.h dNewton/*.h dCollision/*.h dNewton/dJoints/*.h dNewton/dModels/*.h dNewton/dModels/dVehicle/*.h)
	file(GLOB CPP_SOURCE 
		dTinyxml/*.h
		dTinyxml/*.cpp
		dCore/*.h 
		dCore/*.cpp
		dNewton/*.h 
		dNewton/*.cpp
		dCollision/*.h 
		dCollision/*.cpp
		dNewton/dModels/*.h 
		dNewton/dModels/*.cpp
		dNewton/dJoints/*.h 
		dNewton/dJoints/*.cpp
		dNewton/dModels/dVehicle/*.h 
		dNewton/dModels/dVehicle/*.cpp)

	if(MSVC)
		 set_source_files_properties(dNewton/ndDynamicsUpdateAvx2.cpp PROPERTIES COMPILE_FLAGS " /arch:AVX2 " )
	endif(MSVC)

	if(UNIX)
		set_source_files_properties(dNewton/ndDynamicsUpdateAvx2.cpp PROPERTIES COMPILE_FLAGS " -march=haswell " )
	endif(UNIX)

	source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

	add_definitions(-D_D_SINGLE_LIBRARY)

	if(NEWTON_BUILD_SHARED_LIBS STREQUAL "ON")
		add_definitions(-D_D_CORE_EXPORT_DLL)
		add_definitions(-D_D_COLLISION_EXPORT_DLL)
		add_definitions(-D_D_NEWTON_EXPORT_DLL)
		add_library(${projectName} SHARED ${CPP_SOURCE})
	else()
		add_library(${projectName} STATIC ${CPP_SOURCE})
	endif()
    
	if (MSVC)
		if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
		#if(CMAKE_VS_MSBUILD_COMMAND OR CMAKE_VS_DEVENV_COMMAND)
			set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YudCoreStdAfx.h")
			set_source_files_properties(dCore/dCoreStdAfx.cpp PROPERTIES COMPILE_FLAGS "/YcdCoreStdAfx.h")
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

	target_include_directories(${projectName} PUBLIC .)
	if (NEWTON_BUILD_PROFILER)
		target_link_libraries (${projectName} dProfiler)
	endif()

	install(TARGETS ${projectName} 
			LIBRARY DESTINATION lib
			ARCHIVE DESTINATION lib
			RUNTIME DESTINATION bin)

	install(FILES ${HEADERS} DESTINATION include/${projectName})

endif()

	
