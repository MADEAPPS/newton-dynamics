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

if (NEWTON_USE_DEFAULT_NEW_AND_DELETE)
	add_definitions(-DD_USE_DEFAULT_NEW_AND_DELETE)
endif()

if (NEWTON_BUILD_PROFILER)
	add_subdirectory(dProfiler)
endif()

add_subdirectory(dExtensions)

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
	include_directories(dNewton/dIkSolver)
	include_directories(dNewton/dParticles/)
	include_directories(dNewton/dModels/dVehicle)
	include_directories(dNewton/dModels/dCharacter)

	if(NEWTON_ENABLE_AVX2_SOLVER)
		add_definitions(-D_D_USE_AVX2_SOLVER)
		include_directories(dExtensions/dAvx2)
	endif()

	if (NEWTON_ENABLE_CUDA_SOLVER)
		add_definitions(-D_D_NEWTON_CUDA)
		include_directories(dExtensions/dCuda)

		#if(NEWTON_BUILD_SHARED_LIBS)
		#	if (PTR_SIZE EQUAL 8)
		#		link_libraries ("${CMAKE_CURRENT_SOURCE_DIR}/dExtensions/dOpencl/CL/lib/x64/OpenCL.lib")
		#	else()
		#		link_libraries ("${CMAKE_CURRENT_SOURCE_DIR}/dExtensions/dOpencl/CL/lib/x86/OpenCL.lib")
		#	endif()
		#endif()
	endif(NEWTON_ENABLE_CUDA_SOLVER)

	if (NEWTON_ENABLE_OPENCL_SOLVER)
		add_definitions(-D_D_NEWTON_OPENCL)
		include_directories(dExtensions/dOpencl)

		if(NEWTON_BUILD_SHARED_LIBS)
			if (PTR_SIZE EQUAL 8)
				link_libraries ("${CMAKE_CURRENT_SOURCE_DIR}/dExtensions/dOpencl/CL/lib/x64/OpenCL.lib")
			else()
				link_libraries ("${CMAKE_CURRENT_SOURCE_DIR}/dExtensions/dOpencl/CL/lib/x86/OpenCL.lib")
			endif()
		endif()
	endif(NEWTON_ENABLE_OPENCL_SOLVER)

	# source and header files
	file(GLOB HEADERS 
		dTinyxml/*.h 
		dCore/*.h 
		dNewton/*.h 
		dCollision/*.h 
		dNewton/dJoints/*.h 
		dNewton/dModels/*.h 
		dNewton/dIkSolver/*.h
		dNewton/dParticles/*.h 
		dNewton/dModels/dVehicle/*.h
		dNewton/dModels/dCharacter/*.h)

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
		dNewton/dIkSolver/*.h
		dNewton/dIkSolver/*.cpp
		dNewton/dParticles/*.h
		dNewton/dParticles/*.cpp
		dNewton/dModels/dVehicle/*.h 
		dNewton/dModels/dVehicle/*.cpp
		dNewton/dModels/dCharacter/*.h
		dNewton/dModels/dCharacter/*.cpp)

	source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

	add_definitions(-D_D_SINGLE_LIBRARY)

	if(NEWTON_BUILD_SHARED_LIBS)
		add_definitions(-D_D_CORE_EXPORT_DLL)
		add_definitions(-D_D_NEWTON_EXPORT_DLL)
		add_definitions(-D_D_TINYXML_EXPORT_DLL)
		add_definitions(-D_D_COLLISION_EXPORT_DLL)
		add_library(${projectName} SHARED ${CPP_SOURCE})

		if(NEWTON_ENABLE_AVX2_SOLVER)
			target_link_libraries (${projectName} ndSolverAvx2)
		endif()

		if (NEWTON_ENABLE_CUDA_SOLVER)
			target_link_libraries (${projectName} ndSolverCuda)
		endif()

		if (NEWTON_ENABLE_OPENCL_SOLVER)
			target_link_libraries (${projectName} ndSolverOpenCL)
		endif()

		if(MSVC OR MINGW)
			target_link_options(${projectName} PUBLIC "/DEBUG") 
		endif()

	else()
		add_library(${projectName} STATIC ${CPP_SOURCE})
	endif()
    
	if (MSVC)
		if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
		#if(CMAKE_VS_MSBUILD_COMMAND OR CMAKE_VS_DEVENV_COMMAND)
			set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YundCoreStdafx.h")
			set_source_files_properties(dCore/ndCoreStdAfx.cpp PROPERTIES COMPILE_FLAGS "/YcndCoreStdafx.h")
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

		if(NEWTON_BUILD_SHARED_LIBS)
			install(FILES $<TARGET_PDB_FILE:${projectName}> DESTINATION bin OPTIONAL)
		endif(NEWTON_BUILD_SHARED_LIBS)

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
