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

if (NEWTON_USE_DEFAULT_NEW_AND_DELETE)
	add_definitions(-DD_USE_DEFAULT_NEW_AND_DELETE)
endif()

if (NEWTON_BUILD_PROFILER)
	add_subdirectory(dProfiler)
endif()

add_subdirectory(dModel)
add_subdirectory(dBrain)
add_subdirectory(dRender)

if (NEWTON_BUILD_CREATE_SUB_PROJECTS)
	add_subdirectory(dCore)
	add_subdirectory(dNewton)
	add_subdirectory(dCollision)
else()
	set (projectName "ndNewton")
	message (${projectName})

	include_directories(dCore)
	include_directories(dNewton)
	include_directories(dCollision)
	include_directories(dCore/tinyxml)
	include_directories(dNewton/dJoints)
	include_directories(dNewton/dModels)
	include_directories(dNewton/dIkSolver)
	include_directories(dNewton/dModels/dVehicle)

	add_subdirectory(dNewton/dExtensions)

	file(GLOB CPP_SOURCE 
		dCore/*.h 
		dCore/*.cpp
		dNewton/*.h 
		dNewton/*.cpp
		dCollision/*.h 
		dCollision/*.cpp
		dCore/tinyxml/*.h
		dCore/tinyxml/*.cpp 
		dNewton/dModels/*.h 
		dNewton/dModels/*.cpp
		dNewton/dJoints/*.h 
		dNewton/dJoints/*.cpp
		dNewton/dIkSolver/*.h
		dNewton/dIkSolver/*.cpp
		dNewton/dModels/dVehicle/*.h 
		dNewton/dModels/dVehicle/*.cpp)

	if (NEWTON_ENABLE_CUDA_SOLVER)
		add_definitions(-D_D_NEWTON_CUDA)
		include_directories(dNewton/dExtensions/dCuda)
		include_directories(dNewton/dExtensions/dCuda/dContext)

		file(GLOB CPP_SOURCE 
			${CPP_SOURCE} 
			dNewton/dExtensions/dCuda/*.h
			dNewton/dExtensions/dCuda/*.cpp
			dNewton/dExtensions/dCuda/dContext/*.h) 
	endif()

	if(MSVC)
		file(GLOB CPP_SOURCE ${CPP_SOURCE} dCore/ndTypes.natvis)
	endif()

	source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

	add_definitions(-D_D_SINGLE_LIBRARY)

	if(NEWTON_BUILD_SHARED_LIBS)
		add_definitions(-D_D_CORE_EXPORT_DLL)
		add_definitions(-D_D_NEWTON_EXPORT_DLL)
		add_definitions(-D_D_TINYXML_EXPORT_DLL)
		add_definitions(-D_D_COLLISION_EXPORT_DLL)
		add_library(${projectName} SHARED ${CPP_SOURCE})

		if (NEWTON_ENABLE_CUDA_SOLVER)
			target_link_libraries (${projectName} ndSolverCuda)
		endif()

		if(MSVC OR MINGW)
			target_link_options(${projectName} PUBLIC "/DEBUG") 
		endif()

	else()
		add_library(${projectName} STATIC ${CPP_SOURCE})
	endif()
    
	if (MSVC)
		set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YundCoreStdafx.h")
		set_source_files_properties(dCore/ndCoreStdAfx.cpp PROPERTIES COMPILE_FLAGS "/YcndCoreStdafx.h")

		if (NEWTON_BUILD_SHARED_LIBS)
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

	install(FILES ${CPP_SOURCE} DESTINATION "install/source")
	install(TARGETS ${projectName} 
			LIBRARY DESTINATION "install/lib" 
			ARCHIVE DESTINATION "install/lib" 
			RUNTIME DESTINATION "install/bin")
	set_target_properties(${projectName} PROPERTIES FOLDER "newtonSdk")

	if (MSVC)
		#install(FILES ${CPP_SOURCE} DESTINATION source/)
		#install(TARGETS ${projectName} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
		#set_target_properties(${projectName} PROPERTIES FOLDER "newtonSdk")
	endif()
endif()
