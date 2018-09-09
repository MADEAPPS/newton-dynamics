# Copyright (c) <2014-2017> <Newton Game Dynamics>
#
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

cmake_minimum_required(VERSION 3.12.0)

set (projectName "dVehicle")
message (${projectName})

 # low level core
file(GLOB source *.cpp *.h)

if (GENERATE_DLL)
	add_definitions(-D_DCONTAINERS_DLL)
	add_definitions(-D_DVEHICLE_BUILD_DLL)
	add_library(${projectName} SHARED ${source})
	target_link_libraries (${projectName} newton dContainers dMath dCustomJoints dTimeTracker)
	install(TARGETS ${projectName} RUNTIME DESTINATION ${dllPath})
	if (BUILD_SANDBOX_DEMOS)
		add_custom_command(
			TARGET ${projectName} POST_BUILD
			COMMAND ${CMAKE_COMMAND}
			ARGS -E copy $(TargetPath) ${PROJECT_BINARY_DIR}/applications/demosSandbox/${CMAKE_CFG_INTDIR}/$(TargetFileName))
	endif ()
else()
	add_definitions(-D_NEWTON_STATIC_LIB)
	add_definitions(-D_CNEWTON_STATIC_LIB)
	add_definitions(-D_DVEHICLE_STATIC_LIB)
	add_definitions(-D_CUSTOM_JOINTS_STATIC_LIB)
	add_library(${projectName} STATIC ${source})
endif()

if (MSVC)
	set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YudStdafxVehicle.h")
	set_source_files_properties(dStdafxVehicle.cpp PROPERTIES COMPILE_FLAGS "/YcdStdafxVehicle.h")
endif(MSVC)

file(GLOB includes *.h)
install(FILES ${includes} DESTINATION "${includePath}/${projectName}")

install(TARGETS ${projectName} ARCHIVE DESTINATION ${libraryPath})
