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

set (projectName "dNewton")
message (${projectName})

# low level core
file(GLOB CPP_SOURCE *.cpp)
file(GLOB HEADERS *.h)

if (GENERATE_DLL)
	add_definitions(-D_DCONTAINERS_DLL)
	add_definitions(-D_CNEWTON_BUILD_DLL)
    add_library(${projectName} SHARED ${CPP_SOURCE})
	target_link_libraries (${projectName} newton dContainers dMath dCustomJoints dTimeTracker)
	install(TARGETS ${projectName} RUNTIME DESTINATION ${dllPath})
	install(TARGETS ${projectName} ARCHIVE DESTINATION ${libraryPath})
	if (BUILD_SANDBOX_DEMOS)
		add_custom_command(
			TARGET ${projectName} POST_BUILD
			COMMAND ${CMAKE_COMMAND}
			ARGS -E copy $(TargetPath) ${PROJECT_BINARY_DIR}/applications/demosSandbox/${CMAKE_CFG_INTDIR}/$(TargetFileName))
	endif ()
else()
    if (UNIX)
        if(NEWTON_BUILD_SHARED_LIBS)
            add_library(${projectName} SHARED ${CPP_SOURCE})
        else(NEWTON_BUILD_SHARED_LIBS)
            add_library(${projectName} STATIC ${CPP_SOURCE})
        endif(NEWTON_BUILD_SHARED_LIBS)

        install(TARGETS ${projectName} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
        install(FILES ${HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

        target_link_libraries (${projectName} newton dContainers dMath dCustomJoints)

        target_include_directories(
                ${projectName} PUBLIC
                # Headers used from source/build location:
                "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
            )
    endif (UNIX)
    
    if (MSVC)
        add_definitions(-D_NEWTON_STATIC_LIB)
        add_definitions(-D_CNEWTON_STATIC_LIB)
        add_definitions(-D_CUSTOM_JOINTS_STATIC_LIB)
        add_library(${projectName} STATIC ${CPP_SOURCE})
		install(TARGETS ${projectName} ARCHIVE DESTINATION ${libraryPath})
     endif(MSVC)
endif()

if (MSVC)
	set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YudStdAfxNewton.h")
	set_source_files_properties(dStdAfxNewton.cpp PROPERTIES COMPILE_FLAGS "/YcdStdAfxNewton.h")
endif(MSVC)

# Install library
#install(TARGETS ${projectName} DESTINATION lib/${projectName})
# Install library headers

install(FILES ${HEADERS} DESTINATION "${includePath}/${projectName}")
#install(FILES ${HEADERS} DESTINATION include/${projectName})
