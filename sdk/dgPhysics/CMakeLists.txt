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

set (projectName "dgPhysics")
message (${projectName})

if (MSVC)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /fp:fast")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /fp:fast")
endif(MSVC)

# low level core
file(GLOB CPP_SOURCE *.cpp ../dgMeshUtil/*.cpp)
file(GLOB HEADERS *.h ../dgMeshUtil/*.h)

if (MSVC)
        add_library(${projectName} STATIC ${CPP_SOURCE})
	set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YudgPhysicsStdafx.h")
	set_source_files_properties(dgWorld.cpp PROPERTIES COMPILE_FLAGS "/YcdgPhysicsStdafx.h")
endif(MSVC)

if (UNIX)
    if(NEWTON_BUILD_SHARED_LIBS)
        add_library(${projectName} SHARED ${CPP_SOURCE})
    else(NEWTON_BUILD_SHARED_LIBS)
        add_library(${projectName} STATIC ${CPP_SOURCE})
    endif(NEWTON_BUILD_SHARED_LIBS)

    install(TARGETS ${projectName} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
    install(FILES ${HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

    target_include_directories(
            ${projectName} PUBLIC
            # Headers used from source/build location:
            "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
        )
endif (UNIX)

if (MSVC)
    if (NOT GENERATE_DLL)
            install(TARGETS ${projectName} ARCHIVE DESTINATION ${libraryPath})
    endif()
endif(MSVC)

install(FILES ${HEADERS} DESTINATION "${includePath}/${projectName}")
