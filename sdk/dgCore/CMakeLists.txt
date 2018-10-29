# Copyright (c) <2014-2017> <Newton Game Dynamics>
#
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

cmake_minimum_required(VERSION 3.10.0)

set (projectName "dgCore")
message (${projectName})

# low level core
file(GLOB CPP_SOURCE *.cpp)
file(GLOB HEADERS *.h)

if (UNIX)
	if(NEWTON_BUILD_SHARED_LIBS)
	   add_library(${projectName} SHARED ${CPP_SOURCE})
	else(NEWTON_BUILD_SHARED_LIBS)
	   add_library(${projectName} STATIC ${CPP_SOURCE})
	endif(NEWTON_BUILD_SHARED_LIBS)

        target_include_directories(
                      ${projectName} PUBLIC
                      # Headers used from source/build location:
                      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
        )
endif(UNIX)

if (MSVC)
	   add_library(${projectName} STATIC ${CPP_SOURCE})
       set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YudgStdAfx.h")
       set_source_files_properties(dgTypes.cpp PROPERTIES COMPILE_FLAGS "/YcdgStdAfx.h")
endif(MSVC)

install(TARGETS ${projectName}
       LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
       ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
       RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(FILES ${HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${projectName})
