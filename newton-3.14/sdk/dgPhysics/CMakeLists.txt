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

set (projectName "dgPhysics")
message (${projectName})

if(NEWTON_USE_PLUGIN_DLL)
	add_definitions(-DDG_USE_PLUGINS)
endif()

#source and header files
file(GLOB CPP_SOURCE 
	*.h
	*.cpp 
	../dgMeshUtil/*.h 
	../dgMeshUtil/*.cpp
)
#source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

if (MSVC)
       set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /fp:fast")
       set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /fp:fast")
endif(MSVC)

if(NEWTON_BUILD_SHARED_LIBS)
	if(MSVC)
		add_library(${projectName} STATIC ${CPP_SOURCE})
	else()
		add_library(${projectName} SHARED ${CPP_SOURCE})
	endif()
else()
	add_library(${projectName} STATIC ${CPP_SOURCE})
endif()

if (MSVC)
   if(CMAKE_VS_MSBUILD_COMMAND OR CMAKE_VS_DEVENV_COMMAND)
	   set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YudgPhysicsStdafx.h")
      set_source_files_properties(dgWorld.cpp PROPERTIES COMPILE_FLAGS "/YcdgPhysicsStdafx.h")
   endif()
endif(MSVC)

target_include_directories(${projectName} PUBLIC . ../dgMeshUtil)
target_link_libraries(${projectName} dgCore)

install(TARGETS ${projectName} 
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib
		RUNTIME DESTINATION bin)

install(FILES ${HEADERS} DESTINATION include/${projectName})

