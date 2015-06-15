# Copyright (c) <2009> <Newton Game Dynamics>
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely

set (project newton)

file(GLOB_RECURSE headers *.h)
file(GLOB_RECURSE sources *.cpp)

add_library(${project} STATIC ${sources} ${headers})

include_directories ("${corePath}/core") 
include_directories ("${corePath}/newton") 
include_directories ("${corePath}/physics") 
include_directories ("${corePath}/meshUtil") 
include_directories ("${corePath}/pthreads.2") 

if (MSVC)
	set_target_properties (${project} PROPERTIES COMPILE_FLAGS "/YuNewtonStdAfx.h")
	set_source_files_properties(NewtonClass.cpp PROPERTIES COMPILE_FLAGS "/YcNewtonStdAfx.h")
	set_target_properties (${project} PROPERTIES COMPILE_DEFINITIONS "_NEWTON_STATIC_LIB;_WIN_32_VER;PTW32_BUILD;PTW32_STATIC_LIB;_CRT_SECURE_NO_WARNINGS")
endif(MSVC)
