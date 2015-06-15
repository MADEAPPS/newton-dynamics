# Copyright (c) <2009> <Newton Game Dynamics>
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely

set (projectName dgCore)
include_directories ("${corePath}/core") 
include_directories ("${corePath}/pthreads.2") 

file(GLOB_RECURSE headers *.h)
file(GLOB_RECURSE sources *.cpp)
add_library(${projectName} ${headers} ${sources})
target_link_libraries (${projectName} ${corePath}/pthreads.2/dgThreads)

if (MSVC)
	set_target_properties (${projectName} PROPERTIES COMPILE_FLAGS "/YudgStdafx.h")
	set_source_files_properties(dgTypes.cpp PROPERTIES COMPILE_FLAGS "/YcdgStdafx.h")
	set_target_properties (${projectName} PROPERTIES COMPILE_DEFINITIONS "_NEWTON_STATIC_LIB;_WIN_32_VER;PTW32_BUILD;PTW32_STATIC_LIB;_CRT_SECURE_NO_WARNINGS")
endif(MSVC)
