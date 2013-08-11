# Copyright (c) <2009> <Newton Game Dynamics>
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely

set (projectName dgPhysics)
include_directories ("${path}/core") 
include_directories ("${path}/physics") 
include_directories ("${path}/meshUtil") 
include_directories ("${path}/pthreads.2") 


file(GLOB_RECURSE headers *.h ${path}/meshUtil/*.h)
file(GLOB_RECURSE sources *.cpp ${path}/meshUtil/*.cpp)
#file(GLOB_RECURSE headers ${path}/openCL/*.h)
#file(GLOB_RECURSE sources ${path}/openCL/*.cpp)
add_library(${projectName} ${headers} ${sources})

if (MSVC)
	set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YudgPhysicsStdafx.h")
	set_source_files_properties(dgWorld.cpp PROPERTIES COMPILE_FLAGS "/YcdgPhysicsStdafx.h")
	set_target_properties (${projectName} PROPERTIES COMPILE_DEFINITIONS "_WIN_32_VER;PTW32_BUILD;PTW32_STATIC_LIB;_CRT_SECURE_NO_WARNINGS")
endif(MSVC)
