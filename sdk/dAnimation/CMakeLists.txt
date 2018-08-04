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

set (projectName "dAnimation")
message (${projectName})


 # low level core
file(GLOB source *.cpp *.h)

add_library(${projectName} STATIC ${source})

if (MSVC)
	set_target_properties(${projectName} PROPERTIES COMPILE_FLAGS "/YudAnimationStdAfx.h")
	set_source_files_properties(dAnimationStdAfx.cpp PROPERTIES COMPILE_FLAGS "/YcdAnimationStdAfx.h")
endif(MSVC)