# Copyright (c) <2014-2017> <Newton Game Dynamics>
#
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

cmake_minimum_required(VERSION 2.8.8)


 # low level core
include_directories(../dProfiler/dTimeTracker/)
file(GLOB dgCore_srcs *.cpp *.h)

add_library(dgCore STATIC ${dgCore_srcs})



if (MSVC)
	if (CONFIG_STATIC_SYSTEM_LIBRARIES)
	set_target_properties(dgCore PROPERTIES COMPILE_FLAGS "/YudgStdAfx.h")

	endif(CONFIG_STATIC_SYSTEM_LIBRARIES)
	set_target_properties(dgCore PROPERTIES COMPILE_FLAGS "/YudgStdAfx.h")
	set_source_files_properties(dgTypes.cpp PROPERTIES COMPILE_FLAGS "/YcdgStdAfx.h")
endif(MSVC)