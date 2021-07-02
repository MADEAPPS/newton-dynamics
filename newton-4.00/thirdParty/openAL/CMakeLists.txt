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

set (projectName "openal")
message (${projectName})

if (MSVC)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4018 /wd4244 /wd4389 /wd4458 /wd4505")
endif()

add_definitions(-DAL_LIBTYPE_STATIC)
add_definitions(-DAL_ALEXT_PROTOTYPES)
add_definitions(-D_CRT_SECURE_NO_WARNINGS)
add_definitions(-D_CRT_NONSTDC_NO_DEPRECATE)
add_definitions(-D_WIN32_WINNT=0x0501)
add_definitions(-Dstrcasecmp=_stricmp)
add_definitions(-Dstrncasecmp=_strnicmp)
#add_definitions(-Dsnprintf=_snprintf)


include_directories(./)
include_directories(include/)
include_directories(OpenAL32/include/)

file(GLOB HEADERS 
	config.h
	include/AL/*.h
	OpenAL32/include/*.h)

file(GLOB CPP_SOURCE 
	Alc/*.c 
	Alc/backends/winmm.c
	Alc/backends/wave.c
	Alc/backends/null.c
	Alc/backends/loopback.c
	Alc/backends/mmdevapi.c
	OpenAL32/*.c 
	config.h
	include/AL/*.h
	OpenAL32/include/*.h)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

add_library(${projectName} STATIC ${CPP_SOURCE})

