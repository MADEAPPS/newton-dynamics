# Copyright (c) <2014-2017> <Newton Game Dynamics>
#
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR)

set (projectName "glatter")
message (${projectName})

# low level core
include_directories(include/)
file(GLOB source src/glatter/glatter.c)

add_library(${projectName} STATIC ${source})

#target_compile_options(${projectName} PUBLIC  $<$<COMPILE_LANGUAGE:CXX>:$<$<CXX_COMPILER_ID:MSVC>:/wd4201>>)

if (MSVC)
	set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /Wall /wd4018 /wd4054 /wd4055 /wd4100 /wd4204 /wd4244 /wd4255 /wd4389 /wd4458 /wd4505 /wd4710 /wd4711 /wd4820")
	set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Wall /wd4018 /wd4054 /wd4055 /wd4100 /wd4204 /wd4244 /wd4255 /wd4389 /wd4458 /wd4505 /wd4710 /wd4711 /wd4820")
	set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} /Wall /wd4018 /wd4054 /wd4055 /wd4100 /wd4204 /wd4244 /wd4255 /wd4389 /wd4458 /wd4710 /wd4711 /wd4505 /wd4820")

	set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /Od")
	set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} /Zi /O2 /fp:fast")
	set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Zi /GS- /Ox /Oi /Ot /Ob2 /Oy /fp:fast")
endif()

if (MSVC)
	set_target_properties(${projectName} PROPERTIES FOLDER "thirdParty")
endif()