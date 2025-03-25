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

set (projectName "ndSolverCuda")
message (${projectName})

set(CMAKE_CUDA_STANDARD 14)
CMAKE_POLICY(SET CMP0104 OLD)
project(${projectName} LANGUAGES CXX CUDA)
set(CMAKE_CUDA_COMPILER $ENV{CUDA_PATH}/bin/nvcc.exe)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 14)

include_directories(./)
include_directories(dCudaDriver/)

file(GLOB CPP_SOURCE dContext/*.h dContext/*.cpp dContext/*.cuh dContext/*.cu)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})


Set(CMAKE_CUDA_FLAGS_DEBUG "-Xcompiler=\"-MDd -W4 -wd4100 -wd4201 -wd4324 -wd4505 -Zi -Ob0 -Od /RTC1\" ")
Set(CMAKE_CUDA_FLAGS_RELEASE "-Xcompiler=\"-MD -W4 -wd4100 -wd4201 -wd4324 -wd4505 -Zi -O2 -Ob2\" -DNDEBUG") 
Set(CMAKE_CUDA_FLAGS_RELWITHDEBINFO "-Xcompiler=\"-MD -W4 -wd4100 -wd4201 -wd4324 -wd4505 -Zi -O2 -Ob1\" -DNDEBUG")

#Set(CMAKE_CUDA_FLAGS_DEBUG "${CMAKE_CUDA_FLAGS_DEBUG} -lineinfo")
#Set(CMAKE_CUDA_FLAGS_RELEASE "${CMAKE_CUDA_FLAGS_RELEASE} -lineinfo")
#Set(CMAKE_CUDA_FLAGS_RELWITHDEBINFO "${CMAKE_CUDA_FLAGS_RELWITHDEBINFO} -lineinfo")

Set(CMAKE_CUDA_FLAGS_DEBUG "${CMAKE_CUDA_FLAGS_DEBUG} --extended-lambda")
Set(CMAKE_CUDA_FLAGS_RELEASE "${CMAKE_CUDA_FLAGS_RELEASE} --extended-lambda")
Set(CMAKE_CUDA_FLAGS_RELWITHDEBINFO "${CMAKE_CUDA_FLAGS_RELWITHDEBINFO} --extended-lambda")

Set(CMAKE_CUDA_FLAGS_DEBUG "${CMAKE_CUDA_FLAGS_DEBUG} -rdc=true")
Set(CMAKE_CUDA_FLAGS_RELEASE "${CMAKE_CUDA_FLAGS_RELEASE} -rdc=true")
Set(CMAKE_CUDA_FLAGS_RELWITHDEBINFO "${CMAKE_CUDA_FLAGS_RELWITHDEBINFO} -rdc=true")

#Set(CMAKE_CUDA_FLAGS_DEBUG "${CMAKE_CUDA_FLAGS_DEBUG} --ptxas-options=-v")
#Set(CMAKE_CUDA_FLAGS_RELEASE "${CMAKE_CUDA_FLAGS_RELEASE} --ptxas-options=-v")
#Set(CMAKE_CUDA_FLAGS_RELWITHDEBINFO "${CMAKE_CUDA_FLAGS_RELWITHDEBINFO} --ptxas-options=-v")

Set(CMAKE_CUDA_FLAGS_DEBUG "${CMAKE_CUDA_FLAGS_DEBUG} --device-debug")
Set(CMAKE_CUDA_FLAGS_RELWITHDEBINFO "${CMAKE_CUDA_FLAGS_RELWITHDEBINFO} --device-debug")

#Set(CMAKE_CUDA_FLAGS_DEBUG "${CMAKE_CUDA_FLAGS_DEBUG} -use_fast_math")
#Set(CMAKE_CUDA_FLAGS_RELEASE "${CMAKE_CUDA_FLAGS_RELEASE} -use_fast_math")
#Set(CMAKE_CUDA_FLAGS_RELWITHDEBINFO "${CMAKE_CUDA_FLAGS_RELWITHDEBINFO} -use_fast_math")

message ("cuda debug options ${CMAKE_CUDA_FLAGS_DEBUG}")
message ("cuda release options ${CMAKE_CUDA_FLAGS_RELEASE}")
message ("cuda release with debug options ${CMAKE_CUDA_FLAGS_RELWITHDEBINFO}")

add_definitions(-D_D_CUDA_EXPORT_DLL)
add_library(${projectName} SHARED ${CPP_SOURCE})

set_target_properties(${projectName} PROPERTIES CUDA_SEPARABLE_COMPILATION ON CUDA_RESOLVE_DEVICE_SYMBOLS ON)

#it seems this is the broken
#target_compile_options(${projectName} PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-gencode arch=compute_60 code=sm_60>)
#target_compile_options(${projectName} PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-gencode compute_60,sm_60>)
#target_compile_options(${projectName} PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-gencode arch=compute_52 code=sm_52>)

install(FILES ${HEADERS} DESTINATION include/${projectName})
install(FILES $<TARGET_PDB_FILE:${projectName}> DESTINATION bin OPTIONAL)
install(TARGETS ${projectName} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

if (NEWTON_BUILD_SANDBOX_DEMOS)
	add_custom_command(
		TARGET ${projectName} POST_BUILD
		COMMAND ${CMAKE_COMMAND}
		ARGS -E copy $<TARGET_FILE:${projectName}> ${PROJECT_BINARY_DIR}/../../../../applications/ndSandbox/${CMAKE_CFG_INTDIR}/$<TARGET_FILE_NAME:${projectName}>)
endif ()
