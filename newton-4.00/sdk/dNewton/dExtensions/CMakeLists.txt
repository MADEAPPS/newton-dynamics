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

if(NEWTON_ENABLE_AVX2_SOLVER)
	message ("adding avx2 solver")
	add_subdirectory(dAvx2)
endif()

if (NEWTON_ENABLE_CUDA_SOLVER)
	message ("adding cuda solver")
	add_subdirectory(dCuda)
endif()
