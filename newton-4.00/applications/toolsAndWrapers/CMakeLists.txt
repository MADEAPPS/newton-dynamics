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

if (NEWTON_BUILD_NEWTON_PYTHON)
	add_subdirectory(newtonPy)
endif()

if (NEWTON_BUILD_NEWTON_C_SHARP)
	add_subdirectory(newtonSharp)
endif()

if (NEWTON_BUILD_NEWTON_JAVA)
	add_subdirectory(newtonJava)
endif()

if (NEWTON_BUILD_ANDROID_STUDIO)
	add_subdirectory(newtonAndroid)
endif()
