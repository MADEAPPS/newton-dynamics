# Copyright (c) <2009> <Newton Game Dynamics>
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely

cmake_minimum_required( VERSION 2.8 )
project (newtonSDK)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR})

set (corePath ${PROJECT_SOURCE_DIR}/coreLibrary_300/source)

include_directories ("${PROJECT_SOURCE_DIR}/packages") 
include_directories ("${PROJECT_SOURCE_DIR}/coreLibrary_300") 
add_subdirectory (packages)
add_subdirectory (coreLibrary_300)

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR})
#file(GLOB_RECURSE NewtonLibSrc ${CMAKE_SOURCE_DIR}/coreLibrary_300/source/*.cpp)
#file(GLOB_RECURSE NewtonLibH ${CMAKE_SOURCE_DIR}/coreLibrary_300/source/*.h)

#set (LIB_TYPE SHARED)
#add_library(Newton ${LIB_TYPE} ${NewtonLibH} ${NewtonLibSrc})

