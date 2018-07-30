# Copyright (c) <2014-2017> <Newton Game Dynamics>
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

project(demosSandbox)

# keep in demo folder otherwise media won't be found
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})

include_directories(sdkDemos/)
include_directories(sdkDemos/toolBox/)

file(GLOB_RECURSE sandbox_srcs RELATIVE ${CMAKE_CURRENT_LIST_DIR} sdkDemos/*.cpp)

# these can't be compiled
list(REMOVE_ITEM sandbox_srcs "sdkDemos/demos/SpecialTrackJoints.cpp")
list(REMOVE_ITEM sandbox_srcs "sdkDemos/toolBox/LevelPrimitive.cpp")
list(REMOVE_ITEM sandbox_srcs "sdkDemos/toolBox/ShaderPrograms.cpp")

add_executable(demosSandbox ${sandbox_srcs})
target_link_libraries(demosSandbox dMath dScene dCustomJoints dContainers dAnimation imgui)


# 3rd party dependencies

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})
target_link_libraries(demosSandbox ${OPENGL_LIBRARIES})

#find_package(OpenAL REQUIRED)
#include_directories(${OPENAL_INCLUDE_DIR})
#target_link_libraries(demosSandbox ${OPENAL_LIBRARY})

if (WIN32 OR APPLE)
  target_link_libraries(demosSandbox glfw tinyxml)
else ()
  find_package(TinyXML REQUIRED)
  include_directories(${TinyXML_INCLUDE_DIRS})
  target_link_libraries(demosSandbox ${TinyXML_LIBRARIES})

  find_package(glfw3 REQUIRED)
  target_link_libraries(demosSandbox glfw)
endif ()


if (UNIX)
  add_definitions(-DD_JOINTLIBRARY_STATIC_LIB)

  if (BUILD_64)
    add_definitions(-D_POSIX_VER_64)
  else (BUILD_64)
    add_definitions(-D_POSIX_VER)
  endif (BUILD_64)
endif(UNIX)

if (CMAKE_COMPILER_IS_GNUCC)
  add_definitions(-fpic -msse -msse2 -c -Wall -std=c++11) 
endif(CMAKE_COMPILER_IS_GNUCC)
 
