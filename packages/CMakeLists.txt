# Copyright (c) <2014> <Newton Game Dynamics>
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

# dContainers
file(GLOB dContainers_srcs dContainers/*.cpp)
add_library(dContainers ${dContainers_srcs})

target_include_directories(dContainers PUBLIC dContainers/)
target_link_libraries(dContainers dMath)


# dCustomJoints
file(GLOB dCustomJoints_srcs dCustomJoints/*.cpp)
add_library(dCustomJoints ${dCustomJoints_srcs})

target_include_directories(dCustomJoints PUBLIC dCustomJoints/)
target_link_libraries(dCustomJoints dContainers NewtonStatic)


# dMath
file(GLOB dMath_srcs dMath/*.cpp)
add_library(dMath ${dMath_srcs})

target_include_directories(dMath PUBLIC dMath/)


# dScene
file(GLOB dScene_srcs dScene/*.cpp)
add_library(dScene ${dScene_srcs})

target_include_directories(dScene PUBLIC dScene/)
target_link_libraries(dScene dMath dContainers NewtonStatic)


# dVisualDebuggerServer
file(GLOB dVisualDebuggerServer_srcs dVisualDebuggerServer/*.cpp)
add_library(dVisualDebuggerServer ${dVisualDebuggerServer_srcs})

target_include_directories(dVisualDebuggerServer PUBLIC dVisualDebuggerServer)
target_link_libraries(dVisualDebuggerServer dContainers dMath NewtonStatic)

find_package(ENet)
include_directories(${ENet_INCLUDE_DIRS})
target_link_libraries(dVisualDebuggerServer ${ENet_LIBRARY})


if (UNIX)
  if (BUILD_64)
    add_definitions(-D_POSIX_VER_64)
  else (BUILD_64)
    add_definitions(-D_POSIX_VER_64)
  endif (BUILD_64)
endif(UNIX)

if (CMAKE_COMPILER_IS_GNUCC)
  add_definitions(-msse  -msse2 -ffloat-store -ffast-math -freciprocal-math -funsafe-math-optimizations -fsingle-precision-constant -c -Wall -Wno-strict-aliasing) 
endif(CMAKE_COMPILER_IS_GNUCC)
