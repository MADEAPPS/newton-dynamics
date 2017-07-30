# Copyright (c) <2014> <Newton Game Dynamics>
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

# dAnimation
file(GLOB dAnimation_srcs dAnimation/*.cpp)
add_library(dAnimation ${dAnimation_srcs})

target_include_directories(dAnimation PUBLIC dAnimation/)
target_link_libraries(dAnimation dMath dContainers)


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
target_link_libraries(dScene dAnimation dMath dContainers NewtonStatic)


if (UNIX)
  add_definitions(-D_POSIX_VER)
endif(UNIX)

if (CMAKE_COMPILER_IS_GNUCC)
  add_definitions(-msse  -msse2 -ffloat-store -ffast-math -freciprocal-math -funsafe-math-optimizations -fsingle-precision-constant -c -Wall -Wno-strict-aliasing) 
endif(CMAKE_COMPILER_IS_GNUCC)
