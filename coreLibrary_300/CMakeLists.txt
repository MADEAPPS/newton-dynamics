# Copyright (c) <2014> <Newton Game Dynamics>
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

cmake_minimum_required(VERSION 2.8)

project(NewtonLib)

set(NEWTON_SOURCE ${PROJECT_SOURCE_DIR}/source)


file(GLOB dg_srcs source/core/*.cpp)         # low level library
file(GLOB physics_srcs source/physics/*.cpp) # physics engine
file(GLOB mesh_srcs source/meshUtil/*.cpp)   # mesh geometry
file(GLOB newton_srcs source/newton/*.cpp)   # Newton engine

if (WIN32)
  # modified pthread library
  file(GLOB_RECURSE pthread_srcs ${NEWTON_SOURCE}/pthreads.2/pthread.c)
else (WIN32)
  set(pthread_srcs "")
endif (WIN32)

add_library(NewtonObj OBJECT ${dg_srcs} ${physics_srcs} ${mesh_srcs} ${newton_srcs} ${pthread_srcs})


if (WIN32)
  # add modified pthread library
  target_include_directories(NewtonObj PRIVATE ${NEWTON_SOURCE}/pthreads.2)
endif (WIN32)

target_include_directories(NewtonObj PRIVATE ${NEWTON_SOURCE}/core)
target_include_directories(NewtonObj PRIVATE ${NEWTON_SOURCE}/physics)
target_include_directories(NewtonObj PRIVATE ${NEWTON_SOURCE}/meshUtil)
target_include_directories(NewtonObj INTERFACE ${NEWTON_SOURCE}/newton)

# create static library
add_library(NewtonStatic STATIC $<TARGET_OBJECTS:NewtonObj>)
target_include_directories(NewtonStatic INTERFACE ${NEWTON_SOURCE}/newton)

# create shared library
add_library(Newton SHARED $<TARGET_OBJECTS:NewtonObj>)
target_include_directories(Newton INTERFACE ${NEWTON_SOURCE}/newton)

if (UNIX)
  set_target_properties(NewtonStatic PROPERTIES OUTPUT_NAME Newton)
endif(UNIX)

# choose right place on UNIX
if (UNIX)
  include("GNUInstallDirs")
else (UNIX)
  set(CMAKE_INSTALL_LIBDIR "lib")
  set(CMAKE_INSTALL_INCLUDEDIR "include")
endif (UNIX)

install(TARGETS Newton DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES ${NEWTON_SOURCE}/newton/Newton.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})


if (${UNIX})
  if (BUILD_64)
    add_definitions(-D_POSIX_VER_64)
  else (BUILD_64)
    add_definitions(-D_POSIX_VER)
  endif (BUILD_64)
endif(${UNIX})

if (CMAKE_COMPILER_IS_GNUCC)
  add_definitions(-fpic -msse -msse3 -mfpmath=sse -ffloat-store -ffast-math -freciprocal-math -funsafe-math-optimizations -fsingle-precision-constant)
endif(CMAKE_COMPILER_IS_GNUCC)

if (MSVC)
  set_source_files_properties(${NEWTON_SOURCE}/core/dgTypes.cpp PROPERTIES COMPILE_FLAGS "/YcdgStdafx.h")
  set_source_files_properties(${NEWTON_SOURCE}/newton/NewtonClass.cpp PROPERTIES COMPILE_FLAGS "/YcNewtonStdAfx.h")
  set_source_files_properties(${NEWTON_SOURCE}/physics/dgWorld.cpp PROPERTIES COMPILE_FLAGS "/YcdgPhysicsStdafx.h")

  set_target_properties(NewtonStatic PROPERTIES COMPILE_FLAGS "/YudgStdafx.h /YuNewtonStdAfx.h /YudgPhysicsStdafx.h")
  set_target_properties (NewtonStatic PROPERTIES COMPILE_DEFINITIONS "_NEWTON_STATIC_LIB;_WIN_32_VER;PTW32_BUILD;PTW32_STATIC_LIB;_CRT_SECURE_NO_WARNINGS")

  set_target_properties(Newton PROPERTIES COMPILE_FLAGS "/YudgStdafx.h /YuNewtonStdAfx.h /YudgPhysicsStdafx.h")
  set_target_properties (Newton PROPERTIES COMPILE_DEFINITIONS "_NEWTON_BUILD_DLL;_WIN_32_VER;PTW32_BUILD;PTW32_STATIC_LIB;_CRT_SECURE_NO_WARNINGS")
endif(MSVC)
