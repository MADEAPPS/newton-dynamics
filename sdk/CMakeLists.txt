# Copyright (c) <2014-2017> <Newton Game Dynamics>
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

cmake_minimum_required(VERSION 2.8)

if(WIN32 OR NEWTON_DEMOS_SANDBOX)
  # Required dependencies
  add_subdirectory(thirdParty)
endif()

file(GLOB dgCore_srcs dgCore/*.cpp)               # low level core
file(GLOB dgMeshUtil_srcs dgMeshUtil/*.cpp)       # mesh geometry
file(GLOB dgNewton_srcs dgNewton/*.cpp)           # Newton engine
file(GLOB dgPhysics_srcs dgPhysics/*.cpp)         # physics
#file(GLOB dgTimeTracker_srcs dgTimeTracker/*.cpp) # time tracker

add_library(NewtonObj OBJECT ${dgCore_srcs} ${dgMeshUtil_srcs} ${dgNewton_srcs} ${dgPhysics_srcs}) #${dgTimeTracker_srcs}

target_include_directories(NewtonObj PRIVATE dgCore/)
target_include_directories(NewtonObj PRIVATE dgMeshUtil/)
target_include_directories(NewtonObj PRIVATE dgNewton/)
target_include_directories(NewtonObj PRIVATE dgPhysics/)
target_include_directories(NewtonObj PRIVATE dgTimeTracker/)

# create static library
add_library(NewtonStatic STATIC $<TARGET_OBJECTS:NewtonObj>)
target_include_directories(NewtonStatic INTERFACE dgNewton)

# create shared library
add_library(Newton SHARED $<TARGET_OBJECTS:NewtonObj>)
target_include_directories(Newton INTERFACE dgNewton)

if (UNIX)
  set_target_properties(NewtonStatic PROPERTIES OUTPUT_NAME Newton)
endif(UNIX)

# choose right place on UNIX
if (UNIX)
  include("GNUInstallDirs")
else (UNIX)
  set(CMAKE_INSTALL_LIBDIR "${CMAKE_INSTALL_PREFIX}/lib")
  set(CMAKE_INSTALL_INCLUDEDIR "${CMAKE_INSTALL_PREFIX}/include")
endif (UNIX)

install(TARGETS Newton DESTINATION "${CMAKE_INSTALL_LIBDIR}")
install(FILES dgNewton/Newton.h DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")

if (MSVC)
  set_source_files_properties(dgCore/dgTypes.cpp PROPERTIES COMPILE_FLAGS "/YcdgStdafx.h")
  set_source_files_properties(dgNewton/NewtonClass.cpp PROPERTIES COMPILE_FLAGS "/YcNewtonStdAfx.h")
  set_source_files_properties(dgPhysics/dgWorld.cpp PROPERTIES COMPILE_FLAGS "/YcdgPhysicsStdafx.h")

  set_target_properties(NewtonStatic PROPERTIES COMPILE_FLAGS "/YudgStdafx.h /YuNewtonStdAfx.h /YudgPhysicsStdafx.h")
  set_target_properties (NewtonStatic PROPERTIES COMPILE_DEFINITIONS "_NEWTON_STATIC_LIB;_WIN_32_VER;PTW32_BUILD;PTW32_STATIC_LIB;_CRT_SECURE_NO_WARNINGS")

  set_target_properties(Newton PROPERTIES COMPILE_FLAGS "/YudgStdafx.h /YuNewtonStdAfx.h /YudgPhysicsStdafx.h")
  set_target_properties (Newton PROPERTIES COMPILE_DEFINITIONS "_NEWTON_BUILD_DLL;_WIN_32_VER;PTW32_BUILD;PTW32_STATIC_LIB;_CRT_SECURE_NO_WARNINGS")
endif(MSVC)




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

target_include_directories(dCustomJoints PUBLIC dgTimeTracker/)
target_include_directories(dCustomJoints PUBLIC dCustomJoints/)
target_link_libraries(dCustomJoints dContainers NewtonStatic)

target_compile_definitions(dCustomJoints PRIVATE _CUSTOM_JOINTS_BUILD_DLL)

# dMath
file(GLOB dMath_srcs dMath/*.cpp)
add_library(dMath ${dMath_srcs})

target_include_directories(dMath PUBLIC dMath/)


# dScene
file(GLOB dScene_srcs dScene/*.cpp)
add_library(dScene ${dScene_srcs})

target_include_directories(dScene PUBLIC dScene/)
target_link_libraries(dScene dAnimation dMath dContainers NewtonStatic)

if(WIN32)
  target_link_libraries(dScene tinyxml)
endif(WIN32)


if (CMAKE_COMPILER_IS_GNUCC)
  add_definitions(-fpic -msse -msse3 -msse4 -mfpmath=sse -ffloat-store -ffast-math -freciprocal-math -funsafe-math-optimizations -fsingle-precision-constant -c -Wall -Wno-strict-aliasing)
endif(CMAKE_COMPILER_IS_GNUCC)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  add_definitions(-D_MACOSX_VER)
else(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  if (${UNIX})
    if (BUILD_64)
      add_definitions(-D_POSIX_VER_64)
    else (BUILD_64)
      add_definitions(-D_POSIX_VER)
    endif (BUILD_64)
  endif(${UNIX})
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
