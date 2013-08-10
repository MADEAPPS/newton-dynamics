# Copyright (c) <2009> <Newton Game Dynamics>
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely

set (NewtonCorePath ${CMAKE_SOURCE_DIR}/coreLibrary_300/source)

include_directories ("${NewtonCorePath}/ai") 
include_directories ("${NewtonCorePath}/core") 
include_directories ("${NewtonCorePath}/newton") 
include_directories ("${NewtonCorePath}/openCL") 
include_directories ("${NewtonCorePath}/physics") 
include_directories ("${NewtonCorePath}/meshUtil") 
include_directories ("${NewtonCorePath}/pthreads.2") 


# add the thread core library library
file(GLOB_RECURSE dgThreadSrc ${NewtonCorePath}/pthreads.2/pthread.c)
file(GLOB_RECURSE dgThreadHeaders ${NewtonCorePath}/pthreads.2/pthread.h ${NewtonCorePath}/pthreads.2/implement.h ${NewtonCorePath}/pthreads.2/semaphore.h ${NewtonCorePath}/pthreads.2/sched.h)
add_library (dgThreads ${dgThreadSrc} ${dgThreadHeaders})

# add the core library
file(GLOB_RECURSE dgCoreSrc ${NewtonCorePath}/core/*.cpp)
file(GLOB_RECURSE dgCoreHeaders ${NewtonCorePath}/core/*.h)
add_library(dgCore ${dgCoreHeaders} ${dgCoreSrc})
target_link_libraries (dgCore dgThreads)


# add the physic layer
file(GLOB_RECURSE dgPhysicsSrc ${NewtonCorePath}/physics/*.cpp ${NewtonCorePath}/meshUtil/*.cpp)
file(GLOB_RECURSE dgPhysicsHeaders ${NewtonCorePath}/physics/*.h ${NewtonCorePath}/meshUtil/*.h)
#file(GLOB_RECURSE dgPhysicsSrc ${NewtonCorePath}/openCL/*.cpp)
#file(GLOB_RECURSE dgPhysicsHeaders ${NewtonCorePath}/openCL/*.h)
add_library(dgPhysics ${dgPhysicsHeaders} ${dgPhysicsSrc})
target_link_libraries (dgPhysics dgCore dgThreads)

# add the c inteface 
file(GLOB_RECURSE NewtonSrc ${NewtonCorePath}/newton/*.cpp)
file(GLOB_RECURSE NewtonHeaders ${NewtonCorePath}/newton/*.h)
#set (LIB_TYPE SHARED)
#add_library(Newton ${LIB_TYPE} ${dgCoreHeaders} ${dgCoreSrc})
add_library(Newton ${NewtonSrc} ${NewtonHeaders})
target_link_libraries (Newton dgThreads dgCore dgPhysics)

# add the core library
file(GLOB_RECURSE dgAiSrc ${NewtonCorePath}/ai/*.cpp)
file(GLOB_RECURSE dgAiHeaders ${NewtonCorePath}/ai/*.h)
add_library(dgAi ${dgAiHeaders} ${dgAiSrc})
target_link_libraries (dgAi Newton dgThreads dgCore dgPhysics)


if (MSVC)
  set_target_properties(dgCore PROPERTIES COMPILE_FLAGS "/YudgStdafx.h")
  set_source_files_properties(${NewtonCorePath}/core/dgTypes.cpp PROPERTIES COMPILE_FLAGS "/YcdgStdafx.h")

  set_target_properties(dgPhysics PROPERTIES COMPILE_FLAGS "/YudgPhysicsStdafx.h")
  set_source_files_properties(${NewtonCorePath}/physics/dgWorld.cpp PROPERTIES COMPILE_FLAGS "/YcdgPhysicsStdafx.h")

  set_target_properties(Newton PROPERTIES COMPILE_FLAGS "/YuNewtonStdAfx.h")
  set_source_files_properties(${NewtonCorePath}/newton/NewtonClass.cpp PROPERTIES COMPILE_FLAGS "/YcNewtonStdAfx.h")
  
  set_target_properties(dgAi PROPERTIES COMPILE_FLAGS "/YudgAIStdafx.h")
  set_source_files_properties(${NewtonCorePath}/ai/dgAIStdafx.cpp PROPERTIES COMPILE_FLAGS "/YcdgAIStdafx.h")
  
  set_target_properties (Newton PROPERTIES COMPILE_DEFINITIONS "_NEWTON_STATIC_LIB;_WIN_32_VER;PTW32_BUILD;PTW32_STATIC_LIB;_CRT_SECURE_NO_WARNINGS")
  set_target_properties (dgCore dgThreads dgPhysics dgAi PROPERTIES COMPILE_DEFINITIONS "_WIN_32_VER;PTW32_BUILD;PTW32_STATIC_LIB;_CRT_SECURE_NO_WARNINGS")
endif(MSVC)
