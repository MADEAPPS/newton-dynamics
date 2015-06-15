# Copyright (c) <2009> <Newton Game Dynamics>
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely

set (projectName dgThreads)
include_directories ("${corePath}/pthreads.2") 

# add the thread core library library
file(GLOB_RECURSE sources pthread.c)
file(GLOB_RECURSE headers pthread.h implement.h semaphore.h sched.h)
add_library (${projectName} ${headers} ${sources})

if (MSVC)
  set_target_properties (${projectName} PROPERTIES COMPILE_DEFINITIONS "_WIN_32_VER;PTW32_BUILD;PTW32_STATIC_LIB;_CRT_SECURE_NO_WARNINGS")
endif(MSVC)
