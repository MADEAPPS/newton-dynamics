# Copyright (c) <2009> <Newton Game Dynamics>
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely

set (path ${PROJECT_SOURCE_DIR}/coreLibrary_300/source)

add_subdirectory ("${path}/ai")
add_subdirectory ("${path}/core")
add_subdirectory ("${path}/newton")
add_subdirectory ("${path}/physics")
add_subdirectory ("${path}/pthreads.2")

#
# for some reason I can not foguret out hwo to set depencies with CMake, I get the follwing error
# 
# CMake Error at coreLibrary_300/CMakeLists.txt:23 (target_link_libraries):
#    Attempt to add link library
#    "C:/Users/Julio/Desktop/newton-dynamics/coreLibrary_300/source/pthreads.2/dgThreads"
#    to target "dgCore" which is not built in this directory. 
#
# if anyone knows how to fix this, please let me know, for now dependencies will not work an teh lib may have to build one at a time
#  
# target_link_libraries (dgCore ${path}/pthreads.2/dgThreads)
# target_link_libraries (${path}/core/dgCore ${path}/pthreads.2/dgThreads)
# target_link_libraries (dgPhysics dgThreads dgCore)

