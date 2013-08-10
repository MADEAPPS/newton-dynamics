# Copyright (c) <2009> <Newton Game Dynamics>
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely

# add the core library
file(GLOB_RECURSE dgCoreSrc ${CMAKE_SOURCE_DIR}/coreLibrary_300/source/core/*.cpp)
file(GLOB_RECURSE dgCoreHeaders ${CMAKE_SOURCE_DIR}/coreLibrary_300/source/core/*.h)
add_library(dgCore ${dgCoreHeaders} ${dgCoreSrc})

# add the physic layer
file(GLOB_RECURSE dgPhysicsSrc ${CMAKE_SOURCE_DIR}/coreLibrary_300/source/physics/*.cpp ${CMAKE_SOURCE_DIR}/coreLibrary_300/source/meshUtil/*.cpp ${CMAKE_SOURCE_DIR}/coreLibrary_300/source/openCL/*.cpp)
file(GLOB_RECURSE dgPhysicsHeaders ${CMAKE_SOURCE_DIR}/coreLibrary_300/source/physics/*.h ${CMAKE_SOURCE_DIR}/coreLibrary_300/source/meshUtil/*.h ${CMAKE_SOURCE_DIR}/coreLibrary_300/source/openCL/*.h)
add_library(dgPhysics ${dgPhysicsHeaders} ${dgPhysicsSrc})
target_link_libraries (dgPhysics dgCore)

# add the c inteface 
file(GLOB_RECURSE NewtonSrc ${CMAKE_SOURCE_DIR}/coreLibrary_300/source/physics/*.cpp)
file(GLOB_RECURSE NewtonHeaders ${CMAKE_SOURCE_DIR}/coreLibrary_300/source/physics/*.h)
#set (LIB_TYPE SHARED)
#add_library(Newton ${LIB_TYPE} ${dgCoreHeaders} ${dgCoreSrc})
add_library(Newton ${NewtonSrc} ${NewtonHeaders})
target_link_libraries (Newton dgCore)
target_link_libraries (Newton dgPhysics)