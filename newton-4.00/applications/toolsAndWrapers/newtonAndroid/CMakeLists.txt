# Copyright (c) <2014-2017> <Newton Game Dynamics>
#
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

cmake_minimum_required(VERSION 3.9.0 FATAL_ERROR)

set (projectName "ndAndroidStudio")
message (${projectName})


set (srcPath "../../../sdk")
set (dstPath "${PROJECT_BINARY_DIR}/applications/toolsAndWrapers/newtonAndroid/ndNewton")

file(GLOB ANDROID_SOURCE_CORE ${srcPath}/dCore/*.h ${srcPath}/dCore/*.cpp)
file(GLOB ANDROID_SOURCE_XML ${srcPath}/dTinyxml/*.h ${srcPath}/dTinyxml/*.cpp)
file(GLOB ANDROID_SOURCE_COLLISION ${srcPath}/dCollision/*.h ${srcPath}/dCollision/*.cpp)

file(GLOB ANDROID_SOURCE_NEWTON 
	${srcPath}/dNewton/*.h 
	${srcPath}/dNewton/*.cpp
	${srcPath}/dNewton/dJoints/*.h
	${srcPath}/dNewton/dJoints/*.cpp
	${srcPath}/dNewton/dIkSolver/*.h
	${srcPath}/dNewton/dIkSolver/*.cpp
	${srcPath}/dNewton/dParticles/*.h
	${srcPath}/dNewton/dParticles/*.cpp
	${srcPath}/dNewton/dModels/*.h
	${srcPath}/dNewton/dModels/*.cpp
	${srcPath}/dNewton/dModels/dVehicle/*.h
	${srcPath}/dNewton/dModels/dVehicle/*.cpp
	${srcPath}/dNewton/dModels/dCharacter/*.h
	${srcPath}/dNewton/dModels/dCharacter/*.cpp
)

file(COPY "cmakeScript/CMakeLists.txt" DESTINATION "${dstPath}")

foreach(file IN LISTS ANDROID_SOURCE_CORE)
	#get_filename_component(name_ ${file} NAME)
	file(COPY "${file}" DESTINATION "${dstPath}/dCore")
endforeach()

foreach(file IN LISTS ANDROID_SOURCE_COLLISION )
	#get_filename_component(name ${file} NAME)
	file(COPY "${file}" DESTINATION "${dstPath}/dCollision")
endforeach()

foreach(file IN LISTS ANDROID_SOURCE_XML)
	#get_filename_component(name ${file} NAME)
	file(COPY "${file}" DESTINATION "${dstPath}/dTinyxml")
endforeach()

foreach(file IN LISTS ANDROID_SOURCE_NEWTON)
	#get_filename_component(name ${file} NAME)
	file(COPY "${file}" DESTINATION "${dstPath}/dNewton")
endforeach()
