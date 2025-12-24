# Copyright (c) <2014-2017> <Newton Game Dynamics>
#
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

cmake_minimum_required(VERSION 3.10.0 FATAL_ERROR)

set (projectName "ndAndroidStudio")
message (${projectName})

if(MSVC OR MINGW)
	set (swig "${CMAKE_SOURCE_DIR}/thirdParty/swigwin/swigwin-4.4.1/swig.exe")
else()
	set (swig "/usr/bin/swig")
endif()

set (newtonSrcPath "../../../sdk")
set (androidProjectPath "${CMAKE_SOURCE_DIR}/applications/toolsAndWrapers/newtonAndroid")
set (dstPath "${PROJECT_BINARY_DIR}/androidApp")

macro(copyJavaAplication srcDir dstDir)
    file(GLOB templateFiles RELATIVE ${srcDir} ${srcDir}/*)

    foreach(templateFile ${templateFiles})
		set(srcTemplatePath ${srcDir}/${templateFile})
		if(IS_DIRECTORY ${srcTemplatePath})
			set(subDir ${dstDir}/${templateFile})
			file(MAKE_DIRECTORY ${subDir})
			copyJavaAplication(${srcTemplatePath} ${subDir})
		else()
			file(COPY ${srcTemplatePath} DESTINATION ${dstDir})
		endif()
    endforeach(templateFile)
endmacro(copyJavaAplication)

message (${dstPath})
file(REMOVE_RECURSE ${dstPath})
file(MAKE_DIRECTORY ${dstPath})
copyJavaAplication ("${androidProjectPath}/androidApp" ${dstPath})

set (dstNewtonPath "${dstPath}/app/src/main/cpp")
file(MAKE_DIRECTORY "${dstNewtonPath}")
file(COPY "cmakeScript/CMakeLists.txt" DESTINATION ${dstNewtonPath})

set (newtonJavaWrapper "${dstPath}/app/src/main/java/com/newton")
set (swigScript "${CMAKE_SOURCE_DIR}/applications/toolsAndWrapers/newtonAndroid")
file(MAKE_DIRECTORY ${newtonJavaWrapper})
execute_process(COMMAND ${swig} -c++ -package "com.newton" -outdir "${newtonJavaWrapper}" -java ${swigScript}/newton.i)


file(GLOB ANDROID_SOURCE_ROOT *.h *.cpp *.cxx)
file(GLOB ANDROID_SOURCE_CORE ${newtonSrcPath}/dCore/*.h ${newtonSrcPath}/dCore/*.cpp)
file(GLOB ANDROID_SOURCE_XML ${newtonSrcPath}/dTinyxml/*.h ${newtonSrcPath}/dTinyxml/*.cpp)
file(GLOB ANDROID_SOURCE_COLLISION ${newtonSrcPath}/dCollision/*.h ${newtonSrcPath}/dCollision/*.cpp)

file(GLOB ANDROID_SOURCE_NEWTON 
	${newtonSrcPath}/dNewton/*.h 
	${newtonSrcPath}/dNewton/*.cpp
	${newtonSrcPath}/dNewton/dJoints/*.h
	${newtonSrcPath}/dNewton/dJoints/*.cpp
	${newtonSrcPath}/dNewton/dIkSolver/*.h
	${newtonSrcPath}/dNewton/dIkSolver/*.cpp
	${newtonSrcPath}/dNewton/dParticles/*.h
	${newtonSrcPath}/dNewton/dParticles/*.cpp
	${newtonSrcPath}/dNewton/dModels/*.h
	${newtonSrcPath}/dNewton/dModels/*.cpp
	${newtonSrcPath}/dNewton/dModels/dVehicle/*.h
	${newtonSrcPath}/dNewton/dModels/dVehicle/*.cpp
	${newtonSrcPath}/dNewton/dModels/dCharacter/*.h
	${newtonSrcPath}/dNewton/dModels/dCharacter/*.cpp
)

foreach(file IN LISTS ANDROID_SOURCE_ROOT)
	file(COPY "${file}" DESTINATION "${dstNewtonPath}")
endforeach()

foreach(file IN LISTS ANDROID_SOURCE_CORE)
	#get_filename_component(name_ ${file} NAME)
	file(COPY "${file}" DESTINATION "${dstNewtonPath}/dCore")
endforeach()

foreach(file IN LISTS ANDROID_SOURCE_XML)
	#get_filename_component(name ${file} NAME)
	file(COPY "${file}" DESTINATION "${dstNewtonPath}/dTinyxml")
endforeach()

foreach(file IN LISTS ANDROID_SOURCE_COLLISION )
	#get_filename_component(name ${file} NAME)
	file(COPY "${file}" DESTINATION "${dstNewtonPath}/dCollision")
endforeach()

foreach(file IN LISTS ANDROID_SOURCE_NEWTON)
	#get_filename_component(name ${file} NAME)
	file(COPY "${file}" DESTINATION "${dstNewtonPath}/dNewton")
endforeach()
