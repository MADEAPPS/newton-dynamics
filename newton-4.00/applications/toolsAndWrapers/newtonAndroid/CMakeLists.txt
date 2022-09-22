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


set (projectName "ndNewton")
message (${projectName})


set (srcPath "../../../sdk")
set (dstPath "${PROJECT_BINARY_DIR}/newtonAndroid/ndNewton")

file(COPY "cmakeScript/CMakeLists.txt" DESTINATION "${dstPath}")
file(COPY "${srcPath}/dCore" DESTINATION "${dstPath}")
file(COPY "${srcPath}/dNewton" DESTINATION "${dstPath}")
file(COPY "${srcPath}/dTinyxml" DESTINATION "${dstPath}")
file(COPY "${srcPath}/dCollision" DESTINATION "${dstPath}")

