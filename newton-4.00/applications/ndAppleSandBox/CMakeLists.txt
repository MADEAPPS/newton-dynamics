# This source file is part of the Swift open source project
#
# Copyright (c) 2023 Apple Inc. and the Swift project authors.
# Licensed under Apache License v2.0 with Runtime Library Exception
#
# See https://swift.org/LICENSE.txt for license information

cmake_minimum_required(VERSION 3.22)

set (projectName "ndSandbox")
message (${projectName})

file(GLOB CPP_SOURCE *.swift)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

project(${projectName} LANGUAGES Swift)
add_executable(${projectName} ${CPP_SOURCE})
target_link_libraries (${projectName} ndNewton ndBrain ndModel vhacd openfbx lodepng)

