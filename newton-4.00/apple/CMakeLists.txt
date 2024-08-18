# This source file is part of the Swift open source project
#
# Copyright (c) 2023 Apple Inc. and the Swift project authors.
# Licensed under Apache License v2.0 with Runtime Library Exception
#
# See https://swift.org/LICENSE.txt for license information

cmake_minimum_required(VERSION 3.22)

set (projectName "ndSandbox")
message (${projectName})

project(${projectName} LANGUAGES CXX Swift)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

set(CMAKE_CXX_FLAGS "-x objective-c++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fms-extensions")

set(CMAKE_XCODE_ATTRIBUTE_SWIFT_OBJC_INTEROP_MODE objcxx CACHE INTERNAL “”)
set(CMAKE_SHARED_LIBRARY_CREATE_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_CREATE_CXX_FLAGS} -undefined dynamic_lookup")

#file(GLOB SOURCE_FILES *.swift, *.modulemap)
file(GLOB SOURCE_FILES *.swift)


source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${SOURCE_FILES})

add_executable(${projectName} ${SOURCE_FILES})

#target_link_libraries (${projectName} ndNewton ndBrain)
#target_link_libraries (${projectName} ndNewton)
    
target_compile_options(${projectName} PRIVATE "SHELL:-cxx-interoperability-mode=default")



