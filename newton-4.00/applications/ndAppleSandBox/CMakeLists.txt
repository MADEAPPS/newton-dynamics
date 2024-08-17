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

set(CMAKE_XCODE_ATTRIBUTE_SWIFT_OBJC_INTEROP_MODE objcxx CACHE INTERNAL “”)

file(GLOB CPP_SOURCE *.swift)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/" FILES ${CPP_SOURCE})

add_executable(${projectName} ${CPP_SOURCE})

target_link_libraries (${projectName} ndNewton ndBrain)
    
target_compile_options(${projectName} PRIVATE "SHELL:-cxx-interoperability-mode=default")
target_compile_options(${projectName} PUBLIC "$<$<COMPILE_LANGUAGE:Swift>:-cxx-interoperability-mode=default>”)


