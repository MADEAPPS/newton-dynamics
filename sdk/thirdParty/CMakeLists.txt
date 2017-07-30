# Copyright (c) <2016> <Newton Game Dynamics>
#
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

# timeTracker
file(GLOB timeTracker_srcs timeTracker/dTimeTracker.cpp)
add_library(timeTracker ${timeTracker_srcs})

target_include_directories(timeTracker PUBLIC timeTracker/)
target_link_libraries(timeTracker dMath dContainers)

# imgui
file(GLOB imgui_srcs imgui/*.cpp)
add_library(imgui ${imgui_srcs})

target_include_directories(imgui PUBLIC imgui/)
