# Copyright (c) <2016> <Newton Game Dynamics>
#
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

# imgui
file(GLOB imgui_srcs imgui/*.cpp)
add_library(imgui ${imgui_srcs})
target_include_directories(imgui PUBLIC imgui/)


if (WIN32 OR APPLE)
  # tinyxml
  file(GLOB tinyxml_srcs tinyxml/*.cpp)
  add_library(tinyxml ${tinyxml_srcs})
  target_include_directories(tinyxml PUBLIC tinyxml/)

  # glfw
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  add_subdirectory(glfw/)
endif ()
