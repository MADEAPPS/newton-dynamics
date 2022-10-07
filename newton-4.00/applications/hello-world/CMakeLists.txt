# Need 3.18+ because it adds the REQUIRED option for `find_library` call.
cmake_minimum_required(VERSION 3.18.0 FATAL_ERROR)

project("newton-hello-world")
message (${PROJECT_NAME})

# Where to find the installed Newton headers.
include_directories(/usr/local/include/ndNewton)

# Ensure the Newton libraries are installed.
find_library(NEWTON_LIB ndNewton REQUIRED)
find_library(AVX_LIB ndSolverAvx2 REQUIRED)

# Build the app and link it to the Newton libraries.
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries (${PROJECT_NAME} ${NEWTON_LIB} ${AVX_LIB} pthread)
