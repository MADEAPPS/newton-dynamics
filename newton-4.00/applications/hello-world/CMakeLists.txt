# Need 3.18+ because it adds the REQUIRED option for `find_library` call.
cmake_minimum_required(VERSION 3.18.0 FATAL_ERROR)

# ----------------------------------------------------------------------
# Common section for all applications.
# ----------------------------------------------------------------------

# Where `make install` put the Newton headers and libraries.
include_directories(/usr/local/include/ndNewton)
set(CMAKE_LIBRARY_PATH /usr/local/lib)

# Ensure the Newton libraries are installed.
find_library(NEWTON_LIB ndNewton REQUIRED)
find_library(AVX_LIB ndSolverAvx2 REQUIRED)

# ----------------------------------------------------------------------
# Build the individual applications.
# ----------------------------------------------------------------------

project("hello")
add_executable(${PROJECT_NAME} hello.cpp)
target_link_libraries (${PROJECT_NAME} ${NEWTON_LIB} ${AVX_LIB} pthread)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall)

project("transformCallback")
add_executable(${PROJECT_NAME} transformCallback.cpp)
target_link_libraries (${PROJECT_NAME} ${NEWTON_LIB} ${AVX_LIB} pthread)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall)

project("contactCallback")
add_executable(${PROJECT_NAME} contactCallback.cpp)
target_link_libraries (${PROJECT_NAME} ${NEWTON_LIB} ${AVX_LIB} pthread)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall)
