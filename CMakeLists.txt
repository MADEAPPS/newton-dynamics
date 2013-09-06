# The name of our project is "Newton". CMakeLists files in this project can
# refer to the root source directory of the project as ${HELLO_SOURCE_DIR} and
# to the root binary directory of the project as ${HELLO_BINARY_DIR}.
cmake_minimum_required (VERSION 2.8)

# Use relative paths
# This is mostly to reduce path size for command-line limits on windows
if(WIN32)
  # This seems to break Xcode projects so definitely don't enable on Apple builds
  set(CMAKE_USE_RELATIVE_PATHS true)
  set(CMAKE_SUPPRESS_REGENERATION true)
endif()

project(newtonbuild)

# Include necessary submodules
#CMAKE_MODULE_PATH 
#    tell CMake to search first in directories listed in CMAKE_MODULE_PATH when you use FIND_PACKAGE() or INCLUDE()
# maybe disabling include_directories
#set(CMAKE_MODULE_PATH
# "${newtonbuild_SOURCE_DIR}/CMake"
#  "${newtonbuild_SOURCE_DIR}/CMake/Utils"
#  "${newtonbuild_SOURCE_DIR}/CMake/Packages"
#)

#####################################################################
# Set up the basic build environment
#####################################################################

if (CMAKE_BUILD_TYPE STREQUAL "")
  # CMake defaults to leaving CMAKE_BUILD_TYPE empty. This screws up
  # differentiation between debug and release builds.
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: None (CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif ()

if (NOT APPLE)
  # Create debug libraries with _d postfix
  set(CMAKE_DEBUG_POSTFIX "_d")
endif ()

if (MSVC)
  if (CMAKE_BUILD_TOOL STREQUAL "nmake")
    # set variable to state that we are using nmake makefiles
	set(NMAKE TRUE)
  endif ()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:fast")
  # Enable intrinsics on MSVC in debug mode
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Oi")
  if (CMAKE_CL_64)
    # Visual Studio bails out on debug builds in 64bit mode unless
	# this flag is set...
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj")
	set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /bigobj")
  endif ()
  if (MSVC_VERSION GREATER 1600 OR MSVC_VERSION EQUAL 1600)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
  endif ()
endif ()

# Specify build paths
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${newtonbuild_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${newtonbuild_BINARY_DIR}/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${newtonbuild_BINARY_DIR}/bin")
if (WIN32 OR APPLE)
  if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    # We don't want to install in default system location, install is really for the SDK, so call it that
    set(CMAKE_INSTALL_PREFIX
	  "${newtonbuild_BINARY_DIR}/sdk" CACHE PATH "Newton SDK install directory prefix" FORCE
    )
  endif (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
endif(WIN32 OR APPLE)

###################################################################
# disable (useless) compiler warnings on project level
###################################################################
if(MSVC)
	add_definitions( /wd4786 /wd4503 /wd4251 /wd4275 /wd4290 /wd4661 /wd4996 /wd4127 /wd4100)
endif()

# Add include paths
#include_directories("${newtonbuild_SOURCE_DIR}/coreLibrary_300")

set (corePath ${newtonbuild_SOURCE_DIR}/coreLibrary_300/source)
set (packagesPath ${newtonbuild_SOURCE_DIR}/packages)
add_subdirectory ("${newtonbuild_SOURCE_DIR}/coreLibrary_300")
add_subdirectory ("${newtonbuild_SOURCE_DIR}/packages")

#file(GLOB_RECURSE NewtonLibSrc ${newtonbuild_SOURCE_DIR}/coreLibrary_300/source/*.cpp)
#file(GLOB_RECURSE NewtonLibH ${newtonbuild_SOURCE_DIR}/coreLibrary_300/source/*.h)
#set (LIB_TYPE STATIC)
#add_library(Newton ${LIB_TYPE} ${NewtonLibH} ${NewtonLibSrc})

