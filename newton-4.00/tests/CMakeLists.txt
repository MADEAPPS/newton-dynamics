cmake_minimum_required(VERSION 3.18)
project(newton_tests)

# ----------------------------------------------------------------------
# Newton Settings.
# ----------------------------------------------------------------------

include_directories(../sdk/dCore)
include_directories(../sdk/dNewton)
include_directories(../sdk/dCollision)
include_directories(../sdk/dNewton/dModels)
include_directories(../sdk/dNewton/dIkSolver)
include_directories(../sdk/dNewton/dParticles)
include_directories(../sdk/dNewton/dModels/dVehicle)

# ----------------------------------------------------------------------
# Google Test Settings.
# Adopted from https://google.github.io/googletest/quickstart-cmake.html
# ----------------------------------------------------------------------

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)

# If gtest isn't already available in the build, fetch it.
if(NOT TARGET gtest)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.12.1
  )

  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

enable_testing()
include(GoogleTest)

file(GLOB CPP_SOURCE *.cpp)

# ----------------------------------------------------------------------
# Compile the test cases into a single binary.
# ----------------------------------------------------------------------
add_executable(${PROJECT_NAME} ${CPP_SOURCE})

target_link_libraries(${PROJECT_NAME} GTest::gtest_main)
target_link_libraries(${PROJECT_NAME} ndNewton ndSolverAvx2)

if (NEWTON_ENABLE_CUDA_SOLVER)
	target_link_libraries (${PROJECT_NAME} ndSolverCuda)
endif()

if (MSVC)
	target_compile_options(${PROJECT_NAME} PRIVATE "/W4")
endif()

if (MSVC)
	set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "demos")
endif()