cmake_minimum_required(VERSION 3.18)
project(newton_tests)

# ----------------------------------------------------------------------
# Newton Settings.
# ----------------------------------------------------------------------

include_directories(../sdk/dCore)
include_directories(../sdk/dNewton)
include_directories(../sdk/dTinyxml)
include_directories(../sdk/dCollision)
include_directories(../sdk/dNewton/dJoints)
include_directories(../sdk/dNewton/dModels)
include_directories(../sdk/dNewton/dIkSolver)
include_directories(../sdk/dNewton/dParticles)
include_directories(../sdk/dNewton/dModels/dVehicle)
include_directories(../sdk/dNewton/dModels/dCharacter)

# ----------------------------------------------------------------------
# Google Test Settings.
# Adopted from https://google.github.io/googletest/quickstart-cmake.html
# ----------------------------------------------------------------------

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
include(GoogleTest)

# ----------------------------------------------------------------------
# Compile the test cases into a single binary.
# ----------------------------------------------------------------------
add_executable(
  ${PROJECT_NAME}
  world_test.cpp
  rigidBody_test.cpp
  staticBody_test.cpp
  instancing_test.cpp
  bodyNotifyCallback_test.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  GTest::gtest_main
)
target_link_libraries (${PROJECT_NAME} ndNewton ndSolverAvx2)

if (MSVC)
	target_compile_options(${PROJECT_NAME} PRIVATE "/W4")
endif()
